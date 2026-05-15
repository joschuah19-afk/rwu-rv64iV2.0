
// asCPUx.sv – Cache version (RV_NoPipelineCache)
//
// Differences from RV_NoPipeline/src/asCPUx.sv:
//   - Flat iBus / dBus ports removed; icpu_if + dcpu_if are fully active.
//   - FETCH1_ST: stays until icpu_if.ic_rvalid = 1  (variable-latency I-Cache)
//   - EXECLD_ST: stays until dcpu_if.dc_rvalid = 1  (variable-latency D-Cache / Scratchpad)
//   - ic_req: 1-cycle pulse in FETCH0_ST only
//   - dc_req: 1-cycle pulse in EXEC_ST for loads only (stores complete in EXEC)
//   - dc_wdata / dc_wstrb computed from func3 and addr[2:0] for stores
//   - result_s for RES_MEM = dcpu_if.dc_rdata (sign ext done by scratchpad / cache)
`timescale 1ns/1ps

import as_pack::*;

module as_cpux (input  logic                         clk_i,
               input  logic                          rst_i,
               input  logic tck_i,
               output logic [instr_width-1:0]        ir_o,
               input  logic dr_cap_i,
              // Scan Chain
               output logic                          sc01_tdo_o,
               input  logic                          sc01_tdi_i,
               input  logic                          sc01_shift_i,
               input  logic                          sc01_clock_i,
               // Cache interfaces (active in this version)
               as_icache_if.cpu                      icpu_if,
               as_dcache_if.cpu                      dcpu_if,
               // IRQ
               input logic [irq_total_num_ext_c-1:0] irq_ext_i
              );

  localparam int XLEN = reg_width;

  logic aluSrcB_s, regWr_s, jump_s, take_s;
  mux_a_t                    aluSrcA_s;
  result_src_t               resultSrcx_s;
  imm_src_t                  immSrcx_s;
  alu_op_t                   aluSela_s;
  br_op_t                    aluSelb_s;

  // instruction / data bus internals
  logic [iaddr_width-1:0]     iBusAddr_s;
  logic [reg_width-1:0]       dBusDataRd_s;
  logic [reg_width-1:0]       dBusDataWr_s;
  logic [daddr_width-1:0]     dBusAddr_s;
  logic                       dMemRd_s;
  logic                       dMemWr_s;

  // PC
  logic [iaddr_width-1:0] PCp4_s;
  logic [iaddr_width-1:0] PCbr_s;
  logic [iaddr_width-1:0] PCorRS1_s;

  logic [reg_width-1:0] immExt_s;
  logic [reg_width-1:0] srcA_s, regA_s;
  logic [reg_width-1:0] srcB_s;

  logic [reg_width-1:0] result_s;
  logic [reg_width-1:0] aluRes_s, aluCalcRes_s;

  logic and_in01_s, sc01_01_s, sc01_02_s, sc01_03_s;
  logic and_in02_s, and_out_s;

  // IRQ / CSR
  logic [63:0] csr_mepc_s, csr_mcause_s, csr_mtvec_s, csr_mstatus_s, csr_mie_s, csr_mip_s;
  logic [reg_width-1:0] csr_data_s;
  logic [reg_width-1:0] regfile_data_w_s;
  logic csr_mstatus_mpie, csr_mstatus_mie;
  logic regWr_final_s;
  logic trap_taken_s;
  logic irq_pending_s;

  logic [instr_width-1:0] ir_s;
  logic ir_valid_s;
  logic trap_illegal_instrx_s;

  // FSM
  typedef enum logic [1:0] {FETCH0_ST, FETCH1_ST, EXEC_ST, EXECLD_ST} statetype_t;
  statetype_t state_s, nextstate_s;
  logic fetch0_phase_s, fetch1_phase_s, exec_phase_s, execld_phase_s;
  logic instr_commit_s;
  logic trap_misaligned_s;
  logic load_pending_s;
  logic irq_ext_sync1_s, irq_ext_sync2_s;
  logic is_mret_s, is_mret_fetched_s;
  logic [iaddr_width-1:0] PC_s;
  logic [6:0] opcode_s;
  logic trap_illegal_s;
  logic mret_pending_s;
  logic gated_clk_s, clk_mux_s;

  // Store data / strobe computed from func3 and addr offset
  logic [63:0] dc_wdata_s;
  logic [7:0]  dc_wstrb_s;

  assign ir_o = ir_s;

  //--------------------------------------------
  // I-Cache interface
  //--------------------------------------------
  assign icpu_if.ic_addr  = PC_s;
  assign icpu_if.ic_req   = fetch0_phase_s;   // 1-cycle pulse in FETCH0
  assign icpu_if.ic_flush = 1'b0;

  //--------------------------------------------
  // D-Cache interface
  //--------------------------------------------
  // dc_req: 1-cycle pulse in EXEC for loads; stores also pulse here then proceed to FETCH0
  assign dcpu_if.dc_req   = exec_phase_s && (opcode_s == 7'b0000011 || opcode_s == 7'b0100011);
  assign dcpu_if.dc_addr  = dBusAddr_s;       // = aluRes_s
  assign dcpu_if.dc_wr    = dMemWr_s && exec_phase_s;
  assign dcpu_if.dc_size  = ir_s[14:12];      // func3 → size + sign
  assign dcpu_if.dc_wdata = dc_wdata_s;
  assign dcpu_if.dc_wstrb = dc_wstrb_s;
  assign dcpu_if.dc_flush = 1'b0;

  // Internal data-bus aliases
  assign iBusAddr_s   = PC_s;
  assign dBusAddr_s   = aluRes_s;
  assign dBusDataRd_s = dcpu_if.dc_rdata;     // sign/zero ext done by scratchpad / cache

  //--------------------------------------------
  // Store data and byte-enable generation
  //--------------------------------------------
  always_comb begin
    dc_wdata_s = dBusDataWr_s;
    dc_wstrb_s = 8'hFF;
    if (dMemWr_s && exec_phase_s) begin
      case (ir_s[14:12])
        3'b000: begin  // SB
          dc_wdata_s = {8{dBusDataWr_s[7:0]}};
          dc_wstrb_s = 8'h01 << aluRes_s[2:0];
        end
        3'b001: begin  // SH
          dc_wdata_s = {4{dBusDataWr_s[15:0]}};
          dc_wstrb_s = 8'h03 << {aluRes_s[2:1], 1'b0};
        end
        3'b010: begin  // SW
          dc_wdata_s = {2{dBusDataWr_s[31:0]}};
          dc_wstrb_s = 8'h0F << {aluRes_s[2], 2'b0};
        end
        default: begin  // SD
          dc_wdata_s = dBusDataWr_s;
          dc_wstrb_s = 8'hFF;
        end
      endcase
    end
  end

  //--------------------------------------------
  // PC
  //--------------------------------------------
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i)
      PC_s <= 64'h0000000000000000;
    else begin
      if(fetch0_phase_s) begin
        if(trap_taken_s)      PC_s <= {csr_mtvec_s[63:2], 2'b00};
        else if(mret_pending_s) PC_s <= csr_mepc_s;
        else if(take_s)        PC_s <= PCbr_s;
        else                   PC_s <= PCp4_s;
      end
    end
  end

  //--------------------------------------------
  // Instruction Register
  //--------------------------------------------
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i) begin
      ir_s       <= 32'h00000013;
      ir_valid_s <= 1'b0;
    end else begin
      if(fetch1_phase_s && icpu_if.ic_rvalid) begin
        if(!mret_pending_s) begin
          ir_s       <= icpu_if.ic_rdata;
          ir_valid_s <= 1'b1;
        end else begin
          ir_s       <= 32'h00000013;
          ir_valid_s <= 1'b0;
        end
      end else if(trap_taken_s) begin
        ir_s       <= 32'h00000013;
        ir_valid_s <= 1'b0;
      end else if(is_mret_s && exec_phase_s) begin
        ir_s       <= 32'h00000013;
        ir_valid_s <= 1'b0;
      end
    end
  end

  //--------------------------------------------
  // FSM – variable-latency FETCH1 and EXECLD
  //--------------------------------------------
  // Stall-FSM 1: nextstate CLC
  always_comb begin
    nextstate_s = state_s;
    case(state_s)
      FETCH0_ST: nextstate_s = FETCH1_ST;
      FETCH1_ST: nextstate_s = icpu_if.ic_rvalid ? EXEC_ST  : FETCH1_ST;
      EXEC_ST:   nextstate_s = load_pending_s     ? EXECLD_ST : FETCH0_ST;
      EXECLD_ST: nextstate_s = dcpu_if.dc_rvalid  ? FETCH0_ST : EXECLD_ST;
      default:   nextstate_s = FETCH0_ST;
    endcase
  end

  // Stall-FSM 2: state register
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i) state_s <= FETCH0_ST;
    else       state_s <= nextstate_s;
  end

  // Stall-FSM 3: phase decode
  assign fetch0_phase_s = (state_s == FETCH0_ST);
  assign fetch1_phase_s = (state_s == FETCH1_ST);
  assign exec_phase_s   = (state_s == EXEC_ST);
  assign execld_phase_s = (state_s == EXECLD_ST);

  //--------------------------------------------
  // Commit and load detection
  //--------------------------------------------
  assign instr_commit_s = (exec_phase_s && !load_pending_s) || (execld_phase_s && dcpu_if.dc_rvalid);
  assign load_pending_s = (opcode_s == 7'b0000011) && exec_phase_s;

  //--------------------------------------------
  // PCp4, branch target, JALR mux
  //--------------------------------------------
  assign PCp4_s    = PC_s + 64'd4;
  assign PCorRS1_s = jump_s ? regA_s : iBusAddr_s;
  assign PCbr_s    = PCorRS1_s + immExt_s;

  //--------------------------------------------
  // Register file
  //--------------------------------------------
  assign regfile_data_w_s = result_s;
  assign regWr_final_s    = regWr_s && !trap_taken_s && instr_commit_s;

  as_regfile regfile (.clk_i(clk_i),
                      .rst_i(rst_i),
                      .we_i(regWr_final_s),
                      .raddr01_i(ir_s[19:15]),
                      .raddr02_i(ir_s[24:20]),
                      .waddr01_i(ir_s[11:7]),
                      .wdata01_i(regfile_data_w_s),
                      .rdata01_o(regA_s),
                      .rdata02_o(dBusDataWr_s)
                     );

  //--------------------------------------------
  // Immediate generation
  //--------------------------------------------
  always_comb
    case(immSrcx_s)
      IMM_I    : immExt_s = {{(XLEN-12){ir_s[31]}}, ir_s[31:20]};
      IMM_S    : immExt_s = {{(XLEN-12){ir_s[31]}}, ir_s[31:25], ir_s[11:7]};
      IMM_B    : immExt_s = {{(XLEN-12){ir_s[31]}}, ir_s[7], ir_s[30:25], ir_s[11:8], 1'b0};
      IMM_J    : immExt_s = {{(XLEN-20){ir_s[31]}}, ir_s[19:12], ir_s[20], ir_s[30:21], 1'b0};
      IMM_U    : immExt_s = {{(XLEN-32){1'b0}}, ir_s[31:12], 12'b0};
      IMM_NONE : immExt_s = {reg_width{1'b0}};
      default  : immExt_s = {reg_width{1'b0}};
    endcase

  //--------------------------------------------
  // ALU
  //--------------------------------------------
  assign srcB_s = aluSrcB_s ? immExt_s : dBusDataWr_s;

  always_comb
    case(aluSrcA_s)
      SRC_REGA : srcA_s = regA_s;
      SRC_PC   : srcA_s = PC_s;
      SRC_ZERO : srcA_s = {reg_width{1'b0}};
      default  : srcA_s = regA_s;
    endcase

  as_alu alua (.data01_i(srcA_s), .data02_i(srcB_s), .alu_op_i(aluSela_s), .aluResult_o(aluRes_s));
  assign aluCalcRes_s = aluRes_s;

  as_alu_branch alub (.data01_i(srcA_s), .data02_i(srcB_s), .br_op_i(aluSelb_s), .take_o(take_s));

  //--------------------------------------------
  // Result mux → register file
  //--------------------------------------------
  always_comb
    case(resultSrcx_s)
      RES_ALU : result_s = aluCalcRes_s;
      RES_MEM : result_s = dBusDataRd_s;     // from scratchpad / D-Cache
      RES_PC4 : result_s = PCp4_s;
      RES_CSR : result_s = csr_data_s;
      default : result_s = {reg_width{1'b0}};
    endcase

  //--------------------------------------------
  // Instruction decoder
  //--------------------------------------------
  as_instr_decode control (
    .instr_opcode_i(ir_s[6:0]),
    .instr_func3_i(ir_s[14:12]),
    .instr_func7b5_i(ir_s[30]),
    .take_i(take_s),
    .mux_resultSrc_o(resultSrcx_s),
    .en_dMemWr_o(dMemWr_s),
    .en_dMemRd_o(dMemRd_s),
    .mux_aluSrcB_o(aluSrcB_s),
    .mux_aluSrcA_o(aluSrcA_s),
    .en_regWr_o(regWr_s),
    .mux_jump_o(jump_s),
    .sel_immSrc_o(immSrcx_s),
    .alu_op_o(aluSela_s),
    .br_op_o(aluSelb_s),
    .trap_illegal_instr_o(trap_illegal_instrx_s)
  );

  //--------------------------------------------
  // IRQ
  //--------------------------------------------
  assign is_mret_s = (ir_s[6:0] == 7'b1110011) && (ir_s[14:12] == 3'b000) && (ir_s[31:20] == 12'h302);
  // ic_rdata is not valid during EXEC in the cache version; use ir_s directly
  assign is_mret_fetched_s = is_mret_s;

  // 2-FF synchronizer
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i) begin
      irq_ext_sync1_s <= 1'b0;
      irq_ext_sync2_s <= 1'b0;
    end else begin
      irq_ext_sync1_s <= irq_ext_i[7];
      irq_ext_sync2_s <= irq_ext_sync1_s;
    end
  end

  assign irq_pending_s = csr_mip_s[11] && csr_mie_s[11] && csr_mstatus_mie;
  assign opcode_s      = ir_s[6:0];
  assign trap_illegal_s = trap_illegal_instrx_s;

  // Misalignment
  always_comb begin
    trap_misaligned_s = 1'b0;
    if(ir_valid_s && exec_phase_s) begin
      if(opcode_s == 7'b0000011 || opcode_s == 7'b0100011) begin
        case(ir_s[14:12])
          3'b000, 3'b100: trap_misaligned_s = 1'b0;
          3'b001, 3'b101: trap_misaligned_s = (dBusAddr_s[0] != 1'b0);
          3'b010:         trap_misaligned_s = (dBusAddr_s[1:0] != 2'b00);
          3'b011:         trap_misaligned_s = (dBusAddr_s[2:0] != 3'b000);
          default:        trap_misaligned_s = 1'b0;
        endcase
      end
      if(opcode_s == 7'b1100011 || opcode_s == 7'b1101111 || opcode_s == 7'b1100111) begin
        if(take_s && PCbr_s[1:0] != 2'b00)
          trap_misaligned_s = 1'b1;
      end
    end
  end

  // MRET pending
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i)                             mret_pending_s <= 1'b0;
    else if(exec_phase_s && is_mret_fetched_s) mret_pending_s <= 1'b1;
    else if(fetch0_phase_s)               mret_pending_s <= 1'b0;
  end

  // Trap taken
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i)
      trap_taken_s <= 1'b0;
    else if(instr_commit_s && !is_mret_s) begin
      if(trap_illegal_s)        trap_taken_s <= 1'b1;
      else if(trap_misaligned_s) trap_taken_s <= 1'b1;
      else if(irq_pending_s)    trap_taken_s <= 1'b1;
      else                      trap_taken_s <= 1'b0;
    end else
      trap_taken_s <= 1'b0;
  end

  // MIP
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i) csr_mip_s <= 64'h0;
    else      csr_mip_s[11] <= irq_ext_sync2_s;
  end

  // MIE
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i == 1)
      csr_mie_s <= 64'h0000000000000800;
    else begin
      if(exec_phase_s && ir_valid_s)
        if((ir_s[6:0] == 7'b1110011) && (ir_s[31:20] == 12'h304))
          case(ir_s[14:12])
            3'b001: csr_mie_s <= regA_s;
            3'b010: csr_mie_s <= csr_mie_s | regA_s;
            3'b011: csr_mie_s <= csr_mie_s & ~regA_s;
            3'b101: csr_mie_s <= {{52{1'b0}}, ir_s[19:15], 7'b0};
            3'b110: csr_mie_s <= csr_mie_s | {{52{1'b0}}, ir_s[19:15], 7'b0};
            3'b111: csr_mie_s <= csr_mie_s & ~{{52{1'b0}}, ir_s[19:15], 7'b0};
            default: csr_mie_s <= regA_s;
          endcase
    end
  end

  // MSTATUS
  assign csr_mstatus_mpie = csr_mstatus_s[7];
  assign csr_mstatus_mie  = csr_mstatus_s[3];

  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i == 1)
      csr_mstatus_s <= 64'h0000000000001808;
    else begin
      if(trap_taken_s) begin
        csr_mstatus_s[7] <= csr_mstatus_mie;
        csr_mstatus_s[3] <= 0;
      end else if(is_mret_s && exec_phase_s) begin
        csr_mstatus_s[3] <= csr_mstatus_mpie;
        csr_mstatus_s[7] <= 1'b1;
      end else if(exec_phase_s && ir_valid_s)
        if((ir_s[6:0] == 7'b1110011) && (ir_s[31:20] == 12'h300))
          case(ir_s[14:12])
            3'b001: csr_mstatus_s <= regA_s;
            3'b010: csr_mstatus_s <= csr_mstatus_s | regA_s;
            3'b011: csr_mstatus_s <= csr_mstatus_s & ~regA_s;
            3'b101: csr_mstatus_s <= {{52{1'b0}}, ir_s[19:15], 7'b0};
            3'b110: csr_mstatus_s <= csr_mstatus_s | {{52{1'b0}}, ir_s[19:15], 7'b0};
            3'b111: csr_mstatus_s <= csr_mstatus_s & ~{{52{1'b0}}, ir_s[19:15], 7'b0};
            default: csr_mie_s <= regA_s;
          endcase
    end
  end

  // MEPC
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i)
      csr_mepc_s <= 0;
    else begin
      if(trap_taken_s) begin
        if(trap_illegal_s || trap_misaligned_s)
          csr_mepc_s <= PC_s;
        else
          csr_mepc_s <= PCp4_s;
      end else if(exec_phase_s && ir_valid_s)
        if((ir_s[6:0] == 7'b1110011) && (ir_s[31:20] == 12'h341))
          case(ir_s[14:12])
            3'b001: csr_mepc_s <= regA_s;
            3'b010: csr_mepc_s <= csr_mepc_s | regA_s;
            3'b011: csr_mepc_s <= csr_mepc_s & ~regA_s;
            3'b101: csr_mepc_s <= {{52{1'b0}}, ir_s[19:15], 7'b0};
            3'b110: csr_mepc_s <= csr_mepc_s | {{52{1'b0}}, ir_s[19:15], 7'b0};
            3'b111: csr_mepc_s <= csr_mepc_s & ~{{52{1'b0}}, ir_s[19:15], 7'b0};
            default: csr_mie_s <= regA_s;
          endcase
    end
  end

  // MTVEC
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i == 1)
      csr_mtvec_s <= 64'h0000000000007F00;
    else if(exec_phase_s && ir_valid_s)
      if((ir_s[6:0] == 7'b1110011) && (ir_s[31:20] == 12'h305))
        case(ir_s[14:12])
          3'b001: csr_mtvec_s <= regA_s;
          3'b010: csr_mtvec_s <= csr_mtvec_s | regA_s;
          3'b011: csr_mtvec_s <= csr_mtvec_s & ~regA_s;
          3'b101: csr_mtvec_s <= {{52{1'b0}}, ir_s[19:15], 7'b0};
          3'b110: csr_mtvec_s <= csr_mtvec_s | {{52{1'b0}}, ir_s[19:15], 7'b0};
          3'b111: csr_mtvec_s <= csr_mtvec_s & ~{{52{1'b0}}, ir_s[19:15], 7'b0};
          default: csr_mie_s <= regA_s;
        endcase
  end

  // MCAUSE
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i)
      csr_mcause_s <= 64'h0;
    else if(trap_taken_s) begin
      if(trap_illegal_s)         csr_mcause_s <= 64'd2;
      else if(trap_misaligned_s) begin
        if(dMemRd_s)       csr_mcause_s <= 64'd4;
        else if(dMemWr_s)  csr_mcause_s <= 64'd6;
        else               csr_mcause_s <= 64'd0;
      end else if(irq_pending_s) csr_mcause_s <= {1'b1, 63'd11};
    end
  end

  // CSR read mux
  always_comb begin
    csr_data_s = 64'h0;
    if((ir_s[6:0] == 7'b1110011) && (ir_s[14:12] != 3'b000))
      case(ir_s[31:20])
        12'h300: csr_data_s = csr_mstatus_s;
        12'h304: csr_data_s = csr_mie_s;
        12'h305: csr_data_s = csr_mtvec_s;
        12'h341: csr_data_s = csr_mepc_s;
        12'h342: csr_data_s = csr_mcause_s;
        12'h344: csr_data_s = csr_mip_s;
        default: csr_data_s = 64'h0;
      endcase
  end

  //--------------------------------------------
  // Test Scan Chain
  //--------------------------------------------
  assign clk_mux_s  = (sc01_shift_i == 1) ? tck_i : gated_clk_s;
  assign gated_clk_s = clk_i && dr_cap_i;

  scan_cell sc01 (clk_mux_s, rst_i, sc01_shift_i, 1'b0, sc01_tdi_i, and_in01_s, sc01_01_s);
  scan_cell sc02 (clk_mux_s, rst_i, sc01_shift_i, 1'b0, sc01_01_s, and_in02_s, sc01_02_s);
  assign and_out_s = and_in01_s & and_in02_s;
  scan_cell sc03 (clk_mux_s, rst_i, sc01_shift_i, and_out_s, sc01_02_s, , sc01_03_s);
  scan_cell sc04 (clk_mux_s, rst_i, sc01_shift_i, 1'b0, sc01_03_s, , sc01_tdo_o);

endmodule : as_cpux
