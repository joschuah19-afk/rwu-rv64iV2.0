
// asCPUx.sv
`timescale 1ns/1ps

import as_pack::*;

module as_cpux (input  logic                         clk_i,
               input  logic                          rst_i,
               input  logic tck_i,
               output logic [instr_width-1:0]        ir_o,         // needed for byte-select logic in D-Mem
               input  logic dr_cap_i,
              // Scan Chain
               output logic                          sc01_tdo_o,   // scan: serial out
               input  logic                          sc01_tdi_i,   // scan: serial in
               input  logic                          sc01_shift_i, // scan: shift enable
               input  logic                          sc01_clock_i, // scan: clock enabe
               // Instruction bus (legacy flat BPI)
               output logic [iaddr_width-1:0]        iBusAddr_o,     // I-Bus: address
               input  logic [instr_width-1:0]        iBusDataRd_i,   // I-Bus: data
               // Data bus (legacy flat BPI)
               output logic [daddr_width-1:0]        dBusAddr_o,     // address for dmem
               output logic [reg_width-1:0]          dBusDataWr_o,   // data to dmem
               input  logic [reg_width-1:0]          dBusDataRd_i,   // data from dmem
               output logic                          dBusWe_o,       // write enable fo dmem
               // Cache interfaces (disabled: tied to 0 until as_top_mem connects asMemTop)
               as_icache_if.cpu                      icpu_if,
               as_dcache_if.cpu                      dcpu_if,
               // IRQ
               input logic [irq_total_num_ext_c-1:0] irq_ext_i // External interrupts, irq_ext_i[7] = GPIO
              );

  localparam int XLEN = reg_width;

  // Umbau
  logic aluSrcB_s,regWr_s,jump_s,take_s; //,PCsrc_s;
  mux_a_t                    aluSrcA_s;
  result_src_t               resultSrcx_s;
  imm_src_t                  immSrcx_s;
  alu_op_t                   aluSela_s;
  br_op_t                    aluSelb_s;

  // instruction
  logic [instr_width-1:0]     iBusDataRd_s; // data out from imem
  logic [iaddr_width-1:0]     iBusAddr_s;   // address for imem

  // data
  logic [reg_width-1:0]       dBusDataRd_s; // data out from dmem
  logic [reg_width-1:0]       dBusDataWr_s; // data in to dmem
  logic [daddr_width-1:0]     dBusAddr_s;   // address for dmem
  logic                       dMemRd_s;     // read enable for dmem
  logic                       dMemWr_s;     // write enable for dmem

  // PC
  logic [iaddr_width-1:0] PCp4_s;   // linear code
  logic [iaddr_width-1:0] PCbr_s;   // branch target; PCTarget
  logic	[iaddr_width-1:0] PCorRS1_s;

  // Immediate extention
  logic [reg_width-1:0] immExt_s;
  // Register file
  logic [reg_width-1:0] srcA_s, regA_s;
  logic [reg_width-1:0] srcB_s;

  // D-Mem
  logic [reg_width-1:0] result_s;
  // ALU
  logic [reg_width-1:0]	aluRes_s, aluCalcRes_s;

  logic	and_in01_s;
  logic	sc01_01_s;
  logic	sc01_02_s;
  logic	sc01_03_s;
  logic	and_in02_s;
  logic	and_out_s;

  //IRQ
  logic [63:0] csr_mepc_s;
  logic [63:0] csr_mcause_s;
  logic [63:0] csr_mtvec_s;
  logic [63:0] csr_mstatus_s;
  logic [63:0] csr_mie_s;
  logic [63:0] csr_mip_s;

  logic [reg_width-1:0] csr_data_s;
  logic [reg_width-1:0] regfile_data_w_s;
  logic			csr_mstatus_mpie;
  logic			csr_mstatus_mie;
  
  logic	regWr_final_s;
  
  logic	trap_taken_s;
  
  logic	irq_pending_s;

  logic [instr_width-1:0] ir_s; // instruction register
  logic ir_valid_s;
  logic	trap_illegal_instrx_s;

  // FSM for pipeline timing
  typedef enum logic [1:0] {FETCH0_ST, FETCH1_ST, EXEC_ST, EXECLD_ST} statetype_t;
  statetype_t state_s, nextstate_s;
  logic fetch0_phase_s;   // address to I-Mem
  logic fetch1_phase_s;   // data from I-Mem
  logic exec_phase_s;     // IR ausführen
  logic execld_phase_s;     // IR ausführen
    
  logic instr_commit_s;

  logic	trap_misaligned_s;
  
  logic load_pending_s;
  
  logic irq_ext_sync1_s;
  logic irq_ext_sync2_s;
  
  logic is_mret_s;          // Current instruction is MRET
  logic [iaddr_width-1:0] PC_s;

  logic [6:0] opcode_s;
  logic trap_illegal_s;
  logic	mret_pending_s;
  logic is_mret_fetched_s;  // MRET detected in fetch1 (from I-Mem)

  logic	gated_clk_s;
  logic	clk_mux_s;
  
  

  assign ir_o = ir_s; // instruction register

  //--------------------------------------------
  // Master BPI Instruction Bus
  //--------------------------------------------
  assign iBusAddr_o = iBusAddr_s;
  assign iBusDataRd_s = iBusDataRd_i;

  //--------------------------------------------
  // Master BPI Data Bus
  //--------------------------------------------
  //assign dbpi_req_s = 1'b1; // kann weg!!

  assign dBusWe_o     = dMemWr_s && exec_phase_s;
  assign dBusAddr_o   = dBusAddr_s;
  assign dBusDataWr_o = dBusDataWr_s;
  assign dBusDataRd_s = dBusDataRd_i;

  //--------------------------------------------
  // PC, Program Counter and IR (Instruction Register)
  //--------------------------------------------
  assign iBusAddr_s = PC_s;
  
  //--------------------------------------------
  // ... PC itself:
  // PC represents the last committed architectural instruction
  // and is only advanced on instr_commit_s
  //--------------------------------------------
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i)
      PC_s <= 64'h0000000000000000;
    else
    begin
      if(fetch0_phase_s)
      begin
        if(trap_taken_s)
          PC_s <= {csr_mtvec_s[63:2], 2'b00};  // Jump to trap vector
        else
          if(mret_pending_s)
            PC_s <= csr_mepc_s;                // Return from trap
          else 
            if(take_s)
              PC_s <= PCbr_s;  // Branch/Jump
            else
              PC_s <= PCp4_s;  // Sequential
      end
    end
  end

  //===========================================
  // Instruction Register
  //===========================================
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i)
    begin
      ir_s <= 32'h00000013;  // NOP
      ir_valid_s <= 1'b0;
    end
    else
    begin
      if(fetch1_phase_s)
      begin
        // Don't load instruction if MRET is pending (PC is changing)
        if(!mret_pending_s)
        begin
          ir_s <= iBusDataRd_s;
          ir_valid_s <= 1'b1;
        end
        else
        begin
          ir_s <= 32'h00000013;  // NOP during MRET
          ir_valid_s <= 1'b0;
        end
      end
      else
        if(trap_taken_s)
        begin
          ir_s <= 32'h00000013;  // NOP after trap
          ir_valid_s <= 1'b0;
        end
        else
          if(is_mret_s && exec_phase_s)
          begin
            // Invalidate after MRET execution
            ir_s <= 32'h00000013;  // NOP
            ir_valid_s <= 1'b0;
          end
    end
  end
  
  //--------------------------------------------
  // Establishes working phases because of PC-IR pipeline (nothing more than a simple 3-counter so far)
  //
  // Stall-FSM 1: nextstate CLC
  always_comb 
  begin
    nextstate_s = state_s;
    case(state_s)
      FETCH0_ST  : nextstate_s = FETCH1_ST;
      FETCH1_ST  : nextstate_s = EXEC_ST;
      EXEC_ST    : if(load_pending_s)
                     nextstate_s = EXECLD_ST;
                   else
                     nextstate_s = FETCH0_ST;
      EXECLD_ST  : nextstate_s = FETCH0_ST;
      default    : nextstate_s = FETCH0_ST;
    endcase
  end // always_comb
  
  // Stall-FSM 2: delay
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
      state_s <= FETCH0_ST;
    else
      state_s <= nextstate_s;
  end
  
  // Stall-FSM 3: output CLC
  assign fetch0_phase_s = (state_s == FETCH0_ST) ? 1 : 0;
  assign fetch1_phase_s = (state_s == FETCH1_ST) ? 1 : 0;
  assign exec_phase_s   = (state_s == EXEC_ST)   ? 1 : 0;
  assign execld_phase_s = (state_s == EXECLD_ST)   ? 1 : 0;
  //--------------------------------------------
  
 
  
  //--------------------------------------------
  // ... indicates, when a regular instruction should be executed
  //--------------------------------------------
  //assign instr_commit_s = exec_phase_s && ir_valid_s && !flush_fetch_s && !load_pending_s;
  assign instr_commit_s = (exec_phase_s && !load_pending_s) || execld_phase_s;
  // Load detection
  assign load_pending_s = (opcode_s == 7'b0000011) && exec_phase_s;
  
  //--------------------------------------------
  // Adder +4 for the address of the next instruction
  //--------------------------------------------
  assign PCp4_s = PC_s + 64'd4;

  //--------------------------------------------
  // Mux for jumps of jalr instruction or normal branches.
  //         - pc_o   : jalr
  //         - regA_s : normal branch
  //--------------------------------------------
  assign PCorRS1_s = jump_s ? regA_s : iBusAddr_s;

  //--------------------------------------------
  // Adder for the branch targets
  //--------------------------------------------
  assign PCbr_s = PCorRS1_s + immExt_s;

  //--------------------------------------------
  // Mux for the PC, either +4 or branch target
  //--------------------------------------------
  //assign PCnext_mux_s = PCsrc_s ? PCbr_s : PCp4_s;

  //--------------------------------------------
  // Register file
  //--------------------------------------------
  // CSR or result
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
      IMM_I    : immExt_s = {{(XLEN-12){ir_s[31]}},ir_s[31:20]}; // I-type: sign ext, immediate (12 b)
      IMM_S    : immExt_s = {{(XLEN-12){ir_s[31]}},ir_s[31:25],ir_s[11:7]}; // S-type: sign ext, immediate1 (7 b), immediate2 (5 b)
      IMM_B    : immExt_s = {{(XLEN-12){ir_s[31]}},ir_s[7],ir_s[30:25],ir_s[11:8],1'b0}; // B-type: sign ext, imm1 (1 b), imm2 (6 b), imm3 (4 b), *2
      IMM_J    : immExt_s = {{(XLEN-20){ir_s[31]}},ir_s[19:12],ir_s[20],ir_s[30:21],1'b0}; // J-type: sign ext, imm1 (8 b), imm2 (1 b), imm3 (10 b), *2
      IMM_U    : immExt_s = {{(XLEN-32){1'b0}}, ir_s[31:12], 12'b0}; // // U-type: zero ext, imm, zero; lui, auipc
      IMM_NONE : immExt_s = {reg_width{1'b0}};
      default  : immExt_s = {reg_width{1'b0}};
    endcase

  //--------------------------------------------
  // ALU: input mux for regB or immediate
  //--------------------------------------------
  assign srcB_s = aluSrcB_s ? immExt_s : dBusDataWr_s;

  //--------------------------------------------
  // ALU: input mux for regA or PC
  //--------------------------------------------
  always_comb
    case(aluSrcA_s)
      SRC_REGA    : srcA_s = regA_s;
      SRC_PC      : srcA_s = PC_s;
      SRC_ZERO    : srcA_s = {reg_width{1'b0}};
      default     : srcA_s = regA_s;
    endcase // case (aluSrcA_s)
  
  //--------------------------------------------
  // ALU
  //--------------------------------------------
  as_alu alua (.data01_i(srcA_s),
               .data02_i(srcB_s),
               .alu_op_i(aluSela_s),
               .aluResult_o(aluRes_s)
              );
  assign dBusAddr_s   = aluRes_s;
  assign aluCalcRes_s = aluRes_s;

  as_alu_branch alub (.data01_i(srcA_s),
                      .data02_i(srcB_s),
                      .br_op_i(aluSelb_s),
                      .take_o(take_s)
                     );

  //--------------------------------------------
  // Mux for aluResult, dmem or PC+4 to register file
  //--------------------------------------------
  always_comb
    case(resultSrcx_s)
      RES_ALU    : result_s = aluCalcRes_s;
      RES_MEM    : result_s = dBusDataRd_s;
      RES_PC4    : result_s = PCp4_s;
      RES_CSR    : result_s = csr_data_s;
      default    : result_s = {reg_width{1'b0}};
    endcase
  
  //--------------------------------------------
  // Instruction decoder
  //--------------------------------------------
  as_instr_decode control (.instr_opcode_i(ir_s[6:0]),
                           .instr_func3_i(ir_s[14:12]),
                           .instr_func7b5_i(ir_s[30]),
                           .take_i(take_s),                 // branch taken
                           .mux_resultSrc_o(resultSrcx_s),  // sel for mux which writes data to the RegFile
                           .en_dMemWr_o(dMemWr_s),          // D-Mem write enable
                           .en_dMemRd_o(dMemRd_s),          // D-Mem read ensable, not needed by the D-Mem but as control for other logic
                           .mux_aluSrcB_o(aluSrcB_s),       // sel for mux which writes data to the ALU B entry
                           .mux_aluSrcA_o(aluSrcA_s),       // sel for mux which writes data to the ALU A entry
                           .en_regWr_o(regWr_s),            // RegFile write enable
                           .mux_jump_o(jump_s),             // sel for mux which writes the jump target address
                           .sel_immSrc_o(immSrcx_s),        // sel for the correct immediate generation
                           .alu_op_o(aluSela_s),            // selects the arithmetic ALU operation
                           .br_op_o(aluSelb_s),             // selects the branch ALU operation
                           .trap_illegal_instr_o(trap_illegal_instrx_s) // indicator for an illegal instruction
                      );
  
  //--------------------------------------------
  // IRQ
  //--------------------------------------------

  // MRET detection
  assign is_mret_s = (ir_s[6:0] == 7'b1110011) && (ir_s[14:12] == 3'b000) && (ir_s[31:20] == 12'h302);
  assign is_mret_fetched_s = (iBusDataRd_s[6:0] == 7'b1110011) && 
                             (iBusDataRd_s[14:12] == 3'b000) && 
                             (iBusDataRd_s[31:20] == 12'h302);

  //===========================================
  // IRQ Synchronization (2-FF)
  //===========================================
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i) begin
      irq_ext_sync1_s <= 1'b0;
      irq_ext_sync2_s <= 1'b0;
    end else begin
      irq_ext_sync1_s <= irq_ext_i[7];  // GPIO IRQ
      irq_ext_sync2_s <= irq_ext_sync1_s;
    end
  end

  //===========================================
  // Interrupt & Trap Detection
  //===========================================
  // IRQ pending when: MIP.MEIP & MIE.MEIE & MSTATUS.MIE
  assign irq_pending_s = csr_mip_s[11] && csr_mie_s[11] && csr_mstatus_mie;
  
  // Illegal instruction detection
  assign opcode_s = ir_s[6:0];

  assign trap_illegal_s = trap_illegal_instrx_s;
  
  // Misaligned detection
  always_comb begin
    trap_misaligned_s = 1'b0;
    if(ir_valid_s && exec_phase_s) begin
      // Load/Store misalignment
      if(opcode_s == 7'b0000011 || opcode_s == 7'b0100011) begin
        case(ir_s[14:12])  // funct3
          3'b000, 3'b100:  // LB, LBU, SB - no alignment required
            trap_misaligned_s = 1'b0;
          3'b001, 3'b101:  // LH, LHU, SH - halfword alignment
            trap_misaligned_s = (dBusAddr_s[0] != 1'b0);
          3'b010:          // LW, SW - word alignment
            trap_misaligned_s = (dBusAddr_s[1:0] != 2'b00);
          3'b011:          // LD, SD - doubleword alignment
            trap_misaligned_s = (dBusAddr_s[2:0] != 3'b000);
          default:
            trap_misaligned_s = 1'b0;
        endcase
      end
      // Branch/Jump misalignment (must be 4-byte aligned)
      if(opcode_s == 7'b1100011 || opcode_s == 7'b1101111 || opcode_s == 7'b1100111) begin
        if(take_s && PCbr_s[1:0] != 2'b00)
          trap_misaligned_s = 1'b1;
      end
    end
  end

  //===========================================
  // Trap Taking Logic
  //===========================================
  // Trap is taken at instruction commit boundary
  // Priority: Illegal instruction > Misaligned > IRQ

  // MRET pending flag: set during exec, used during next fetch0
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i)
      mret_pending_s <= 1'b0;
    else
      //if(is_mret_s && exec_phase_s)
      if(exec_phase_s && is_mret_fetched_s)// && ir_valid_s)
        mret_pending_s <= 1'b1;
      else if(fetch0_phase_s)
        mret_pending_s <= 1'b0;
  end
  
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i)
      trap_taken_s <= 1'b0;
    else
      if(instr_commit_s && !is_mret_s)
        // Check traps in priority order
        if(trap_illegal_s)
          trap_taken_s <= 1'b1;
        else
          if(trap_misaligned_s)
            trap_taken_s <= 1'b1;
          else
            if(irq_pending_s)
              trap_taken_s <= 1'b1;
            else
              trap_taken_s <= 1'b0;
      else
        trap_taken_s <= 1'b0;
  end


  // Write the CSRs
  // MIP: 64 bit, r, h344; OK (only GPIO IRQs)
  /* From spec: on external IRQ -> mip.MEIP = 1 (remains 1 as long as the external IRQ is active)
                                                (can not be changed by SW)
                                                (will be removed only ba the external IRQ)
   */
  //===========================================
  // MIP Register (Read-Only for MEIP)
  //===========================================
  // MIP[11] = MEIP (Machine External Interrupt Pending)
  // This is directly driven by synchronized external IRQ
  always_ff @(posedge clk_i, posedge rst_i) begin
    if(rst_i)
      csr_mip_s <= 64'h0;
    else
      csr_mip_s[11] <= irq_ext_sync2_s;  // Direct wire-through
  end
  
  // MIE: 64 bit, rw, h304); OK
  //===========================================
  // CSR: MIE (Machine Interrupt Enable)
  //===========================================
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
      csr_mie_s <= 64'h0000000000000800;  // MEIE enabled by default
    else
    begin
      if(exec_phase_s && ir_valid_s)
        if((ir_s[6:0] == 7'b1110011) && (ir_s[31:20] == 12'h304))               // csr..
          case(ir_s[14:12])
            3'b001: csr_mie_s <= regA_s;                                        // csrrw
            3'b010: csr_mie_s <= csr_mie_s | regA_s;                            // csrrs
            3'b011: csr_mie_s <= csr_mie_s & ~regA_s;                           // csrrc
            3'b101: csr_mie_s <= {{52{1'b0}}, ir_s[19:15], 7'b0};               // csrrwi
            3'b110: csr_mie_s <= csr_mie_s | {{52{1'b0}}, ir_s[19:15], 7'b0};   // csrrsi
            3'b111: csr_mie_s <= csr_mie_s & ~{{52{1'b0}}, ir_s[19:15], 7'b0};  // csrrci
	    default: csr_mie_s <= regA_s;
          endcase
    end
  end

  // MSTATUS: 64 bit, rw/r, h300
  /* From spec: on external IRQ -> mstatus.MPIE = mstatus.MIE, 
                                   mstatus.MIE  = 0
                on mret         -> mstatus.MIE  = mstatus.MPIE
                                   mstatus.MPIE = 1
   */
  //===========================================
  // CSR: MSTATUS
  //===========================================
  assign csr_mstatus_mpie = csr_mstatus_s[7];
  assign csr_mstatus_mie  = csr_mstatus_s[3];
  
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
      csr_mstatus_s <= 64'h0000000000001808;  // MIE=1, MPIE=1
    else
    begin
      if( trap_taken_s )  // Trap entry: MPIE <= MIE, MIE <= 0
      begin
        csr_mstatus_s[7] <= csr_mstatus_mie;  // MPIE
        csr_mstatus_s[3] <= 0;                // MIE
      end
      else
        if(is_mret_s && exec_phase_s)            // MRET: MIE <= MPIE, MPIE <= 1
        begin
          csr_mstatus_s[3] <= csr_mstatus_mpie;  // MIE
          csr_mstatus_s[7] <= 1'b1;              // MPIE
        end
        else
          if(exec_phase_s && ir_valid_s) // CSR write
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
  
  // MEPC: 64 bit, WARL, h341, OK
  //===========================================
  // CSR: MEPC (Machine Exception PC)
  //===========================================
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i)
      csr_mepc_s <= 0;
    else
    begin
      if( trap_taken_s )
        if(trap_illegal_s || trap_misaligned_s)
          csr_mepc_s <= PC_s; // PC_s, iBusAddr_s
        else
          csr_mepc_s <= PCp4_s; // csr_mepc_s <= PCp4_s; iBusAddr_s + 64'd4; iBusAddr_s
      else
        if(exec_phase_s && ir_valid_s)
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
  
  // MTVEC: 64 bit, WARL, h305; OK
  // MTVEC[1:0] = 0 => direct mode
  //===========================================
  // CSR: MTVEC (Machine Trap Vector)
  //===========================================
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
      csr_mtvec_s <= 64'h0000000000007F00; // set the ISR address by reset!!!!!
    else
      if(exec_phase_s && ir_valid_s)
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
  
  // MCAUSE: 64 bit, rw/WLRL, h342; OK
  //===========================================
  // CSR: MCAUSE (Machine Cause)
  //===========================================
  always_ff @(posedge clk_i, posedge rst_i) 
  begin
    if(rst_i)
      csr_mcause_s <= 64'h0;
    else 
    begin
      if(trap_taken_s) 
      begin
        // Priority: Illegal > Misaligned > IRQ
        if(trap_illegal_s)
          csr_mcause_s <= 64'd2;  // Illegal instruction
        else if(trap_misaligned_s) 
        begin
          if(dMemRd_s)
            csr_mcause_s <= 64'd4;  // Load address misaligned
          else if(dMemWr_s)
            csr_mcause_s <= 64'd6;  // Store address misaligned
          else
            csr_mcause_s <= 64'd0;  // Instruction address misaligned
        end
        else if(irq_pending_s)
          csr_mcause_s <= {1'b1, 63'd11};  // Machine external interrupt
      end
    end
  end // always_ff @ (posedge clk_i, posedge rst_i)
  
  //===========================================
  // CSR Read Mux
  //===========================================
  always_comb
  begin
    csr_data_s = 64'h0;
    if( (ir_s[6:0] == 7'b1110011) && (ir_s[14:12] != 3'b000) ) // one of the CSRs and func3 != mret
      case(ir_s[31:20])                              // CSR address
        12'h300: csr_data_s = csr_mstatus_s;
        12'h304: csr_data_s = csr_mie_s;
        12'h305: csr_data_s = csr_mtvec_s;
        12'h341: csr_data_s = csr_mepc_s;
        12'h342: csr_data_s = csr_mcause_s;
        12'h344: csr_data_s = csr_mip_s;
        default: csr_data_s = 64'h0;
      endcase
  end

  

  // 2Do: - mret einbauen                                (done)
  //      - PC-Mux: ISR-Zieladresse einbauen             (done)
  //      - die restlichen csrxx Instruktionen einbauen  (done)
  //      - mehr IRQ-Quellen, Exceptions ud Traps        (done)
  //      - Assembler-Testprogramm schreiben             (done)
  //      - External IRQ programmable: level, rising, falling edge

  //--------------------------------------------
  // Cache interfaces – disabled, tied to 0
  // Active low-level: CPU drives addr/req/wr to 0; cache responses unused.
  //--------------------------------------------
  assign icpu_if.ic_addr  = '0;
  assign icpu_if.ic_req   = 1'b0;
  assign icpu_if.ic_flush = 1'b0;

  assign dcpu_if.dc_addr  = '0;
  assign dcpu_if.dc_req   = 1'b0;
  assign dcpu_if.dc_wr    = 1'b0;
  assign dcpu_if.dc_size  = '0;
  assign dcpu_if.dc_wdata = '0;
  assign dcpu_if.dc_wstrb = '0;
  assign dcpu_if.dc_flush = 1'b0;

  //--------------------------------------------
  // Test Scan Chain
  //--------------------------------------------
  assign clk_mux_s = (sc01_shift_i == 1) ? tck_i : gated_clk_s; // Must be a real clock mux!
  assign gated_clk_s = clk_i && dr_cap_i;                       // Must be a real clock gate!
  
  //assign clk_mux_s = ~tck_i; // !!! Must be a real clock mux!!!
  scan_cell sc01 (clk_mux_s, rst_i, sc01_shift_i, 1'b0, sc01_tdi_i, and_in01_s, sc01_01_s);
  scan_cell sc02 (clk_mux_s, rst_i, sc01_shift_i, 1'b0, sc01_01_s, and_in02_s, sc01_02_s);
  assign and_out_s = and_in01_s & and_in02_s;
  scan_cell sc03 (clk_mux_s, rst_i, sc01_shift_i, and_out_s, sc01_02_s, , sc01_03_s); // to_some_pin1_s
  scan_cell sc04 (clk_mux_s, rst_i, sc01_shift_i, 1'b0, sc01_03_s, , sc01_tdo_o); // to_some_pin2_s

endmodule : as_cpux

