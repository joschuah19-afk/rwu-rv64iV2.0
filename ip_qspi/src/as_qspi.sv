
// =============================================================================
// as_qspi.sv  –  QSPI Peripheral Kernel
// =============================================================================
//
// FSM States
// ==========
//   Normal: IDLE → CMD → ADDR → DUM → DAT → DONE → IDLE
//   XIP:    IDLE → ADDR → DUM → DAT → DONE → IDLE  (CMD skipped)
//
// Bus modes
// =========
//   ctrl_reg_i.quad / .dual control bits-per-SCK-cycle for ADDR and DATA.
//   CMD is ALWAYS Single-SPI (8 SCK cycles), regardless of quad/dual setting.
//   This matches all standard 1-x-x and x-1-x mode flash devices.
//
// SCK generation
// ==============
//   f_SCK = f_clk / (2*(clkdiv_reg_i+1))
//   sck_fall_s / sck_rise_s: one clk_i cycle BEFORE the actual SCK edge.
//   Mode 0 (CPOL=0,CPHA=0): drive on fall_s, sample on rise_s.
//
// stat_done_o
// ===========
//   Combinatorial level signal: high for exactly one clk_i cycle when
//   state_r == DONE_ST.
//
// XIP mode
// =========
//   xip_active_r latches when ctrl_reg_i.xip=1 AND state_r==DONE_ST.
//   Cleared immediately when ctrl_reg_i.xip=0.
//   During XIP, the first dummy cycle drives xip_mode_bits_i on the bus
//   (Winbond 0xA0 = stay in Continuous Read mode).
// =============================================================================
`timescale 1ns/1ps
import as_pack::*;

module as_qspi (
  input  logic        rst_i,
  input  logic        clk_i,
  input  logic        start_i,
  input  qspi_ctrl_t  ctrl_reg_i,
  input  logic [7:0]  cmd_reg_i,
  input  logic [31:0] addr_reg_i,
  input  logic [15:0] len_reg_i,
  input  logic [5:0]  dummy_reg_i,
  input  logic [7:0]  clkdiv_reg_i,
  input  logic [31:0] timeout_reg_i,
  input  logic [7:0]  xip_mode_bits_i,
  output logic        xip_active_o,
  output logic        stat_busy_o,
  output logic        stat_done_o,
  output logic        stat_error_o,
  output logic        stat_timeout_o,
  input  logic        tx_empty_i,
  input  logic        rx_full_i,
  output logic        tx_rd_o,
  output logic        rx_wr_o,
  input  logic [63:0] tx_data_i,
  output logic [63:0] rx_data_o,
  output logic        sck_o,
  output logic        cs_o,
  inout  tri  [3:0]   data_io
);

  // ---------------------------------------------------------------------------
  // FSM: declaration
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {idle_st, cmd_st, addr_st, dum_st, dat_st, done_st} statetype_t;
  statetype_t state_s, nextstate_s;

  // ---------------------------------------------------------------------------
  // Preparation logic:
  // ---------------------------------------------------------------------------
  // ---------------------------------------------------------------------------
  // Bus width; detects from ctrl-reg if the transaction should be done with 1 bit, 2 bits or 4 bits in parallel
  // ---------------------------------------------------------------------------
  logic [2:0] bpc_s;   // bits per SCK cycle for ADDR/DATA phase
  assign bpc_s = ctrl_reg_i.quad ? 3'd4 :
                 ctrl_reg_i.dual ? 3'd2 : 3'd1;

  // ---------------------------------------------------------------------------
  // Phase cycle counts (0-based max counter values)
  // CMD: always 8 Single cycles → max = 7
  // ADDR: 24 or 32 bits at bpc_s bits/cycle
  // DATA: len_reg_i bytes = len_reg_i*8 bits (counted in bits)
  // ---------------------------------------------------------------------------
  localparam logic [2:0] CMD_MAX = 3'd7;   // 8 Single cycles, always

  logic [4:0] addr_max_s;
  always_comb
    case (bpc_s)
      3'd4:    addr_max_s = ctrl_reg_i.addr_len ? 5'd7  : 5'd5;   // 32/4-1, 24/4-1
      3'd2:    addr_max_s = ctrl_reg_i.addr_len ? 5'd15 : 5'd11;
      default: addr_max_s = ctrl_reg_i.addr_len ? 5'd31 : 5'd23;
    endcase

  logic [19:0] dat_max_s;   // in bits
  assign dat_max_s = {len_reg_i, 3'b000} - 20'd1;


  // ---------------------------------------------------------------------------
  // Signal declarations (grouped here so all uses in FSM block 1 are post-declaration)
  // ---------------------------------------------------------------------------
  logic xip_active_r;
  logic sck_fall_s, sck_rise_s;
  logic sck_drive_s, sck_sample_s;
  logic tx_flag_r;
  logic [2:0]  cnt_cmd_r;
  logic [4:0]  cnt_addr_r;
  logic [5:0]  cnt_dum_r;
  logic [19:0] cnt_dat_r;

  // ---------------------------------------------------------------------------
  // FSM:
  // ---------------------------------------------------------------------------
  // ---------------------------------------------------------------------------
  // FSM block 1: nextstate, input CLC
  // ---------------------------------------------------------------------------
  // cnt_dum_r is registered: it holds the count from the *previous* cycle.
  // We leave DUM_ST on the sck_rise_s that would push cnt_dum_r to
  // dummy_reg_i, i.e. when (cnt_dum_r + 1 == dummy_reg_i).
  always_comb 
  begin
    nextstate_s = state_s;
    case(state_s)
      idle_st :  if(start_i == 1'b1)
                   nextstate_s = xip_active_r ? addr_st : cmd_st;
      cmd_st  :  if(sck_drive_s && cnt_cmd_r >= CMD_MAX)
                   nextstate_s = addr_st;
      addr_st :  if(sck_drive_s && cnt_addr_r >= addr_max_s)
                   nextstate_s = (dummy_reg_i == '0) ? dat_st : dum_st;
      dum_st  :  if(sck_rise_s && ({1'b0, cnt_dum_r} + 7'd1 >= {1'b0, dummy_reg_i}))
                   nextstate_s = dat_st;
      dat_st  :  if( tx_flag_r && sck_drive_s  && (cnt_dat_r + {17'd0,bpc_s}) > dat_max_s)
                   nextstate_s = done_st;
                 else if (!tx_flag_r && sck_sample_s && (cnt_dat_r + {17'd0,bpc_s}) > dat_max_s)
                 nextstate_s = done_st;
      done_st :  nextstate_s = idle_st;
      default :  nextstate_s = idle_st;
    endcase
  end

  // ---------------------------------------------------------------------------
  // FSM block 2: delay
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
      state_s <= idle_st;
    else
      state_s <= nextstate_s;
  end
  
  // ---------------------------------------------------------------------------
  // FSM block 3: output
  // ... all below
  // ---------------------------------------------------------------------------

  // ---------------------------------------------------------------------------
  // SCK generator
  // ---------------------------------------------------------------------------
  logic       sck_en_s;
  logic [7:0] sck_cnt_r;
  logic       sck_r;

  // ... set sck enable
  always_comb
    case (state_s)
      cmd_st, addr_st, dum_st, dat_st : sck_en_s = 1'b1;
      default                         : sck_en_s = 1'b0; // IDLE, DONE
    endcase

  // ... sck counter, spi clock
  always_ff @(posedge clk_i, posedge rst_i) 
  begin
    if (rst_i) 
    begin
      sck_cnt_r <= '0;
      sck_r     <= 1'b0;
    end 
    else if (!sck_en_s) // IDLE, DONE
    begin
      sck_cnt_r <= '0;
      sck_r     <= ctrl_reg_i.cpol; // set clock polarity
    end 
    else if (sck_cnt_r >= clkdiv_reg_i) 
    begin
      sck_cnt_r <= '0;
      sck_r     <= ~sck_r;
    end 
    else 
    begin
      sck_cnt_r <= sck_cnt_r + 8'd1;
    end
  end

  assign sck_o = sck_r;

  // Strobes fire one clk_i cycle BEFORE the actual SCK edge.
  assign sck_fall_s  = sck_en_s && (sck_cnt_r == clkdiv_reg_i) &&  sck_r;
  assign sck_rise_s  = sck_en_s && (sck_cnt_r == clkdiv_reg_i) && !sck_r;

  // Mode 0 (CPOL=0,CPHA=0): drive on falling edge, sample on rising edge
  assign sck_drive_s  = ctrl_reg_i.cpol ? sck_rise_s : sck_fall_s;
  assign sck_sample_s = ctrl_reg_i.cpol ? sck_fall_s : sck_rise_s;

  // ---------------------------------------------------------------------------
  // Phase counters
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)                  cnt_cmd_r <= '0;
    else if (state_s != cmd_st) cnt_cmd_r <= '0;
    else if (sck_drive_s)       cnt_cmd_r <= (cnt_cmd_r >= CMD_MAX) ? '0 : cnt_cmd_r + 3'd1;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)                   cnt_addr_r <= '0;
    else if (state_s != addr_st) cnt_addr_r <= '0;
    else if (sck_drive_s)        cnt_addr_r <= (cnt_addr_r >= addr_max_s) ? '0 : cnt_addr_r + 5'd1;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)                  cnt_dum_r <= '0;
    else if (state_s != dum_st) cnt_dum_r <= '0;
    else if (sck_rise_s)        cnt_dum_r <= cnt_dum_r + 6'd1;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)                  cnt_dat_r <= '0;
    else if (state_s != dat_st) cnt_dat_r <= '0;
    else if ( tx_flag_r && sck_drive_s)  cnt_dat_r <= cnt_dat_r + {17'd0, bpc_s};
    else if (!tx_flag_r && sck_sample_s) cnt_dat_r <= cnt_dat_r + {17'd0, bpc_s};

  // ---------------------------------------------------------------------------
  // TX direction flag
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i)                       tx_flag_r <= 1'b0;
    else if (start_i && !tx_empty_i) tx_flag_r <= 1'b1;
    else if (state_s == idle_st)     tx_flag_r <= 1'b0;
  end

  // ---------------------------------------------------------------------------
  // TX shift register – preloaded one cycle before DAT_ST
  // ---------------------------------------------------------------------------
  logic [63:0] tx_shift_r;
  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i)
      tx_shift_r <= '0;
    else if ((state_s == cmd_st  && nextstate_s == dat_st) ||
             (state_s == addr_st && nextstate_s == dat_st))
      tx_shift_r <= tx_data_i;
    else if (state_s == dat_st && tx_flag_r && sck_drive_s)
      case (bpc_s)
        3'd4:    tx_shift_r <= {tx_shift_r[59:0], 4'b0};
        3'd2:    tx_shift_r <= {tx_shift_r[61:0], 2'b0};
        default: tx_shift_r <= {tx_shift_r[62:0], 1'b0};
      endcase
  end

  // ---------------------------------------------------------------------------
  // RX shift register
  // ---------------------------------------------------------------------------
  logic [63:0] rx_shift_r;
  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i)
      rx_shift_r <= '0;
    else if (state_s == dat_st && !tx_flag_r && sck_sample_s)
      case (bpc_s)
        3'd4:    rx_shift_r <= {rx_shift_r[59:0], data_io};
        3'd2:    rx_shift_r <= {rx_shift_r[61:0], data_io[1:0]};
        default: rx_shift_r <= {rx_shift_r[62:0], data_io[0]};
      endcase
  end
  assign rx_data_o = rx_shift_r;

  // ---------------------------------------------------------------------------
  // XIP state
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i)              xip_active_r <= 1'b0;
    else if (!ctrl_reg_i.xip) xip_active_r <= 1'b0;
    else if (state_s == done_st) xip_active_r <= 1'b1;
  end
  assign xip_active_o = xip_active_r;

  // ---------------------------------------------------------------------------
  // data_io driver
  // ---------------------------------------------------------------------------
  logic [3:0] dout_s;
  logic       doe_s;

  // XIP mode-bits: drive xip_mode_bits_i during the first dummy SCK cycle.
  // cnt_dum_r increments on sck_rise_s (= one cycle after the SCK rising edge
  // fires). So cnt_dum_r==0 covers all drive events of the first dummy cycle.
  logic xip_mb_active_s;
  assign xip_mb_active_s = xip_active_r && (state_s == dum_st) && (cnt_dum_r == 6'd0);

  // Sub-index for mode-bit drive (counts sck_drive_s events in first dummy cycle)
  logic [2:0] xip_sub_r;
  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i)                           xip_sub_r <= '0;
    else if (state_s != dum_st)          xip_sub_r <= '0;
    else if (sck_drive_s && xip_mb_active_s) xip_sub_r <= xip_sub_r + 3'd1;
  end

  always_comb begin
    dout_s = 4'b0;
    doe_s  = 1'b0;
    case (state_s)
      cmd_st: begin
        // Always Single, MSB first on io[0]
        dout_s = {3'b0, cmd_reg_i[3'd7 - cnt_cmd_r]};
        doe_s  = 1'b1;
      end
      addr_st: begin
        doe_s = 1'b1;
        case (bpc_s)
          3'd4: dout_s = addr_reg_i[{(addr_max_s - cnt_addr_r), 2'b11} -: 4];
          3'd2: dout_s = {2'b0, addr_reg_i[{(addr_max_s - cnt_addr_r), 1'b1} -: 2]};
          default: dout_s = {3'b0, addr_reg_i[addr_max_s - cnt_addr_r]};
        endcase
      end
      dum_st: begin
        if (xip_mb_active_s) begin
          doe_s = 1'b1;
          case (bpc_s)
            3'd4:    dout_s = xip_sub_r[0] ? xip_mode_bits_i[3:0]
                                           : xip_mode_bits_i[7:4];
            3'd2:    case (xip_sub_r[1:0])
                       2'd0: dout_s = {2'b0, xip_mode_bits_i[7:6]};
                       2'd1: dout_s = {2'b0, xip_mode_bits_i[5:4]};
                       2'd2: dout_s = {2'b0, xip_mode_bits_i[3:2]};
                       2'd3: dout_s = {2'b0, xip_mode_bits_i[1:0]};
                     endcase
            default: dout_s = {3'b0, xip_mode_bits_i[3'd7 - xip_sub_r]};
          endcase
        end
        // else Hi-Z (doe_s = 0)
      end
      dat_st: begin
        if (tx_flag_r) begin
          doe_s = 1'b1;
          case (bpc_s)
            3'd4:    dout_s = tx_shift_r[63:60];
            3'd2:    dout_s = {2'b0, tx_shift_r[63:62]};
            default: dout_s = {3'b0, tx_shift_r[63]};
          endcase
        end
      end
      default: ;
    endcase
  end

  assign data_io = doe_s ? dout_s : 4'bzzzz;

  // ---------------------------------------------------------------------------
  // FIFO strobes
  // ---------------------------------------------------------------------------
  assign tx_rd_o = (state_s == dat_st) &&  tx_flag_r && sck_drive_s  &&
                   ((cnt_dat_r + {17'd0,bpc_s}) > dat_max_s);

  assign rx_wr_o = (state_s == dat_st) && !tx_flag_r && sck_sample_s &&
                   ((cnt_dat_r + {17'd0,bpc_s}) > dat_max_s);

  // ---------------------------------------------------------------------------
  // CS output
  // ---------------------------------------------------------------------------
  assign cs_o = (state_s != idle_st) && (state_s != done_st);

  // ---------------------------------------------------------------------------
  // Error latch
  // ---------------------------------------------------------------------------
  logic error_r;
  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i)     error_r <= 1'b0;
    else if (state_s == dat_st && tx_flag_r  && tx_empty_i) error_r <= 1'b1;
    else if (state_s == dat_st && !tx_flag_r && rx_full_i)  error_r <= 1'b1;
  end
  assign stat_error_o = error_r;

  // ---------------------------------------------------------------------------
  // Timeout
  // ---------------------------------------------------------------------------
  logic [31:0] to_cnt_r;
  logic        to_r;
  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i) begin
      to_cnt_r <= '0;
      to_r     <= 1'b0;
    end else if (state_s == idle_st) begin
      to_cnt_r <= timeout_reg_i;
      to_r     <= 1'b0;
    end else if (timeout_reg_i != '0) begin
      if (to_cnt_r != '0) to_cnt_r <= to_cnt_r - 32'd1;
      else                 to_r     <= 1'b1;
    end
  end
  assign stat_timeout_o = to_r;

  // ---------------------------------------------------------------------------
  // Status outputs
  // ---------------------------------------------------------------------------
  assign stat_busy_o = (state_s != idle_st) && (state_s != done_st);
  assign stat_done_o = (state_s == done_st);   // combinatorial, 1 full cycle

endmodule : as_qspi
