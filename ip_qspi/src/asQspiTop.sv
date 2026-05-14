
// =============================================================================
// asQspiTop.sv  –  QSPI Peripheral Top Level  (v3)
// =============================================================================
// Dual-port architecture:
//   Wishbone B4 Classic slave  → configuration registers, TX/RX FIFO, interrupts
//   AXI4 read-only slave       → cache-refill data (Memory Arbiter → QSPI)
//
// AXI4 port parameters (fixed by architecture):
//   ARLEN=3  (4 beats), ARSIZE=3 (8 B/beat), ARBURST=INCR
//   Only AR and R channels exist; AW/W/B channels are absent.
//
// AXI4 cache-refill sequence:
//   One 32-byte cache line = 4 sequential 8-byte kernel transactions.
//   SW must pre-configure: CMD_REG=0x6B, CTRL.quad=1, DUMMY_REG=8, CLKDIV.
//   AXI4 overrides only addr (ARADDR + beat*8) and len (8 bytes) per sub-tx.
//   The four 64-bit results are buffered internally, then driven as 4 R beats.
//
// Register map (byte offsets from peripheral base):
//   0x00  ID_REG     ro   Peripheral ID
//   0x08  CTRL_REG   rw   [11:4]=qspi_ctrl_t, [3]=start(pulse), [2]=rx_flush, [1]=tx_flush
//   0x10  CMD_REG    rw   [7:0] opcode
//   0x18  ADDR_REG   rw   [31:0] flash address
//   0x20  LEN_REG    rw   [15:0] bytes to transfer
//   0x28  DUMMY_REG  rw   [5:0] dummy cycles
//   0x30  CLKDIV_REG rw   [7:0] SCK divider
//   0x38  TIMEOUT_REG rw  [31:0] timeout counter
//   0x40  ISR        wo   Interrupt Set Register   (w1→set RIS)
//   0x48  RIS        ro   Raw Interrupt Status     [4]=TO,[3]=ERR,[2]=TXHALF,[1]=RXHALF,[0]=DONE
//   0x50  IMSC       rw   Interrupt Mask Control
//   0x58  MIS        ro   Masked Interrupt Status = RIS & IMSC
//   0x60  ICR        wo   Interrupt Clear Register (w1→clear RIS)
//   0x68  RXDATA     ro   Pop from RX FIFO
//   0x70  TXDATA     wo   Push to TX FIFO
//   0x78  FIFOSTAT   ro   [28:24]=rx_level, [20]=rx_full, [19]=rx_half,
//                          [12:8]=tx_level,  [4]=tx_full, [3]=tx_half
//   0x80  XIPMODE    rw   [7:0] XIP mode byte
//   0x88  STATUS     ro   [4]=timeout,[3]=error,[2]=xip_active,[1]=done,[0]=busy
// =============================================================================
`timescale 1ns/1ps

import as_pack::*;

module as_qspi_top #(
  parameter int QSPI_ADDR_WIDTH = 64,
  parameter int QSPI_DATA_WIDTH = 64,
  parameter int FIFO_DEPTH      = 16
)(
  input  logic                       rst_i,
  input  logic                       clk_i,
  // -------------------------------------------------------------------------
  // Wishbone slave port (configuration)
  // -------------------------------------------------------------------------
  input  logic [QSPI_ADDR_WIDTH-1:0] wbdAddr_i,
  input  logic [reg_width-1:0]       wbdDat_i,
  output logic [reg_width-1:0]       wbdDat_o,
  input  logic                       wbdWe_i,
  input  logic [wbdSel-1:0]          wbdSel_i,
  input  logic                       wbdStb_i,
  output logic                       wbdAck_o,
  input  logic                       wbdCyc_i,
  // -------------------------------------------------------------------------
  // AXI4 slave port (cache-refill data, read-only: AR + R channels)
  // AW / W / B channels present in interface but not used; tied to safe idle.
  // -------------------------------------------------------------------------
  as_axi4_if.slave                   axi4_if,
  // -------------------------------------------------------------------------
  // SPI PHY
  // -------------------------------------------------------------------------
  output logic        sck_o,
  output logic        cs_o,
  inout  tri   [3:0]  data_io,
  // IRQ
  output logic        qspi_irq_o
);

  // ---------------------------------------------------------------------------
  // QSPI register map: byte offsets and reset values
  // Defined locally so this module compiles with any as_pack.sv variant.
  // ---------------------------------------------------------------------------
  localparam int OFF_ID      =   0;
  localparam int OFF_CTRL    =   8;
  localparam int OFF_CMD     =  16;
  localparam int OFF_ADDR    =  24;
  localparam int OFF_LEN     =  32;
  localparam int OFF_DUMMY   =  40;
  localparam int OFF_CLKDIV  =  48;
  localparam int OFF_TIMEOUT =  56;
  localparam int OFF_ISR     =  64;
  localparam int OFF_RIS     =  72;
  localparam int OFF_IMSC    =  80;
  localparam int OFF_MIS     =  88;
  localparam int OFF_ICR     =  96;
  localparam int OFF_RXDATA  = 104;
  localparam int OFF_TXDATA  = 112;
  localparam int OFF_FIFOSTAT= 120;
  localparam int OFF_XIPMODE = 128;
  localparam int OFF_STATUS  = 136;

  localparam logic [63:0] RST_ID      = 64'h00000000_00000010;
  localparam logic [63:0] RST_CTRL    = 64'h00000000_00000000;
  localparam logic [63:0] RST_CMD     = 64'h00000000_0000006B; // 0x6B Quad Fast Read
  localparam logic [63:0] RST_ADDR    = 64'h00000000_00000000;
  localparam logic [63:0] RST_LEN     = 64'h00000000_00000008; // 8 bytes
  localparam logic [63:0] RST_DUMMY   = 64'h00000000_00000008; // 8 dummy cycles
  localparam logic [63:0] RST_CLKDIV  = 64'h00000000_00000004; // CLKDIV=4
  localparam logic [63:0] RST_TIMEOUT = 64'h00000000_00000000;
  localparam logic [63:0] RST_XIPMODE = 64'h00000000_000000A0; // Winbond mode byte
  localparam logic [63:0] RST_IMSC    = 64'h00000000_00000000;
  localparam logic [63:0] RST_RIS     = 64'h00000000_00000000;

  // ---------------------------------------------------------------------------
  // BPI: Wishbone ↔ internal bus
  // ---------------------------------------------------------------------------
  logic [QSPI_ADDR_WIDTH-1:0] addr_s;
  logic [reg_width-1:0]       data_wr_s;
  logic [reg_width-1:0]       data_rd_s;
  logic                       wr_s, rd_s;

  as_slave_bpi #(QSPI_ADDR_WIDTH, QSPI_DATA_WIDTH) u_bpi (
    .rst_i          (rst_i),
    .clk_i          (clk_i),
    .addr_o         (addr_s),
    .dat_from_core_i(data_rd_s),
    .dat_to_core_o  (data_wr_s),
    .wr_o           (wr_s),
    .rd_o           (rd_s),
    .wb_s_addr_i    (wbdAddr_i),
    .wb_s_dat_i     (wbdDat_i),
    .wb_s_dat_o     (wbdDat_o),
    .wb_s_we_i      (wbdWe_i),
    .wb_s_sel_i     (wbdSel_i),
    .wb_s_stb_i     (wbdStb_i),
    .wb_s_ack_o     (wbdAck_o),
    .wb_s_cyc_i     (wbdCyc_i)
  );

  // ---------------------------------------------------------------------------
  // Register write/read strobes
  // ---------------------------------------------------------------------------
  logic wr_ctrl_s, wr_cmd_s, wr_addr_s, wr_len_s, wr_dummy_s;
  logic wr_clkdiv_s, wr_timeout_s, wr_xipmode_s, wr_imsc_s;
  logic wr_isr_s, wr_icr_s, wr_txdata_s;
  logic rd_rxdata_s;

  assign wr_ctrl_s    = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_CTRL));
  assign wr_cmd_s     = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_CMD));
  assign wr_addr_s    = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_ADDR));
  assign wr_len_s     = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_LEN));
  assign wr_dummy_s   = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_DUMMY));
  assign wr_clkdiv_s  = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_CLKDIV));
  assign wr_timeout_s = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_TIMEOUT));
  assign wr_xipmode_s = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_XIPMODE));
  assign wr_imsc_s    = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_IMSC));
  assign wr_isr_s     = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_ISR));
  assign wr_icr_s     = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_ICR));
  assign rd_rxdata_s  = rd_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_RXDATA));
  assign wr_txdata_s  = wr_s && (addr_s == QSPI_ADDR_WIDTH'(OFF_TXDATA));

  // ---------------------------------------------------------------------------
  // Configuration registers
  // ---------------------------------------------------------------------------
  logic [reg_width-1:0] id_reg_s;
  logic [reg_width-1:0] ctrl_reg_s;
  logic [reg_width-1:0] cmd_reg_s;
  logic [reg_width-1:0] addr_reg_s;
  logic [reg_width-1:0] len_reg_s;
  logic [reg_width-1:0] dummy_reg_s;
  logic [reg_width-1:0] clkdiv_reg_s;
  logic [reg_width-1:0] timeout_reg_s;
  logic [reg_width-1:0] xipmode_reg_s;
  logic [reg_width-1:0] imsc_reg_s;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i) id_reg_s <= RST_ID;

  // CTRL: mask out the volatile bits (start[3], rx_flush[2], tx_flush[1]) on write
  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)          ctrl_reg_s <= RST_CTRL;
    else if (wr_ctrl_s) ctrl_reg_s <= data_wr_s & ~64'h0E;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)         cmd_reg_s <= RST_CMD;
    else if (wr_cmd_s) cmd_reg_s <= data_wr_s;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)          addr_reg_s <= RST_ADDR;
    else if (wr_addr_s) addr_reg_s <= data_wr_s;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)         len_reg_s <= RST_LEN;
    else if (wr_len_s) len_reg_s <= data_wr_s;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)           dummy_reg_s <= RST_DUMMY;
    else if (wr_dummy_s) dummy_reg_s <= data_wr_s;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)            clkdiv_reg_s <= RST_CLKDIV;
    else if (wr_clkdiv_s) clkdiv_reg_s <= data_wr_s;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)             timeout_reg_s <= RST_TIMEOUT;
    else if (wr_timeout_s) timeout_reg_s <= data_wr_s;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)             xipmode_reg_s <= RST_XIPMODE;
    else if (wr_xipmode_s) xipmode_reg_s <= data_wr_s;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i)          imsc_reg_s <= RST_IMSC;
    else if (wr_imsc_s) imsc_reg_s <= data_wr_s;

  // ---------------------------------------------------------------------------
  // CTRL decode for kernel
  // ---------------------------------------------------------------------------
  qspi_ctrl_t ctrl_k_s;
  assign ctrl_k_s = qspi_ctrl_t'(ctrl_reg_s[11:4]);

  // Wishbone start pulse: one cycle after CTRL[3] write
  logic wb_start_r;
  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i) wb_start_r <= 1'b0;
    else        wb_start_r <= wr_ctrl_s && data_wr_s[3];

  // TX/RX flush: combinatorial, active during the write cycle
  logic tx_flush_s, rx_flush_s;
  assign tx_flush_s = wr_ctrl_s && data_wr_s[1];
  assign rx_flush_s = wr_ctrl_s && data_wr_s[2];

  // ---------------------------------------------------------------------------
  // TX FIFO
  // ---------------------------------------------------------------------------
  logic        tx_full_s, tx_empty_s, tx_half_s;
  logic [63:0] tx_data_rd_s;
  logic        tx_rd_kernel_s;
  logic [$clog2(FIFO_DEPTH):0] tx_level_s;

  as_fifo #(.DATA_WIDTH(64), .FIFO_DEPTH(FIFO_DEPTH)) u_txfifo (
    .rst_i         (rst_i),
    .clk_i         (clk_i),
    .flush_i       (tx_flush_s),
    .wr_en_i       (wr_txdata_s && !tx_full_s),
    .data_wr_i     (data_wr_s),
    .full_o        (tx_full_s),
    .almost_full_o (),
    .half_full_o   (),
    .rd_en_i       (tx_rd_kernel_s),
    .data_rd_o     (tx_data_rd_s),
    .empty_o       (tx_empty_s),
    .almost_empty_o(),
    .half_empty_o  (tx_half_s),
    .level_o       (tx_level_s)
  );

  // ---------------------------------------------------------------------------
  // RX FIFO + 2-cycle write pipeline (NBA timing fix for rx_shift_r)
  // ---------------------------------------------------------------------------
  logic        rx_full_s, rx_empty_s, rx_half_s;
  logic [63:0] rx_data_rd_s;
  logic        rx_wr_kernel_s;
  logic [63:0] rx_data_kernel_s;
  logic [$clog2(FIFO_DEPTH):0] rx_level_s;

  logic        rx_wr_d1, rx_wr_d2;
  logic [63:0] rx_data_snap;
  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i) begin
      rx_wr_d1     <= 1'b0;
      rx_wr_d2     <= 1'b0;
      rx_data_snap <= '0;
    end else begin
      rx_wr_d1 <= rx_wr_kernel_s;
      rx_wr_d2 <= rx_wr_d1;
      if (rx_wr_d1) rx_data_snap <= rx_data_kernel_s;
    end
  end

  // rx_pop_r: delayed read-pointer advance for both WB and AXI4 consumers.
  // AXI4 pops when in AXI_RXPOP state and FIFO is not empty.
  // WB pops one cycle after the bus read of RXDATA.
  logic rx_pop_r;

  as_fifo #(.DATA_WIDTH(64), .FIFO_DEPTH(FIFO_DEPTH)) u_rxfifo (
    .rst_i         (rst_i),
    .clk_i         (clk_i),
    .flush_i       (rx_flush_s),
    .wr_en_i       (rx_wr_d2),
    .data_wr_i     (rx_data_snap),
    .full_o        (rx_full_s),
    .almost_full_o (),
    .half_full_o   (rx_half_s),
    .rd_en_i       (rx_pop_r),
    .data_rd_o     (rx_data_rd_s),
    .empty_o       (rx_empty_s),
    .almost_empty_o(),
    .half_empty_o  (),
    .level_o       (rx_level_s)
  );

  // ---------------------------------------------------------------------------
  // AXI4 slave FSM
  // ---------------------------------------------------------------------------
  // Performs 4 sequential 8-byte kernel transactions per 32-byte cache-line
  // request (ARLEN=3). Results buffered in axi_buf[], then driven as R beats.
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    AXI_IDLE,    // ARREADY=1, waiting for AR handshake
    AXI_KICK,    // Issue kernel start pulse for sub-transaction [sub_cnt]
    AXI_WAIT,    // Wait for kernel done (stat_done_s=1)
    AXI_RXPOP,   // Wait for RX FIFO non-empty, then capture + advance
    AXI_RESP     // Drive R channel beats from axi_buf[]
  } axi_st_t;

  axi_st_t     axi_st;
  logic [31:0] axi_araddr_r;   // captured ARADDR
  logic [3:0]  axi_arid_r;     // captured ARID, echoed on R channel
  logic [1:0]  axi_sub_cnt;    // 0..3: index of current 8-byte sub-transaction
  logic [1:0]  axi_beat_cnt;   // 0..3: index of current R channel beat
  logic [63:0] axi_buf [4];    // receive buffer for 4×8-byte words

  logic axi_active;
  assign axi_active = (axi_st != AXI_IDLE);

  // AW / W / B channels: read-only slave → safe idle
  assign axi4_if.awready = 1'b0;
  assign axi4_if.wready  = 1'b0;
  assign axi4_if.bvalid  = 1'b0;
  assign axi4_if.bid     = '0;
  assign axi4_if.bresp   = 2'b00;

  // ARREADY: high when idle and kernel is not busy from Wishbone side
  assign axi4_if.arready = (axi_st == AXI_IDLE) && !stat_busy_s;

  // R channel outputs
  assign axi4_if.rvalid = (axi_st == AXI_RESP);
  assign axi4_if.rdata  = axi_buf[axi_beat_cnt];
  assign axi4_if.rid    = {axi_arid_r};
  assign axi4_if.rresp  = 2'b00;   // OKAY
  assign axi4_if.rlast  = (axi_beat_cnt == 2'd3);

  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i) begin
      axi_st       <= AXI_IDLE;
      axi_araddr_r <= '0;
      axi_arid_r   <= '0;
      axi_sub_cnt  <= '0;
      axi_beat_cnt <= '0;
      for (int i = 0; i < 4; i++) axi_buf[i] <= '0;
    end else begin
      case (axi_st)

        AXI_IDLE: begin
          if (axi4_if.arvalid && axi4_if.arready) begin
            axi_araddr_r <= axi4_if.araddr[31:0];
            axi_arid_r   <= axi4_if.arid[3:0];
            axi_sub_cnt  <= '0;
            axi_beat_cnt <= '0;
            axi_st       <= AXI_KICK;
          end
        end

        AXI_KICK: begin
          // axi_start_s is high this cycle (combinatorial from state).
          // Kernel accepts it next cycle (was idle).
          axi_st <= AXI_WAIT;
        end

        AXI_WAIT: begin
          if (stat_done_s)
            axi_st <= AXI_RXPOP;
        end

        AXI_RXPOP: begin
          // Wait for the 2-cycle RX write pipeline to deliver the word.
          if (!rx_empty_s) begin
            axi_buf[axi_sub_cnt] <= rx_data_rd_s;  // capture before pop
            if (axi_sub_cnt == 2'd3)
              axi_st <= AXI_RESP;
            else begin
              axi_sub_cnt <= axi_sub_cnt + 2'd1;
              axi_st      <= AXI_KICK;
            end
          end
        end

        AXI_RESP: begin
          if (axi4_if.rvalid && axi4_if.rready) begin
            if (axi_beat_cnt == 2'd3)
              axi_st <= AXI_IDLE;
            else
              axi_beat_cnt <= axi_beat_cnt + 2'd1;
          end
        end

        default: axi_st <= AXI_IDLE;
      endcase
    end
  end

  // AXI4 start pulse: high for exactly one cycle (the AXI_KICK cycle).
  logic axi_start_s;
  assign axi_start_s = (axi_st == AXI_KICK);

  // Kernel addr/len mux: AXI4 overrides addr and len for each sub-transaction.
  // SW must pre-configure CMD_REG=0x6B, CTRL.quad=1, DUMMY_REG, CLKDIV_REG.
  logic [31:0] kernel_addr_s;
  logic [15:0] kernel_len_s;
  assign kernel_addr_s = axi_active
    ? (axi_araddr_r + {27'd0, axi_sub_cnt, 3'd0})  // ARADDR + sub_cnt*8
    : addr_reg_s[31:0];
  assign kernel_len_s  = axi_active ? 16'd8 : len_reg_s[15:0];

  // RX FIFO pop: extended to serve both WB (delayed by 1 cycle) and AXI4 (in RXPOP state).
  logic axi_rxpop_s;
  assign axi_rxpop_s = (axi_st == AXI_RXPOP) && !rx_empty_s;

  always_ff @(posedge clk_i, posedge rst_i)
    if (rst_i) rx_pop_r <= 1'b0;
    else       rx_pop_r <= (rd_rxdata_s && !rx_empty_s) || axi_rxpop_s;

  // ---------------------------------------------------------------------------
  // QSPI kernel instantiation
  // ---------------------------------------------------------------------------
  logic stat_busy_s, stat_done_s, stat_error_s, stat_timeout_s, xip_active_s;

  as_qspi u_qspi (
    .rst_i          (rst_i),
    .clk_i          (clk_i),
    .start_i        (wb_start_r || axi_start_s),
    .ctrl_reg_i     (ctrl_k_s),
    .cmd_reg_i      (cmd_reg_s[7:0]),
    .addr_reg_i     (kernel_addr_s),
    .len_reg_i      (kernel_len_s),
    .dummy_reg_i    (dummy_reg_s[5:0]),
    .clkdiv_reg_i   (clkdiv_reg_s[7:0]),
    .timeout_reg_i  (timeout_reg_s[31:0]),
    .xip_mode_bits_i(xipmode_reg_s[7:0]),
    .xip_active_o   (xip_active_s),
    .stat_busy_o    (stat_busy_s),
    .stat_done_o    (stat_done_s),
    .stat_error_o   (stat_error_s),
    .stat_timeout_o (stat_timeout_s),
    .tx_empty_i     (tx_empty_s),
    .tx_rd_o        (tx_rd_kernel_s),
    .tx_data_i      (tx_data_rd_s),
    .rx_full_i      (rx_full_s),
    .rx_wr_o        (rx_wr_kernel_s),
    .rx_data_o      (rx_data_kernel_s),
    .sck_o          (sck_o),
    .cs_o           (cs_o),
    .data_io        (data_io)
  );

  // ---------------------------------------------------------------------------
  // Interrupt logic
  // RIS[0]=DONE, [1]=RXHALF, [2]=TXHALF, [3]=ERROR, [4]=TIMEOUT
  // ---------------------------------------------------------------------------
  logic [reg_width-1:0] ris_reg_s;
  logic [reg_width-1:0] mis_reg_s;

  logic stat_done_d, rx_half_d, tx_half_d, stat_error_d, stat_timeout_d;
  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i) begin
      stat_done_d    <= 1'b0;
      rx_half_d      <= 1'b0;
      tx_half_d      <= 1'b0;
      stat_error_d   <= 1'b0;
      stat_timeout_d <= 1'b0;
    end else begin
      stat_done_d    <= stat_done_s;
      rx_half_d      <= rx_half_s;
      tx_half_d      <= tx_half_s;
      stat_error_d   <= stat_error_s;
      stat_timeout_d <= stat_timeout_s;
    end
  end

  wire done_pulse    = stat_done_s    & ~stat_done_d;
  wire rxhalf_pulse  = rx_half_s      & ~rx_half_d;
  wire txhalf_pulse  = tx_half_s      & ~tx_half_d;
  wire error_pulse   = stat_error_s   & ~stat_error_d;
  wire timeout_pulse = stat_timeout_s & ~stat_timeout_d;

  always_ff @(posedge clk_i, posedge rst_i) begin
    if (rst_i) begin
      ris_reg_s <= RST_RIS;
    end else begin
      if      (wr_icr_s && data_wr_s[0]) ris_reg_s[0] <= 1'b0;
      else if (wr_isr_s && data_wr_s[0]) ris_reg_s[0] <= 1'b1;
      else if (done_pulse)               ris_reg_s[0] <= 1'b1;

      if      (wr_icr_s && data_wr_s[1]) ris_reg_s[1] <= 1'b0;
      else if (wr_isr_s && data_wr_s[1]) ris_reg_s[1] <= 1'b1;
      else if (rxhalf_pulse)             ris_reg_s[1] <= 1'b1;

      if      (wr_icr_s && data_wr_s[2]) ris_reg_s[2] <= 1'b0;
      else if (wr_isr_s && data_wr_s[2]) ris_reg_s[2] <= 1'b1;
      else if (txhalf_pulse)             ris_reg_s[2] <= 1'b1;

      if      (wr_icr_s && data_wr_s[3]) ris_reg_s[3] <= 1'b0;
      else if (wr_isr_s && data_wr_s[3]) ris_reg_s[3] <= 1'b1;
      else if (error_pulse)              ris_reg_s[3] <= 1'b1;

      if      (wr_icr_s && data_wr_s[4]) ris_reg_s[4] <= 1'b0;
      else if (wr_isr_s && data_wr_s[4]) ris_reg_s[4] <= 1'b1;
      else if (timeout_pulse)            ris_reg_s[4] <= 1'b1;

      ris_reg_s[reg_width-1:5] <= '0;
    end
  end

  assign mis_reg_s  = ris_reg_s & imsc_reg_s;
  assign qspi_irq_o = |mis_reg_s;

  // ---------------------------------------------------------------------------
  // STATUS and FIFOSTAT (combinatorial)
  // ---------------------------------------------------------------------------
  logic [reg_width-1:0] status_s, fifostat_s;

  assign status_s  = {59'b0, stat_timeout_s, stat_error_s,
                      xip_active_s, stat_done_s, stat_busy_s};

  assign fifostat_s = {reg_width{1'b0}}
    | (64'(rx_full_s)  << 25)
    | (64'(rx_half_s)  << 24)
    | (64'(rx_level_s) << 16)
    | (64'(tx_full_s)  <<  9)
    | (64'(tx_half_s)  <<  8)
    | (64'(tx_level_s));

  // ---------------------------------------------------------------------------
  // Wishbone read multiplexer
  // ---------------------------------------------------------------------------
  always_comb begin
    case (addr_s)
      QSPI_ADDR_WIDTH'(OFF_ID)      : data_rd_s = id_reg_s;
      QSPI_ADDR_WIDTH'(OFF_CTRL)    : data_rd_s = ctrl_reg_s;
      QSPI_ADDR_WIDTH'(OFF_CMD)     : data_rd_s = cmd_reg_s;
      QSPI_ADDR_WIDTH'(OFF_ADDR)    : data_rd_s = addr_reg_s;
      QSPI_ADDR_WIDTH'(OFF_LEN)     : data_rd_s = len_reg_s;
      QSPI_ADDR_WIDTH'(OFF_DUMMY)   : data_rd_s = dummy_reg_s;
      QSPI_ADDR_WIDTH'(OFF_CLKDIV)  : data_rd_s = clkdiv_reg_s;
      QSPI_ADDR_WIDTH'(OFF_TIMEOUT) : data_rd_s = timeout_reg_s;
      QSPI_ADDR_WIDTH'(OFF_ISR)     : data_rd_s = '0;   // write-only
      QSPI_ADDR_WIDTH'(OFF_RIS)     : data_rd_s = ris_reg_s;
      QSPI_ADDR_WIDTH'(OFF_IMSC)    : data_rd_s = imsc_reg_s;
      QSPI_ADDR_WIDTH'(OFF_MIS)     : data_rd_s = mis_reg_s;
      QSPI_ADDR_WIDTH'(OFF_ICR)     : data_rd_s = '0;   // write-only
      QSPI_ADDR_WIDTH'(OFF_RXDATA)  : data_rd_s = rx_empty_s ? '0 : rx_data_rd_s;
      QSPI_ADDR_WIDTH'(OFF_TXDATA)  : data_rd_s = '0;   // write-only
      QSPI_ADDR_WIDTH'(OFF_FIFOSTAT): data_rd_s = fifostat_s;
      QSPI_ADDR_WIDTH'(OFF_XIPMODE) : data_rd_s = xipmode_reg_s;
      QSPI_ADDR_WIDTH'(OFF_STATUS)  : data_rd_s = status_s;
      default                       : data_rd_s = '0;
    endcase
  end

endmodule : as_qspi_top
