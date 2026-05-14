// =============================================================================
// asQspiTop_tb.sv  v3
// =============================================================================
// Tests both bus ports:
//   T1–T12  Wishbone configuration port (register R/W, FIFOs, IRQ, transfers)
//   T13     AXI4 data port: 4-beat burst → 4 sequential 8-byte kernel runs
//   T14     AXI4 ARREADY blocked while WB-started kernel is busy
// =============================================================================
`timescale 1ns/1ps
import as_pack::*;

module asQspiTop_tb;

  localparam int CLK_PERIOD = 10;
  localparam int ADDR_W     = 64;
  localparam int DATA_W     = 64;
  localparam int FDEPTH     = 16;

  // ── clocks / reset ─────────────────────────────────────────────────────────
  logic rst_i, clk_i;
  initial clk_i = 0;
  always  #(CLK_PERIOD/2) clk_i = ~clk_i;

  // ── Wishbone signals ───────────────────────────────────────────────────────
  logic [ADDR_W-1:0] wbdAddr_i;
  logic [DATA_W-1:0] wbdDat_i;
  logic [DATA_W-1:0] wbdDat_o;
  logic              wbdWe_i, wbdStb_i, wbdAck_o, wbdCyc_i;
  logic [wbdSel-1:0] wbdSel_i;

  // ── AXI4 interface (master side driven by TB, slave side driven by DUT) ───
  as_axi4_if #(.ADDR_W(32)) axi4_if (.clk_i(clk_i), .rst_i(rst_i));

  // Flat signals kept for backward-compatible task code; bridged to interface.
  logic        axi_arvalid, axi_arready;
  logic [3:0]  axi_arid;
  logic [31:0] axi_araddr;
  logic [3:0]  axi_arlen;
  logic [2:0]  axi_arsize;
  logic [1:0]  axi_arburst;
  logic        axi_rvalid, axi_rready;
  logic [3:0]  axi_rid;
  logic [63:0] axi_rdata;
  logic [1:0]  axi_rresp;
  logic        axi_rlast;

  // TB → interface (master direction: AR + R-ready; no write channel)
  assign axi4_if.arvalid = axi_arvalid;
  assign axi4_if.arid    = {axi_arid};
  assign axi4_if.araddr  = axi_araddr;
  assign axi4_if.arlen   = {4'b0, axi_arlen};
  assign axi4_if.arsize  = axi_arsize;
  assign axi4_if.arburst = axi_arburst;
  assign axi4_if.rready  = axi_rready;
  assign axi4_if.awvalid = 1'b0;  assign axi4_if.awid   = '0;
  assign axi4_if.awaddr  = '0;    assign axi4_if.awlen  = '0;
  assign axi4_if.awsize  = '0;    assign axi4_if.awburst= '0;
  assign axi4_if.wdata   = '0;    assign axi4_if.wstrb  = '0;
  assign axi4_if.wlast   = 1'b0;  assign axi4_if.wvalid = 1'b0;
  assign axi4_if.bready  = 1'b0;

  // Interface → TB (slave direction: AR-ready + R channel)
  assign axi_arready = axi4_if.arready;
  assign axi_rvalid  = axi4_if.rvalid;
  assign axi_rid     = axi4_if.rid[3:0];
  assign axi_rdata   = axi4_if.rdata;
  assign axi_rresp   = axi4_if.rresp;
  assign axi_rlast   = axi4_if.rlast;

  // ── SPI / Flash ────────────────────────────────────────────────────────────
  logic sck_o, cs_o, qspi_irq_o;
  wire  [3:0] data_io;
  logic [3:0] flash_drive_s = 4'b0;
  logic       flash_oe_s    = 1'b0;
  assign data_io = flash_oe_s ? flash_drive_s : 4'bzzzz;

  // ── DUT ───────────────────────────────────────────────────────────────────
  as_qspi_top #(.QSPI_ADDR_WIDTH(ADDR_W),.QSPI_DATA_WIDTH(DATA_W),.FIFO_DEPTH(FDEPTH)) dut (
    .rst_i(rst_i), .clk_i(clk_i),
    // Wishbone
    .wbdAddr_i(wbdAddr_i), .wbdDat_i(wbdDat_i), .wbdDat_o(wbdDat_o),
    .wbdWe_i(wbdWe_i), .wbdSel_i(wbdSel_i), .wbdStb_i(wbdStb_i),
    .wbdAck_o(wbdAck_o), .wbdCyc_i(wbdCyc_i),
    // AXI4
    .axi4_if(axi4_if),
    // SPI
    .sck_o(sck_o), .cs_o(cs_o), .data_io(data_io), .qspi_irq_o(qspi_irq_o)
  );

  // ── Scorecard ──────────────────────────────────────────────────────────────
  int pass_cnt = 0, fail_cnt = 0;

  task automatic chk64(input string lbl, input logic [63:0] got, input logic [63:0] exp);
    if (got !== exp) begin
      $display("FAIL [%7.1f ns] %-42s  got=%016h  exp=%016h", $realtime, lbl, got, exp);
      fail_cnt++;
    end else begin
      $display("PASS [%7.1f ns] %s", $realtime, lbl);
      pass_cnt++;
    end
  endtask

  task automatic chk1(input string lbl, input logic got, input logic exp);
    if (got !== exp) begin
      $display("FAIL [%7.1f ns] %-42s  got=%b exp=%b", $realtime, lbl, got, exp);
      fail_cnt++;
    end else begin
      $display("PASS [%7.1f ns] %s", $realtime, lbl);
      pass_cnt++;
    end
  endtask

  // ── Wishbone tasks ─────────────────────────────────────────────────────────
  task automatic wb_write(input logic [ADDR_W-1:0] addr, input logic [DATA_W-1:0] data);
    @(negedge clk_i);
    wbdAddr_i=addr; wbdDat_i=data; wbdWe_i=1; wbdSel_i='1; wbdStb_i=1; wbdCyc_i=1;
    @(posedge clk_i); #1;
    @(negedge clk_i); wbdStb_i=0; wbdCyc_i=0; wbdWe_i=0;
  endtask

  logic [DATA_W-1:0] rd_data_g;
  task automatic wb_read(input logic [ADDR_W-1:0] addr, output logic [DATA_W-1:0] rdata);
    @(negedge clk_i);
    wbdAddr_i=addr; wbdWe_i=0; wbdSel_i='1; wbdStb_i=1; wbdCyc_i=1;
    @(posedge clk_i); #1; rdata=wbdDat_o;
    @(negedge clk_i); wbdStb_i=0; wbdCyc_i=0;
  endtask

  // ── Reset + bus idle ───────────────────────────────────────────────────────
  task automatic do_reset();
    rst_i=1;
    wbdAddr_i='0; wbdDat_i='0; wbdWe_i=0; wbdSel_i='0; wbdStb_i=0; wbdCyc_i=0;
    axi_arvalid=0; axi_arid='0; axi_araddr='0;
    axi_arlen=4'd3; axi_arsize=3'd3; axi_arburst=2'b01;
    axi_rready=0;
    flash_oe_s=0; flash_drive_s=4'b0;
    repeat(4) @(posedge clk_i);
    @(negedge clk_i); rst_i=0;
    @(posedge clk_i);
  endtask

  // ── AXI4 burst-read task ───────────────────────────────────────────────────
  // Issues a 4-beat read at 'addr' with ID 'id'.
  // Returns the 4 data words. RREADY is held high throughout.
  task automatic axi_burst_read(
    input  logic [31:0] addr,
    input  logic [3:0]  id,
    output logic [63:0] beat0, beat1, beat2, beat3
  );
    // Present AR channel at negedge (setup before the sampling posedge)
    @(negedge clk_i);
    axi_arvalid=1; axi_araddr=addr; axi_arid=id;
    axi_arlen=4'd3; axi_arsize=3'd3; axi_arburst=2'b01;
    // Wait for ARREADY: sample at negedge (combinatorial, before the posedge
    // that captures the handshake).  Checking at posedge+#1 (after NBA) would
    // see arready=0 because the DUT already advanced to AXI_KICK — deadlock.
    while (!axi_arready) begin @(posedge clk_i); @(negedge clk_i); end
    @(posedge clk_i);              // AR handshake captured by DUT on this edge
    @(negedge clk_i); axi_arvalid=0;

    // Assert RREADY and collect 4 beats
    axi_rready=1;
    // Beat 0 – wait for first RVALID (sampled at posedge+#1, after NBA)
    @(posedge clk_i); #1;
    while (!axi_rvalid) begin @(posedge clk_i); #1; end
    beat0 = axi_rdata;
    @(posedge clk_i); #1; beat1 = axi_rdata;
    @(posedge clk_i); #1; beat2 = axi_rdata;
    @(posedge clk_i); #1; beat3 = axi_rdata;
    // Hold RREADY one extra cycle so the last-beat handshake (beat_cnt=3)
    // completes and AXI_RESP transitions to AXI_IDLE.
    @(posedge clk_i);
    @(negedge clk_i); axi_rready=0;
  endtask

  // ── Register offsets (must match asQspiTop.sv / as_pack.sv) ───────────────
  localparam logic [ADDR_W-1:0]
    A_ID       = ADDR_W'(0),   A_CTRL    = ADDR_W'(8),
    A_CMD      = ADDR_W'(16),  A_ADDR_R  = ADDR_W'(24),
    A_LEN      = ADDR_W'(32),  A_DUMMY   = ADDR_W'(40),
    A_CLKDIV   = ADDR_W'(48),  A_TIMEOUT = ADDR_W'(56),
    A_ISR      = ADDR_W'(64),  A_RIS     = ADDR_W'(72),
    A_IMSC     = ADDR_W'(80),  A_MIS     = ADDR_W'(88),
    A_ICR      = ADDR_W'(96),  A_RXDATA  = ADDR_W'(104),
    A_TXDATA   = ADDR_W'(112), A_FIFOSTAT= ADDR_W'(120),
    A_XIPMODE  = ADDR_W'(128), A_STATUS  = ADDR_W'(136);

  // ── Flash model ────────────────────────────────────────────────────────────
  // Reactive to cs_o posedge; counts sck_o posedges, then drives on negedges.
  logic [63:0] fm_payload='0; logic [7:0] fm_skip='0;
  logic fm_quad=0, fm_arm=0;
  always @(negedge cs_o) begin flash_oe_s=0; flash_drive_s=4'b0; end
  always begin
    @(posedge cs_o);
    if (fm_arm) begin
      automatic logic [63:0] p=fm_payload; automatic int skip=int'(fm_skip);
      automatic logic quad=fm_quad; automatic int n=quad?16:64;
      automatic int cnt=0, idx=0;
      flash_oe_s=0; flash_drive_s=4'b0;
      while(cs_o) begin
        @(posedge sck_o); if(!cs_o) break;
        cnt++;
        if(cnt>=skip && idx<n) begin
          @(negedge sck_o); if(!cs_o) break;
          flash_oe_s=1;
          flash_drive_s=quad?p[(15-idx)*4+:4]:{3'b0,p[63-idx]};
          idx++;
          if(idx==n) begin @(posedge sck_o); break; end
        end
      end
      flash_oe_s=0; flash_drive_s=4'b0;
    end
  end

  // ==========================================================================
  // TESTS
  // ==========================================================================
  logic [DATA_W-1:0] rdata;

  initial begin
    $display("============================================================");
    $display("  asQspiTop Testbench  v3  (Wishbone + AXI4)");
    $display("============================================================");
    do_reset();

    // =========================================================================
    // T1 – Register write / read-back
    // =========================================================================
    $display("\n--- T1: Register write/read-back ---");
    wb_write(A_CMD,     64'h0B);     wb_read(A_CMD,rdata);     chk64("T1 CMD",     rdata, 64'h0B);
    wb_write(A_ADDR_R,  64'h001234); wb_read(A_ADDR_R,rdata);  chk64("T1 ADDR",    rdata, 64'h001234);
    wb_write(A_LEN,     64'd8);      wb_read(A_LEN,rdata);     chk64("T1 LEN",     rdata, 64'd8);
    wb_write(A_DUMMY,   64'd8);      wb_read(A_DUMMY,rdata);   chk64("T1 DUMMY",   rdata, 64'd8);
    wb_write(A_CLKDIV,  64'd2);      wb_read(A_CLKDIV,rdata);  chk64("T1 CLKDIV",  rdata, 64'd2);
    wb_write(A_TIMEOUT, 64'd1000);   wb_read(A_TIMEOUT,rdata); chk64("T1 TIMEOUT", rdata, 64'd1000);
    wb_write(A_XIPMODE, 64'hA0);     wb_read(A_XIPMODE,rdata); chk64("T1 XIPMODE", rdata, 64'hA0);
    wb_write(A_IMSC,    64'h1F);     wb_read(A_IMSC,rdata);    chk64("T1 IMSC",    rdata, 64'h1F);

    // =========================================================================
    // T2 – ID register read-only
    // =========================================================================
    $display("\n--- T2: ID register (read-only) ---");
    wb_read(A_ID,rdata);
    chk64("T2 ID reset value", rdata, 64'h00000000_00000010);
    wb_write(A_ID, 64'hDEAD);
    wb_read(A_ID,rdata);
    chk64("T2 ID unchanged after write", rdata, 64'h00000000_00000010);

    // =========================================================================
    // T3 – TX FIFO write via TXDATA; FIFOSTAT tx_level
    // =========================================================================
    $display("\n--- T3: TX FIFO level ---");
    do_reset();
    wb_read(A_FIFOSTAT,rdata); chk64("T3 FIFOSTAT initial tx_level=0", rdata[4:0], 5'd0);
    wb_write(A_TXDATA, 64'hAA); wb_write(A_TXDATA, 64'hBB); wb_write(A_TXDATA, 64'hCC);
    wb_read(A_FIFOSTAT,rdata);
    chk64("T3 FIFOSTAT tx_level=3", rdata[4:0], 5'd3);

    // =========================================================================
    // T4 – STATUS register at idle
    // =========================================================================
    $display("\n--- T4: STATUS at idle ---");
    do_reset();
    wb_read(A_STATUS,rdata);
    chk64("T4 STATUS idle=0", rdata[4:0], 5'b0);

    // =========================================================================
    // T5 – RIS / MIS / IMSC / ICR / IRQ
    // =========================================================================
    $display("\n--- T5: Interrupt registers ---");
    do_reset();
    wb_write(A_IMSC, 64'h01);
    @(posedge clk_i); #1;
    chk1("T5 IRQ low initially", qspi_irq_o, 1'b0);
    wb_write(A_ISR, 64'h01);
    @(posedge clk_i); #1;
    wb_read(A_RIS,rdata); chk64("T5 RIS[0]=1 after ISR",  rdata[0:0], 1'b1);
    wb_read(A_MIS,rdata); chk64("T5 MIS[0]=1 (unmasked)", rdata[0:0], 1'b1);
    @(posedge clk_i); #1;
    chk1("T5 IRQ high", qspi_irq_o, 1'b1);
    wb_write(A_ICR, 64'h01);
    @(posedge clk_i); #1;
    wb_read(A_RIS,rdata); chk64("T5 RIS[0]=0 after ICR", rdata[0:0], 1'b0);
    @(posedge clk_i); #1;
    chk1("T5 IRQ low after ICR", qspi_irq_o, 1'b0);

    // =========================================================================
    // T6 – IMSC masking
    // =========================================================================
    $display("\n--- T6: IMSC masking ---");
    do_reset();
    wb_write(A_IMSC, 64'h00);
    wb_write(A_ISR,  64'h01);
    @(posedge clk_i); #1;
    wb_read(A_RIS,rdata); chk64("T6 RIS[0]=1 (unmasked)", rdata[0:0], 1'b1);
    wb_read(A_MIS,rdata); chk64("T6 MIS[0]=0 (masked)",   rdata[0:0], 1'b0);
    chk1("T6 IRQ=0 when masked", qspi_irq_o, 1'b0);
    wb_write(A_IMSC, 64'h01);
    @(posedge clk_i); #1;
    chk1("T6 IRQ=1 after unmask", qspi_irq_o, 1'b1);
    wb_write(A_ICR, 64'h01);

    // =========================================================================
    // T7 – TX FIFO full: ACK always fires, level stays at FDEPTH
    // =========================================================================
    $display("\n--- T7: TX FIFO full ---");
    do_reset();
    for (int i = 0; i < FDEPTH; i++) wb_write(A_TXDATA, 64'(i));
    wb_read(A_FIFOSTAT,rdata);
    chk64("T7 TX FIFO full (level=16)", rdata[4:0], 5'd16);
    @(negedge clk_i);
    wbdAddr_i=A_TXDATA; wbdDat_i=64'hDEAD; wbdWe_i=1; wbdSel_i='1; wbdStb_i=1; wbdCyc_i=1;
    @(posedge clk_i); #1;
    chk1("T7 ACK on full-FIFO write", wbdAck_o, 1'b1);
    @(negedge clk_i); wbdStb_i=0; wbdCyc_i=0; wbdWe_i=0;
    wb_read(A_FIFOSTAT,rdata);
    chk64("T7 level still 16", rdata[4:0], 5'd16);

    // =========================================================================
    // T8 – Wishbone ACK zero-wait-state
    // =========================================================================
    $display("\n--- T8: Wishbone ACK timing ---");
    do_reset();
    @(negedge clk_i);
    wbdAddr_i=A_CMD; wbdDat_i=64'hAB; wbdWe_i=1; wbdSel_i='1; wbdStb_i=1; wbdCyc_i=1;
    @(posedge clk_i); #1;
    chk1("T8 ACK same cycle as STB", wbdAck_o, 1'b1);
    @(negedge clk_i); wbdStb_i=0; wbdCyc_i=0; wbdWe_i=0;

    // =========================================================================
    // T9 – Full Single Read via Wishbone: program regs, start, wait DONE, pop RX
    // =========================================================================
    $display("\n--- T9: Full Single Read (0x0B, 8 dummy, 8B) via WB ---");
    begin : t9
      automatic logic [63:0] exp_rx = 64'hDEAD_BEEF_CAFE_F00D;
      automatic int tout;
      do_reset();
      fm_payload=exp_rx; fm_skip=8'd40; fm_quad=0; fm_arm=1;

      wb_write(A_CMD,    64'h0B);
      wb_write(A_ADDR_R, 64'h001234);
      wb_write(A_LEN,    64'd8);
      wb_write(A_DUMMY,  64'd8);
      wb_write(A_CLKDIV, 64'd2);
      wb_write(A_TIMEOUT,64'd0);
      wb_write(A_IMSC,   64'h01);   // DONE only

      wb_write(A_CTRL, 64'h08);     // bit[3]=START

      tout=0;
      do begin @(posedge clk_i); #1; tout++; end while (!qspi_irq_o && tout<200000);
      if (tout>=200000) $display("FAIL  T9: transfer timeout");

      wb_read(A_STATUS,rdata);
      chk64("T9 STATUS busy=0",  rdata[0:0], 1'b0);
      chk64("T9 STATUS error=0", rdata[3:3], 1'b0);
      wb_read(A_RIS, rdata);
      chk64("T9 RIS[0] done=1 (latched)", rdata[0:0], 1'b1);
      chk1 ("T9 IRQ fired",      qspi_irq_o, 1'b1);
      wb_write(A_ICR, 64'h01);
      @(posedge clk_i); #1;
      chk1("T9 IRQ cleared", qspi_irq_o, 1'b0);
      wb_read(A_FIFOSTAT,rdata);
      chk64("T9 RX level=1", rdata[20:16], 5'd1);
      @(posedge clk_i); #1;
      wb_read(A_RXDATA, rdata);
      chk64("T9 RX data", rdata, exp_rx);
      fm_arm=0;
    end

    // =========================================================================
    // T10 – RX half-full interrupt (software ISR path)
    // =========================================================================
    $display("\n--- T10: RXHALF interrupt ---");
    do_reset();
    wb_write(A_IMSC, 64'h02);
    wb_write(A_ISR,  64'h02);
    @(posedge clk_i); #1;
    chk1("T10 RXHALF IRQ via ISR", qspi_irq_o, 1'b1);
    wb_write(A_ICR, 64'h02);
    @(posedge clk_i); #1;
    chk1("T10 IRQ cleared", qspi_irq_o, 1'b0);

    // =========================================================================
    // T11 – CTRL fields preserve (start auto-clears)
    // =========================================================================
    $display("\n--- T11: CTRL fields preserve ---");
    do_reset();
    wb_write(A_CTRL, 64'h10);   // quad bit in qspi_ctrl_t packed at bit4
    wb_read(A_CTRL, rdata);
    chk64("T11 CTRL[4] quad preserved",   rdata[4:4], 1'b1);
    chk64("T11 CTRL[3] start auto-clear", rdata[3:3], 1'b0);

    // =========================================================================
    // T12 – TX flush via CTRL[1]
    // =========================================================================
    $display("\n--- T12: TX flush ---");
    do_reset();
    wb_write(A_TXDATA, 64'h01); wb_write(A_TXDATA, 64'h02);
    wb_read(A_FIFOSTAT,rdata);
    chk64("T12 TX level=2 before flush", rdata[4:0], 5'd2);
    wb_write(A_CTRL, 64'h02);   // bit1=tx_flush
    @(posedge clk_i); #1;
    wb_read(A_FIFOSTAT,rdata);
    chk64("T12 TX level=0 after flush", rdata[4:0], 5'd0);

    // =========================================================================
    // T13 – AXI4 burst read: 32-byte cache line (4 × 8-byte sub-transactions)
    // =========================================================================
    // Pre-configure: CMD=0x6B (Quad Fast Read), CTRL.quad=1, DUMMY=8, CLKDIV=2
    // Flash model returns fm_payload for all 4 sub-transactions.
    // Expected: all 4 R beats = fm_payload.
    // =========================================================================
    $display("\n--- T13: AXI4 burst read (4-beat, 32-byte cache line) ---");
    begin : t13
      automatic logic [63:0] exp_word = 64'hCAFE_BABE_1234_5678;
      automatic logic [63:0] b0, b1, b2, b3;

      do_reset();
      // SW configures QSPI for Quad Fast Read
      wb_write(A_CMD,    64'h6B);   // Quad Fast Read opcode
      wb_write(A_DUMMY,  64'd8);    // 8 dummy cycles
      wb_write(A_CLKDIV, 64'd2);
      wb_write(A_CTRL,   64'h10);   // quad=1 (bit4 of qspi_ctrl_t at CTRL[4])

      // Flash model: fm_skip=22 for 0x6B quad (8 CMD + 6 ADDR-quad + 8 DUMMY)
      fm_payload=exp_word; fm_skip=8'd22; fm_quad=1; fm_arm=1;

      axi_burst_read(32'h00_1000_00, 4'h5, b0, b1, b2, b3);

      chk64("T13 beat0", b0, exp_word);
      chk64("T13 beat1", b1, exp_word);
      chk64("T13 beat2", b2, exp_word);
      chk64("T13 beat3", b3, exp_word);
      chk1 ("T13 RLAST on beat3", axi_rlast, 1'b1);
      chk1 ("T13 RRESP=OKAY",     axi_rresp[1], 1'b0);
      chk64("T13 RID matches AID", {60'b0, axi_rid}, {60'b0, 4'h5});

      @(posedge clk_i); #1;
      chk1("T13 RVALID deasserted after burst", axi_rvalid, 1'b0);
      chk1("T13 ARREADY high again",            axi_arready, 1'b1);
      fm_arm=0;
    end
    repeat(5) @(posedge clk_i);

    // =========================================================================
    // T14 – AXI4 ARREADY blocked while WB-started kernel is busy
    // =========================================================================
    $display("\n--- T14: AXI4 ARREADY blocked during WB-started transfer ---");
    begin : t14
      do_reset();
      // Configure for single read (no quad)
      wb_write(A_CMD,    64'h03);
      wb_write(A_ADDR_R, 64'h005000);
      wb_write(A_LEN,    64'd8);
      wb_write(A_DUMMY,  64'd0);
      wb_write(A_CLKDIV, 64'd2);
      wb_write(A_CTRL,   64'h00);   // quad=0
      // Do NOT arm flash model → kernel will run but no data returns (timeout test not needed)
      // We only want to check ARREADY is low while busy.
      fm_arm=0;

      // Start kernel via Wishbone
      wb_write(A_CTRL, 64'h08);     // START
      @(posedge clk_i); #1;
      // Kernel should now be busy
      chk1("T14 kernel busy after WB start", dut.stat_busy_s, 1'b1);

      // Drive AR but expect ARREADY=0
      @(negedge clk_i);
      axi_arvalid=1; axi_araddr=32'h001000; axi_arid=4'h2;
      axi_arlen=4'd3; axi_arsize=3'd3; axi_arburst=2'b01;
      @(posedge clk_i); #1;
      chk1("T14 ARREADY=0 while busy", axi_arready, 1'b0);
      @(negedge clk_i); axi_arvalid=0;

      // Let kernel finish (or reset)
      do_reset();
      @(posedge clk_i); #1;
      chk1("T14 ARREADY=1 after reset", axi_arready, 1'b1);
    end

    // =========================================================================
    // Summary
    // =========================================================================
    repeat(5) @(posedge clk_i);
    $display("\n============================================================");
    $display("  SUMMARY:  %0d PASSED,  %0d FAILED", pass_cnt, fail_cnt);
    $display("============================================================");
    if (fail_cnt==0) $display("  ALL TESTS PASSED");
    else             $display("  SOME TESTS FAILED");
    $finish;
  end

  initial begin #80_000_000; $display("FAIL  WATCHDOG"); $finish; end

endmodule : asQspiTop_tb
