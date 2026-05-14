`timescale 1ns/1ps
/* verilator lint_off INITIALDLY  */
/* verilator lint_off UNUSEDSIGNAL */

import as_pack::*;

// Testbench for asMemTop (complete memory subsystem).
//
// Address map used in this TB:
//   0x0000_0000 … 0x0FFF_FFFF  Flash (I-Cache + D-Cache via AXI4)
//   0x2000_0000 … 0x2000_1FFF  Scratchpad SRAM (8 KB, direct access)
//
// AXI4 memory model: rdata[beat] = { araddr+beat*8+4, araddr+beat*8 }
//   → every 32-bit word / 64-bit doubleword at address A equals A.
//
// TC01  SP ld: write+read 64-bit; lw sign-extend
// TC02  I-Cache cold miss: AXI4 fill, check instr + ARADDR
// TC03  I-Cache hit: same cache line, different offset
// TC04  D-Cache cold miss: AXI4 fill, check rdata + ARADDR
// TC05  D-Cache hit: same cache line, lw (zero-extend)
// TC06  SP lbu + lhu zero-extend
// TC07  D-Cache second miss (new line)
// TC08  I-Cache second miss (new line)
// TC09  I-Cache further hits in line filled by TC08
// TC10  SP partial write (wstrb), verify ld

module tb_asMemTop;

  localparam logic [31:0] SP_BASE = 32'h2000_0000;

  // -------------------------------------------------------------------------
  // Clock / reset
  // -------------------------------------------------------------------------
  localparam int CLK_HALF = 5;   // 100 MHz

  logic clk_s = 0;
  logic rst_s = 1;
  always #CLK_HALF clk_s = ~clk_s;

  // -------------------------------------------------------------------------
  // Interfaces
  // -------------------------------------------------------------------------
  as_icache_if icpu_if (.clk_i(clk_s), .rst_i(rst_s));
  as_dcache_if dcpu_if (.clk_i(clk_s), .rst_i(rst_s));
  as_axi4_if   qspi_if (.clk_i(clk_s), .rst_i(rst_s));

  // -------------------------------------------------------------------------
  // DUT
  // -------------------------------------------------------------------------
  asMemTop #(
    .CACHE_SIZE_B(4096),
    .WAYS        (4),
    .LINE_BYTES  (32),
    .PA_WIDTH    (32),
    .AXI_DW      (64),
    .SP_DEPTH    (1024),
    .SP_BASE     (SP_BASE)
  ) dut (
    .clk_i   (clk_s),
    .rst_i   (rst_s),
    .icpu_if (icpu_if),
    .dcpu_if (dcpu_if),
    .qspi_if (qspi_if)
  );

  // -------------------------------------------------------------------------
  // Test infrastructure
  // -------------------------------------------------------------------------
  int pass_cnt = 0;
  int fail_cnt = 0;

  task automatic chk(input logic cond, input string msg);
    if (cond) pass_cnt++;
    else begin
      $display("  FAIL [%0t ps] %s", $time, msg);
      fail_cnt++;
    end
  endtask

  task automatic chk_eq64(
    input logic [63:0] got, exp,
    input string msg
  );
    if (got === exp) pass_cnt++;
    else begin
      $display("  FAIL [%0t ps] %s: got=0x%016X exp=0x%016X", $time, msg, got, exp);
      fail_cnt++;
    end
  endtask

  task automatic chk_eq32(
    input logic [31:0] got, exp,
    input string msg
  );
    if (got === exp) pass_cnt++;
    else begin
      $display("  FAIL [%0t ps] %s: got=0x%08X exp=0x%08X", $time, msg, got, exp);
      fail_cnt++;
    end
  endtask

  // -------------------------------------------------------------------------
  // Signal initialisation
  // -------------------------------------------------------------------------
  task automatic init_signals();
    icpu_if.ic_req   = 0; icpu_if.ic_addr  = '0; icpu_if.ic_flush = 0;
    dcpu_if.dc_req   = 0; dcpu_if.dc_addr  = '0; dcpu_if.dc_wr   = 0;
    dcpu_if.dc_size  = 0; dcpu_if.dc_wdata = '0; dcpu_if.dc_wstrb = 0;
    dcpu_if.dc_flush = 0;
    qspi_if.arready  = 0;
    qspi_if.rvalid   = 0; qspi_if.rid      = 0;  qspi_if.rdata   = '0;
    qspi_if.rresp    = 0; qspi_if.rlast    = 0;
    qspi_if.awready  = 1; qspi_if.wready   = 1;
    qspi_if.bvalid   = 0; qspi_if.bid      = 0;  qspi_if.bresp   = 0;
  endtask

  // -------------------------------------------------------------------------
  // QSPI AXI4 read burst server (4 beats, no error injection)
  //   rdata model: beat b at araddr → { araddr+b*8+4, araddr+b*8 }
  // -------------------------------------------------------------------------
  task automatic qspi_serve(output logic [31:0] got_araddr);
    forever begin @(posedge clk_s); if (qspi_if.arvalid) break; end
    got_araddr = qspi_if.araddr[31:0];
    #1;
    qspi_if.arready = 1;
    @(posedge clk_s); #1;
    qspi_if.arready = 0;
    for (int b = 0; b < 4; b++) begin
      qspi_if.rvalid = 1;
      qspi_if.rid    = qspi_if.arid;
      qspi_if.rdata  = {32'(got_araddr + b*8 + 4), 32'(got_araddr + b*8)};
      qspi_if.rresp  = 2'b00;
      qspi_if.rlast  = (b == 3);
      forever begin @(posedge clk_s); if (qspi_if.rready) break; end
      #1;
    end
    qspi_if.rvalid = 0; qspi_if.rlast = 0;
    qspi_if.rid = 0; qspi_if.rdata = '0; qspi_if.rresp = 0;
  endtask

  // -------------------------------------------------------------------------
  // CPU I-fetch BFM (one-cycle ic_req pulse, wait for ic_rvalid)
  // -------------------------------------------------------------------------
  task automatic cpu_ifetch(
    input  logic [31:0] addr,
    output logic [31:0] rdata_out
  );
    icpu_if.ic_addr = 64'(addr);
    icpu_if.ic_req  = 1;
    @(posedge clk_s); #1;
    icpu_if.ic_req  = 0;
    icpu_if.ic_addr = '0;
    forever begin @(posedge clk_s); if (icpu_if.ic_rvalid) break; end
    #1;
    rdata_out = icpu_if.ic_rdata;
  endtask

  // -------------------------------------------------------------------------
  // CPU D-read BFM (one-cycle dc_req pulse, wait for dc_rvalid)
  // -------------------------------------------------------------------------
  task automatic cpu_dread(
    input  logic [31:0] addr,
    input  logic [2:0]  size,
    output logic [63:0] rdata_out
  );
    dcpu_if.dc_addr = 64'(addr);
    dcpu_if.dc_req  = 1;
    dcpu_if.dc_wr   = 0;
    dcpu_if.dc_size = size;
    @(posedge clk_s); #1;
    dcpu_if.dc_req  = 0;
    dcpu_if.dc_addr = '0;
    forever begin @(posedge clk_s); if (dcpu_if.dc_rvalid) break; end
    #1;
    rdata_out = dcpu_if.dc_rdata;
  endtask

  // -------------------------------------------------------------------------
  // CPU D-write BFM (one-cycle dc_req pulse, writes never stall)
  // -------------------------------------------------------------------------
  task automatic cpu_dwrite(
    input logic [31:0] addr,
    input logic [2:0]  size,
    input logic [63:0] wdata,
    input logic [7:0]  wstrb
  );
    dcpu_if.dc_addr  = 64'(addr);
    dcpu_if.dc_req   = 1;
    dcpu_if.dc_wr    = 1;
    dcpu_if.dc_size  = size;
    dcpu_if.dc_wdata = wdata;
    dcpu_if.dc_wstrb = wstrb;
    @(posedge clk_s); #1;
    dcpu_if.dc_req   = 0;
    dcpu_if.dc_wr    = 0;
    dcpu_if.dc_addr  = '0;
    dcpu_if.dc_wdata = '0;
    dcpu_if.dc_wstrb = '0;
    @(posedge clk_s); #1;
  endtask

  // =========================================================================
  // MAIN
  // =========================================================================
  initial begin
    logic [63:0] rdata;
    logic [31:0] instr;
    logic [31:0] got_araddr;

    $display("=== tb_asMemTop start ===");
    init_signals();

    rst_s = 1; repeat (4) @(posedge clk_s); #1;
    rst_s = 0; repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC01 – Scratchpad: ld write+read and lw sign-extend
    // ================================================================
    $display("TC01: SP ld write+read, lw sign-extend");
    cpu_dwrite(SP_BASE + 32'h0008, 3'b011, 64'hDEAD_BEEF_CAFE_BABE, 8'hFF);
    cpu_dread (SP_BASE + 32'h0008, 3'b011, rdata);      // ld
    chk_eq64(rdata, 64'hDEAD_BEEF_CAFE_BABE, "TC01: SP ld");
    cpu_dread (SP_BASE + 32'h0008, 3'b010, rdata);      // lw  (bit31=1)
    chk_eq64(rdata, 64'hFFFF_FFFF_CAFE_BABE, "TC01: SP lw sign");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC02 – I-Cache cold miss: AXI4 fill, check instr + ARADDR
    // ================================================================
    $display("TC02: I-Cache cold miss, addr 0x0000_0000");
    fork
      cpu_ifetch(32'h0000_0000, instr);
      qspi_serve(got_araddr);
    join
    chk_eq32(got_araddr, 32'h0000_0000, "TC02: ARADDR");
    chk_eq32(instr, 32'h0000_0000, "TC02: ic_rdata");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC03 – I-Cache hit: same line, different instruction offset
    // ================================================================
    $display("TC03: I-Cache hit, addr 0x0000_0004");
    cpu_ifetch(32'h0000_0004, instr);
    chk_eq32(instr, 32'h0000_0004, "TC03: ic_rdata hit");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC04 – D-Cache cold miss: AXI4 fill, check rdata + ARADDR
    //        ld at 0x0000_1000 → rdata = { 0x1004, 0x1000 }
    // ================================================================
    $display("TC04: D-Cache cold miss, addr 0x0000_1000 (ld)");
    fork
      cpu_dread(32'h0000_1000, 3'b011, rdata);
      qspi_serve(got_araddr);
    join
    chk_eq32(got_araddr, 32'h0000_1000, "TC04: ARADDR");
    chk_eq64(rdata, 64'h0000_1004_0000_1000, "TC04: dc_rdata ld");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC05 – D-Cache hit: same line, lw (zero-extend, bit31=0)
    //        lw at 0x0000_1004 → upper 32-bit of beat0 = 0x0000_1004
    // ================================================================
    $display("TC05: D-Cache hit, addr 0x0000_1004 (lw)");
    cpu_dread(32'h0000_1004, 3'b110, rdata);   // lwu
    chk_eq64(rdata, 64'h0000_0000_0000_1004, "TC05: dc_rdata lwu hit");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC06 – Scratchpad lbu + lhu zero-extend
    //        Write 0x0000_0000_8080_CAFE at SP+0x10 (sw, bytes 0-3)
    //        byte0=0xFE → lbu → 0x00FE
    //        halfword0=0xCAFE (bit15=1) → lhu → 0x0000_CAFE
    // ================================================================
    $display("TC06: SP lbu + lhu zero-extend");
    cpu_dwrite(SP_BASE + 32'h0010, 3'b010, 64'h0000_0000_8080_CAFE, 8'h0F);
    cpu_dread (SP_BASE + 32'h0010, 3'b100, rdata);   // lbu  byte0=0xFE
    chk_eq64(rdata, 64'h0000_0000_0000_00FE, "TC06: SP lbu");
    cpu_dread (SP_BASE + 32'h0010, 3'b101, rdata);   // lhu  half0=0xCAFE
    chk_eq64(rdata, 64'h0000_0000_0000_CAFE, "TC06: SP lhu");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC07 – D-Cache second miss (new line at 0x0000_2000)
    // ================================================================
    $display("TC07: D-Cache second miss, addr 0x0000_2000 (ld)");
    fork
      cpu_dread(32'h0000_2000, 3'b011, rdata);
      qspi_serve(got_araddr);
    join
    chk_eq32(got_araddr, 32'h0000_2000, "TC07: ARADDR");
    chk_eq64(rdata, 64'h0000_2004_0000_2000, "TC07: dc_rdata ld");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC08 – I-Cache second miss (new cache line at 0x0000_0100)
    // ================================================================
    $display("TC08: I-Cache second miss, addr 0x0000_0100");
    fork
      cpu_ifetch(32'h0000_0100, instr);
      qspi_serve(got_araddr);
    join
    chk_eq32(instr, 32'h0000_0100, "TC08: ic_rdata");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC09 – I-Cache hits within line filled by TC08
    //        instr at 0x0000_0104 and 0x0000_011C (both same line)
    // ================================================================
    $display("TC09: I-Cache hits in TC08 line");
    cpu_ifetch(32'h0000_0104, instr);
    chk_eq32(instr, 32'h0000_0104, "TC09: hit 0x104");
    cpu_ifetch(32'h0000_011C, instr);
    chk_eq32(instr, 32'h0000_011C, "TC09: hit 0x11C");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC10 – Scratchpad partial write (wstrb=0x55: bytes 0,2,4,6)
    //        Fresh word at SP+0x20; initial=0.
    //        Write 0xFFFF_FFFF_FFFF_FFFF, wstrb=0x55
    //        → bytes 0,2,4,6 = 0xFF; others = 0x00
    //        → ld = 0x00FF_00FF_00FF_00FF
    // ================================================================
    $display("TC10: SP partial write wstrb=0x55");
    cpu_dwrite(SP_BASE + 32'h0020, 3'b011, 64'hFFFF_FFFF_FFFF_FFFF, 8'h55);
    cpu_dread (SP_BASE + 32'h0020, 3'b011, rdata);
    chk_eq64(rdata, 64'h00FF_00FF_00FF_00FF, "TC10: SP partial write ld");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // Done
    // ================================================================
    if (fail_cnt == 0)
      $display("PASS: all %0d checks passed.", pass_cnt);
    else
      $display("FAIL: %0d/%0d checks failed.", fail_cnt, pass_cnt+fail_cnt);

    $finish;
  end

  // ── Timeout watchdog ─────────────────────────────────────────────────────
  initial begin
    #5_000_000;
    $fatal(1, "TIMEOUT: tb_asMemTop exceeded 5 ms");
  end

endmodule
