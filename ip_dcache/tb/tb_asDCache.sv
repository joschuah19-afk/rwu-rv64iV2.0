`timescale 1ns/1ps
/* verilator lint_off INITIALDLY  */
/* verilator lint_off UNUSEDSIGNAL */

import as_pack::*;

// Testbench for asDCache.
//
// Memory model: each 64-bit doubleword at 8-byte-aligned address D holds
//   {32'(D+4), 32'(D)}   (both 32-bit halves = their own byte address).
//
// Test cases:
//   TC01  Cold read miss  (ld): fill via AXI4, check rdata
//   TC02  Read hit        (ld): same address, no AXI4
//   TC03  Read hit        (lw): word at offset 4 in same line
//   TC04  Write hit       (sw): merge with byte-enable, cache dirty
//   TC05  Read after write: verify merged value from dirty cache line
//   TC06  Write miss / write-allocate: fill + merge, no explicit rdata
//   TC07  Read after write-allocate: verify merged value
//   TC08  Dirty eviction: 5th unique tag in set 0 forces write-back then fill
//   TC09  Flush: all dirty lines written back, then re-fetch misses
//   TC10  AXI4 read error  (RRESP): ic_err pulse, no rvalid
//   TC11  AXI4 write error (BRESP): dc_err pulse during eviction

module tb_asDCache;

  // -------------------------------------------------------------------------
  // Clock / reset
  // -------------------------------------------------------------------------
  localparam int CLK_HALF = 5;  // 100 MHz

  logic clk_s = 0;
  logic rst_s = 1;
  always #CLK_HALF clk_s = ~clk_s;

  // -------------------------------------------------------------------------
  // Interfaces
  // -------------------------------------------------------------------------
  as_dcache_if cpu_if (.clk_i(clk_s), .rst_i(rst_s));
  as_axi4_if   axi_if (.clk_i(clk_s), .rst_i(rst_s));

  // -------------------------------------------------------------------------
  // DUT
  // -------------------------------------------------------------------------
  asDCache dut (
    .clk_i  (clk_s),
    .rst_i  (rst_s),
    .cpu_if (cpu_if),
    .axi_if (axi_if)
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
    cpu_if.dc_req   = 0; cpu_if.dc_addr  = '0; cpu_if.dc_wr   = 0;
    cpu_if.dc_size  = 0; cpu_if.dc_wdata = '0; cpu_if.dc_wstrb = 0;
    cpu_if.dc_flush = 0;
    axi_if.arready  = 0;
    axi_if.rvalid   = 0; axi_if.rid      = 0;  axi_if.rdata   = '0;
    axi_if.rresp    = 0; axi_if.rlast    = 0;
    axi_if.awready  = 1;
    axi_if.wready   = 1;
    axi_if.bvalid   = 0; axi_if.bid      = 0;  axi_if.bresp   = 0;
  endtask

  // -------------------------------------------------------------------------
  // AXI4 read burst server
  //   arready_dly : extra cycles to stall before asserting arready
  //   err_beat    : beat on which to inject RRESP SLVERR (-1 = none)
  //   got_araddr  : output: address presented on ARADDR
  //
  //   Data model: beat B at address BA → {BA+4, BA}
  // -------------------------------------------------------------------------
  task automatic axi_rd_serve(
    input  int          arready_dly,
    input  int          err_beat,
    output logic [31:0] got_araddr
  );
    forever begin @(posedge clk_s); if (axi_if.arvalid) break; end
    got_araddr = axi_if.araddr[31:0];
    if (arready_dly > 0) repeat (arready_dly) @(posedge clk_s);
    #1;
    axi_if.arready = 1;
    @(posedge clk_s); #1;
    axi_if.arready = 0;
    for (int b = 0; b < 4; b++) begin
      axi_if.rvalid = 1;
      axi_if.rid    = 4'h2;
      axi_if.rdata  = {32'(got_araddr + b*8 + 4), 32'(got_araddr + b*8)};
      axi_if.rresp  = (b == err_beat) ? 2'b10 : 2'b00;
      axi_if.rlast  = (b == 3) || (b == err_beat);
      forever begin @(posedge clk_s); if (axi_if.rready) break; end
      #1;
      if (b == err_beat) break;
    end
    axi_if.rvalid = 0; axi_if.rlast = 0;
    axi_if.rid = 0; axi_if.rdata = '0; axi_if.rresp = 0;
  endtask

  // -------------------------------------------------------------------------
  // AXI4 write burst acceptor
  //   Returns: awaddr and the 256-bit line written (4 × 64-bit beats)
  //   bresp_val: response to send (2'b00=OKAY, 2'b10=SLVERR)
  // -------------------------------------------------------------------------
  task automatic axi_wr_serve(
    input  logic [1:0]   bresp_val,
    output logic [31:0]  got_awaddr,
    output logic [255:0] got_wdata
  );
    // awready is already 1 from init_signals; handshake completes the cycle awvalid rises
    forever begin @(posedge clk_s); if (axi_if.awvalid) break; end
    got_awaddr = axi_if.awaddr[31:0];
    #1;
    axi_if.awready = 0;  // deassert after handshake without waiting a full cycle
    got_wdata = '0;
    for (int b = 0; b < 4; b++) begin
      forever begin @(posedge clk_s); if (axi_if.wvalid && axi_if.wready) break; end
      got_wdata[b*64 +: 64] = axi_if.wdata;
      if (axi_if.wlast) break;
    end
    #1;
    axi_if.bvalid = 1;
    axi_if.bid    = 4'h2;
    axi_if.bresp  = bresp_val;
    forever begin @(posedge clk_s); if (axi_if.bready) break; end
    #1;
    axi_if.bvalid = 0; axi_if.bid = 0; axi_if.bresp = 0;
    axi_if.awready = 1;  // restore for next transaction
  endtask

  // -------------------------------------------------------------------------
  // CPU read BFM: one-cycle dc_req pulse, then wait for dc_rvalid
  // -------------------------------------------------------------------------
  task automatic cpu_read(
    input  logic [31:0] addr,
    input  logic [2:0]  size,
    output logic [63:0] rdata_out
  );
    cpu_if.dc_addr  = 64'(addr);
    cpu_if.dc_req   = 1;
    cpu_if.dc_wr    = 0;
    cpu_if.dc_size  = size;
    @(posedge clk_s); #1;
    cpu_if.dc_req   = 0;
    cpu_if.dc_addr  = '0;
    forever begin @(posedge clk_s); if (cpu_if.dc_rvalid) break; end
    #1;
    rdata_out = cpu_if.dc_rdata;
  endtask

  // -------------------------------------------------------------------------
  // CPU write BFM: one-cycle dc_req pulse, wait for dc_stall to deassert
  // -------------------------------------------------------------------------
  task automatic cpu_write(
    input logic [31:0] addr,
    input logic [2:0]  size,
    input logic [63:0] wdata,
    input logic [7:0]  wstrb
  );
    cpu_if.dc_addr  = 64'(addr);
    cpu_if.dc_req   = 1;
    cpu_if.dc_wr    = 1;
    cpu_if.dc_size  = size;
    cpu_if.dc_wdata = wdata;
    cpu_if.dc_wstrb = wstrb;
    @(posedge clk_s); #1;
    cpu_if.dc_req   = 0;
    cpu_if.dc_wr    = 0;
    cpu_if.dc_addr  = '0;
    cpu_if.dc_wdata = '0;
    cpu_if.dc_wstrb = '0;
    // Wait for stall to deassert (write complete)
    forever begin @(posedge clk_s); if (!cpu_if.dc_stall) break; end
    #1;
  endtask

  // -------------------------------------------------------------------------
  // Memory model helpers
  // -------------------------------------------------------------------------
  // 64-bit doubleword at 8B-aligned address D
  function automatic logic [63:0] mem_dw(input logic [31:0] addr);
    automatic logic [31:0] d = addr & ~32'h7;
    return {d + 4, d};
  endfunction

  // Expected result for ld (full 64-bit doubleword)
  function automatic logic [63:0] exp_ld(input logic [31:0] addr);
    return mem_dw(addr);
  endfunction

  // Expected result for lw (sign-extended 32-bit)
  function automatic logic [63:0] exp_lw(input logic [31:0] addr);
    automatic logic [31:0] d = addr & ~32'h7;
    if (addr[2])  // upper half
      return {{32{1'b0}}, d + 4};  // value = d+4; non-negative in test range
    else
      return {{32{1'b0}}, d};
  endfunction

  // Cache-line aligned address
  function automatic logic [31:0] line_addr(input logic [31:0] addr);
    return addr & ~32'h1F;
  endfunction

  // =========================================================================
  // MAIN
  // =========================================================================
  initial begin
    logic [63:0]  rdata;
    logic [31:0]  got_araddr;
    logic [31:0]  got_awaddr;
    logic [255:0] got_wdata;

    $display("=== tb_asDCache start ===");
    init_signals();

    rst_s = 1; repeat (4) @(posedge clk_s); #1;
    rst_s = 0; repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC01 – Cold read miss (ld): fill via AXI4
    // ================================================================
    $display("TC01: cold read miss, ld 0x0000_0000");
    fork
      cpu_read(32'h0000_0000, 3'b011, rdata);   // ld
      axi_rd_serve(0, -1, got_araddr);
    join
    chk_eq64(rdata,     exp_ld(32'h0000_0000), "TC01: rdata");
    chk_eq32(got_araddr, line_addr(32'h0000_0000), "TC01: ARADDR");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC02 – Read hit (ld): same address, no AXI4 transaction
    // ================================================================
    $display("TC02: read hit, ld 0x0000_0000");
    cpu_read(32'h0000_0000, 3'b011, rdata);
    chk_eq64(rdata, exp_ld(32'h0000_0000), "TC02: rdata");
    chk(axi_if.arvalid === 1'b0, "TC02: no AXI4 read");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC03 – Read hit (lw): word at offset 4 in same line
    // ================================================================
    $display("TC03: read hit, lw 0x0000_0004");
    cpu_read(32'h0000_0004, 3'b010, rdata);   // lw, byte_off=4
    chk_eq64(rdata, exp_lw(32'h0000_0004), "TC03: lw rdata");
    chk(axi_if.arvalid === 1'b0, "TC03: no AXI4 read");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC04 – Write hit (sw): store 0xDEAD_BEEF at offset 0 in line 0
    //   addr=0x0, sw → wstrb=8'h0F, wdata[31:0]=0xDEAD_BEEF
    // ================================================================
    $display("TC04: write hit, sw 0xDEAD_BEEF → 0x0000_0000");
    cpu_write(32'h0000_0000, 3'b010, 64'hDEAD_BEEF, 8'h0F);
    chk(axi_if.awvalid === 1'b0, "TC04: no AXI4 write (hit, stays dirty)");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC05 – Read after write: verify merged value
    //   ld 0x0: upper 32 bits unchanged (0x4), lower 32 = 0xDEAD_BEEF
    // ================================================================
    $display("TC05: read after write, ld 0x0000_0000");
    cpu_read(32'h0000_0000, 3'b011, rdata);
    chk_eq64(rdata, 64'h0000_0004_DEAD_BEEF, "TC05: dirty read rdata");
    chk(axi_if.arvalid === 1'b0, "TC05: no AXI4 (cache hit)");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC06 – Write miss / write-allocate: sw to set 1 (not yet cached)
    //   addr=0x0000_0028, dw_sel=1, sw at byte offset 0 within dw1
    //   wstrb=8'h0F, wdata[31:0]=0xCAFE_BABE
    // ================================================================
    $display("TC06: write miss write-allocate, sw 0xCAFE_BABE → 0x0000_0028");
    fork
      cpu_write(32'h0000_0028, 3'b010, 64'hCAFE_BABE, 8'h0F);
      axi_rd_serve(0, -1, got_araddr);
    join
    chk_eq32(got_araddr, line_addr(32'h0000_0028), "TC06: ARADDR");
    chk(axi_if.awvalid === 1'b0, "TC06: no write-back (victim was clean)");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC07 – Read after write-allocate: lw at 0x0000_0028
    //   dw1[31:0] was 0x28 (from memory model) → overwritten with 0xCAFE_BABE
    // ================================================================
    $display("TC07: read after write-allocate, lw 0x0000_0028");
    cpu_read(32'h0000_0028, 3'b010, rdata);
    chk_eq64(rdata, 64'hFFFF_FFFF_CAFE_BABE, "TC07: merged lw rdata");
    chk(axi_if.arvalid === 1'b0, "TC07: cache hit after write-allocate");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC08 – Dirty eviction: fill all 4 ways of set 0 with clean lines,
    //        then write to way 0 (make dirty), then bring in a 5th tag.
    //        Expected: write-back of dirty way 0, then fill new line.
    //
    //        Set 0 lines: tag step = 0x400 (1024 B)
    //          way 0 already filled (TC01): 0x0000_0000, now DIRTY (TC04)
    //          way 1: 0x0000_0400 (tag=1)
    //          way 2: 0x0000_0800 (tag=2)
    //          way 3: 0x0000_0C00 (tag=3)
    //          5th:   0x0000_1000 (tag=4) → evicts PLRU victim (should be way 0 dirty)
    // ================================================================
    $display("TC08: dirty eviction – fill ways 1-3 of set 0 (clean)");
    fork cpu_read(32'h0000_0400, 3'b011, rdata); axi_rd_serve(0, -1, got_araddr); join
    chk_eq64(rdata, exp_ld(32'h0000_0400), "TC08 way1: rdata");
    repeat (1) @(posedge clk_s); #1;
    fork cpu_read(32'h0000_0800, 3'b011, rdata); axi_rd_serve(0, -1, got_araddr); join
    chk_eq64(rdata, exp_ld(32'h0000_0800), "TC08 way2: rdata");
    repeat (1) @(posedge clk_s); #1;
    fork cpu_read(32'h0000_0C00, 3'b011, rdata); axi_rd_serve(0, -1, got_araddr); join
    chk_eq64(rdata, exp_ld(32'h0000_0C00), "TC08 way3: rdata");
    repeat (1) @(posedge clk_s); #1;

    $display("TC08: 5th tag triggers dirty eviction + fill");
    fork
      cpu_read(32'h0000_1000, 3'b011, rdata);
      begin
        // Expect write-back of dirty line 0x0000_0000
        axi_wr_serve(2'b00, got_awaddr, got_wdata);
        // Then expect fill of new line 0x0000_1000
        axi_rd_serve(0, -1, got_araddr);
      end
    join
    chk_eq32(got_awaddr, 32'h0000_0000, "TC08: evict AWADDR = line 0");
    // Verify the dirty merged value (TC04 wrote 0xDEAD_BEEF to dw0[31:0])
    chk_eq64(got_wdata[63:0], 64'h0000_0004_DEAD_BEEF, "TC08: evict dw0 has merged data");
    chk_eq32(got_araddr, 32'h0000_1000, "TC08: fill ARADDR = new tag");
    chk_eq64(rdata, exp_ld(32'h0000_1000), "TC08: rdata after fill");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC09 – Flush: write-back all dirty lines, then re-fetch misses
    // ================================================================
    $display("TC09: flush");
    // After TC08: set 0 has ways 1-3 (clean) + way at tag4 (clean new fill)
    //             set 1 has addr 0x0028 dirty (from TC06)
    // Flush should write back the dirty line in set 1 (addr 0x0000_0020)
    fork
      begin
        cpu_if.dc_flush = 1;
        @(posedge clk_s); #1;
        cpu_if.dc_flush = 0;
        forever begin @(posedge clk_s); if (cpu_if.dc_flush_done) break; end
        chk(cpu_if.dc_flush_done === 1'b1, "TC09: flush_done asserted");
      end
      begin
        // Accept the write-back of set 1's dirty line
        axi_wr_serve(2'b00, got_awaddr, got_wdata);
        // got_awaddr should be set 1's line: 0x0000_0020
        chk_eq32(got_awaddr, 32'h0000_0020, "TC09: flush write-back address");
      end
    join
    repeat (2) @(posedge clk_s); #1;

    // Re-fetch: TC01 address must miss
    $display("TC09: re-fetch after flush (expect miss)");
    fork
      cpu_read(32'h0000_0000, 3'b011, rdata);
      axi_rd_serve(0, -1, got_araddr);
    join
    chk_eq64(rdata, exp_ld(32'h0000_0000), "TC09: rdata after re-fetch");
    chk_eq32(got_araddr, 32'h0000_0000,   "TC09: ARADDR after flush");
    repeat (2) @(posedge clk_s); #1;

    // ================================================================
    // TC10 – AXI4 read error (RRESP SLVERR on beat 0)
    // ================================================================
    $display("TC10: AXI4 read error");
    fork
      begin
        cpu_if.dc_addr = 64'h0000_0040;  // set 2, not cached
        cpu_if.dc_req  = 1;
        cpu_if.dc_wr   = 0;
        cpu_if.dc_size = 3'b011;
        @(posedge clk_s); #1;
        cpu_if.dc_req  = 0;
        cpu_if.dc_addr = '0;
        forever begin @(posedge clk_s); if (cpu_if.dc_err) break; end
      end
      axi_rd_serve(0, 0, got_araddr);   // error on beat 0
    join
    chk(cpu_if.dc_err    === 1'b1, "TC10: dc_err pulsed");
    chk(cpu_if.dc_rvalid === 1'b0, "TC10: no rvalid on read error");
    repeat (4) @(posedge clk_s); #1;

    // ================================================================
    // TC11 – AXI4 write error (BRESP SLVERR during eviction)
    // ================================================================
    $display("TC11: AXI4 write error during eviction");
    // First make set 2, way 0 dirty: write to 0x0000_0040 (fill via AXI4, write-allocate)
    fork
      cpu_write(32'h0000_0040, 3'b011, 64'hAAAA_BBBB_CCCC_DDDD, 8'hFF);
      axi_rd_serve(0, -1, got_araddr);
    join
    repeat (1) @(posedge clk_s); #1;
    // Fill 3 more ways of set 2 to force eviction of dirty way
    fork cpu_read(32'h0000_0440, 3'b011, rdata); axi_rd_serve(0,-1,got_araddr); join
    repeat (1) @(posedge clk_s); #1;
    fork cpu_read(32'h0000_0840, 3'b011, rdata); axi_rd_serve(0,-1,got_araddr); join
    repeat (1) @(posedge clk_s); #1;
    fork cpu_read(32'h0000_0C40, 3'b011, rdata); axi_rd_serve(0,-1,got_araddr); join
    repeat (1) @(posedge clk_s); #1;
    // 5th tag triggers eviction – inject BRESP error
    fork
      begin
        cpu_if.dc_addr = 64'h0000_1040;
        cpu_if.dc_req  = 1;
        cpu_if.dc_wr   = 0;
        cpu_if.dc_size = 3'b011;
        @(posedge clk_s); #1;
        cpu_if.dc_req  = 0;
        cpu_if.dc_addr = '0;
        forever begin @(posedge clk_s); if (cpu_if.dc_err) break; end
      end
      axi_wr_serve(2'b10, got_awaddr, got_wdata);  // SLVERR on eviction
    join
    chk(cpu_if.dc_err    === 1'b1, "TC11: dc_err on write-back error");
    chk(cpu_if.dc_rvalid === 1'b0, "TC11: no rvalid on error");
    repeat (4) @(posedge clk_s); #1;

    // ================================================================
    // RESULT
    // ================================================================
    $display("=== tb_asDCache done ===");
    if (fail_cnt == 0)
      $display("PASS: all %0d checks passed.", pass_cnt);
    else
      $display("FAIL: %0d of %0d checks failed.", fail_cnt, pass_cnt + fail_cnt);
    $finish;
  end

  // Watchdog
  initial begin
    #2_000_000;
    $fatal(1, "TIMEOUT: tb_asDCache exceeded 2 ms");
  end

endmodule : tb_asDCache
