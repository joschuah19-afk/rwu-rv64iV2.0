`timescale 1ns/1ps
import as_pack::*;

// asMemTop – Complete Memory Subsystem
//
// Integrates I-Cache, D-Cache, Scratchpad SRAM and Memory Arbiter.
//
// CPU I-bus → asICache → AXI4 → asMemArb → qspi_if (QSPI controller)
// CPU D-bus → address decode:
//   Scratchpad region  → asScratch  (no AXI4)
//   Flash region       → asDCache   → AXI4 → asMemArb → qspi_if
//
// Address decode (D-bus):
//   if dc_addr[PA_WIDTH-1:0] in [SP_BASE, SP_BASE + SP_DEPTH*8)
//       → scratchpad (synchronous, 1-cycle read stall)
//   else
//       → D-Cache (set-associative, AXI4 fill on miss)
//
// D-Cache is read-only in this system (Flash region, CPU stores go to
// scratchpad). The D-Cache dirty-eviction path in asMemArb is tied off
// with awready/wready=1 and bvalid=0; EVICT states are never reached.
//
// Intended future parent: module as_top_mem
//
// Spec: 03_arch_memory_concept.tex

module asMemTop #(
    parameter int             CACHE_SIZE_B = 4096,
    parameter int             WAYS         = 4,
    parameter int             LINE_BYTES   = 32,
    parameter int             PA_WIDTH     = 32,
    parameter int             AXI_DW       = 64,
    parameter int             SP_DEPTH     = 1024,
    parameter logic [31:0]    SP_BASE      = 32'h2000_0000   // scratchpad base address
) (
    input  logic       clk_i,
    input  logic       rst_i,
    as_icache_if.cache icpu_if,    // CPU instruction bus
    as_dcache_if.cache dcpu_if,    // CPU data bus
    as_axi4_if.master  qspi_if    // AXI4 to QSPI controller (AXI4 slave)
);

    // ── Scratchpad address range ─────────────────────────────────
    localparam int             SP_SIZE_B = SP_DEPTH * 8;
    localparam logic [31:0]    SP_TOP_C  = SP_BASE + 32'(SP_SIZE_B);

    // Combinatorial: is the current D-bus address in the scratchpad?
    logic sp_sel_s;
    assign sp_sel_s = (dcpu_if.dc_addr[PA_WIDTH-1:0] >= SP_BASE) &&
                      (dcpu_if.dc_addr[PA_WIDTH-1:0] <  SP_TOP_C);

    // Registered: which module owns the current in-flight D-bus request.
    // Updated on every dc_req pulse so the output mux stays correct during
    // the multi-cycle response phase (when dc_req=0, dc_addr may change).
    logic sp_req_r;
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) sp_req_r <= 1'b0;
        else if (dcpu_if.dc_req) sp_req_r <= sp_sel_s;
    end

    // Active module: sp_sel_s when a new request is being issued,
    // sp_req_r while waiting for the response (dc_req=0).
    logic sp_active_s;
    assign sp_active_s = dcpu_if.dc_req ? sp_sel_s : sp_req_r;

    // ── Internal interfaces ───────────────────────────────────────
    as_dcache_if dcache_if (.clk_i(clk_i), .rst_i(rst_i));
    as_dcache_if scratch_if(.clk_i(clk_i), .rst_i(rst_i));
    as_axi4_if   icache_axi(.clk_i(clk_i), .rst_i(rst_i));
    as_axi4_if   dcache_axi(.clk_i(clk_i), .rst_i(rst_i));

    // ── D-bus → D-Cache (gated: only active for flash-region requests) ──
    assign dcache_if.dc_addr  = dcpu_if.dc_addr;
    assign dcache_if.dc_req   = dcpu_if.dc_req & ~sp_sel_s;
    assign dcache_if.dc_wr    = dcpu_if.dc_wr;
    assign dcache_if.dc_size  = dcpu_if.dc_size;
    assign dcache_if.dc_wdata = dcpu_if.dc_wdata;
    assign dcache_if.dc_wstrb = dcpu_if.dc_wstrb;
    assign dcache_if.dc_flush = dcpu_if.dc_flush;

    // ── D-bus → Scratchpad (gated: only active for SP-region requests) ──
    assign scratch_if.dc_addr  = dcpu_if.dc_addr;
    assign scratch_if.dc_req   = dcpu_if.dc_req & sp_sel_s;
    assign scratch_if.dc_wr    = dcpu_if.dc_wr;
    assign scratch_if.dc_size  = dcpu_if.dc_size;
    assign scratch_if.dc_wdata = dcpu_if.dc_wdata;
    assign scratch_if.dc_wstrb = dcpu_if.dc_wstrb;
    assign scratch_if.dc_flush = 1'b0;   // scratchpad has no flush

    // ── D-bus → CPU outputs (muxed by sp_active_s) ───────────────
    assign dcpu_if.dc_stall      = sp_active_s ? scratch_if.dc_stall      : dcache_if.dc_stall;
    assign dcpu_if.dc_rvalid     = sp_active_s ? scratch_if.dc_rvalid     : dcache_if.dc_rvalid;
    assign dcpu_if.dc_rdata      = sp_active_s ? scratch_if.dc_rdata      : dcache_if.dc_rdata;
    assign dcpu_if.dc_err        = sp_active_s ? scratch_if.dc_err        : dcache_if.dc_err;
    assign dcpu_if.dc_flush_done = dcache_if.dc_flush_done;

    // ── I-Cache ──────────────────────────────────────────────────
    asICache #(
        .CACHE_SIZE_B(CACHE_SIZE_B),
        .WAYS        (WAYS),
        .LINE_BYTES  (LINE_BYTES),
        .PA_WIDTH    (PA_WIDTH),
        .AXI_DW      (AXI_DW)
    ) icache (
        .clk_i  (clk_i),
        .rst_i  (rst_i),
        .cpu_if (icpu_if),
        .axi_if (icache_axi)
    );

    // ── D-Cache ──────────────────────────────────────────────────
    asDCache #(
        .CACHE_SIZE_B(CACHE_SIZE_B),
        .WAYS        (WAYS),
        .LINE_BYTES  (LINE_BYTES),
        .PA_WIDTH    (PA_WIDTH),
        .AXI_DW      (AXI_DW)
    ) dcache (
        .clk_i  (clk_i),
        .rst_i  (rst_i),
        .cpu_if (dcache_if),
        .axi_if (dcache_axi)
    );

    // ── Scratchpad SRAM ──────────────────────────────────────────
    asScratch #(
        .SP_DEPTH(SP_DEPTH),
        .PA_WIDTH(PA_WIDTH)
    ) scratch (
        .clk_i  (clk_i),
        .rst_i  (rst_i),
        .cpu_if (scratch_if)
    );

    // ── Memory Arbiter (I-Cache + D-Cache → QSPI) ────────────────
    asMemArb arb (
        .clk_i      (clk_i),
        .rst_i      (rst_i),
        .icache_axi4(icache_axi),
        .dcache_axi4(dcache_axi),
        .qspi_axi4  (qspi_if)
    );

endmodule : asMemTop
