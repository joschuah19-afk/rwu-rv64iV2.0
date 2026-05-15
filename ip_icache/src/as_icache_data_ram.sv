// as_icache_data_ram.sv  –  I-Cache data store — behavioral model (FPGA / simulation)
//
// ASIC target: replace with WAYS parallel X-Fab SRAM macros, each SETS entries × LINE_BITS wide.
// Typical pin mapping per-way SRAM:
//   clk_i        → CLK
//   wr_en_i      → ~CEN / ~WEN  (active when writing this way)
//   wr_addr_i    → A
//   wr_data_i    → D
//   rd_addr_i    → A  (separate read port, or time-multiplex on 1-port SRAM)
//   rd_data_o[w] ← Q  (registered, valid 1 cycle after rd_addr_i)
//
// Interface contract:
//   Read  : rd_addr_i presented in cycle N → rd_data_o valid in cycle N+1 (all WAYS).
//   Write : wr_en_i=1 in cycle N → mem updated at posedge N; concurrent write+read to the
//           same address returns the OLD value (read-first; does not occur in normal cache use).
`timescale 1ns/1ps

module as_icache_data_ram #(
    parameter int SETS      = 32,
    parameter int WAYS      = 4,
    parameter int LINE_BITS = 256
)(
    input  logic clk_i,
    // Read port — all WAYS, registered
    input  logic [$clog2(SETS)-1:0]        rd_addr_i,
    output logic [WAYS-1:0][LINE_BITS-1:0] rd_data_o,
    // Write port — one way at a time
    input  logic                           wr_en_i,
    input  logic [$clog2(WAYS)-1:0]        wr_way_i,
    input  logic [$clog2(SETS)-1:0]        wr_addr_i,
    input  logic [LINE_BITS-1:0]           wr_data_i
);

    logic [LINE_BITS-1:0] mem [0:WAYS-1][0:SETS-1];

    always_ff @(posedge clk_i) begin
        if (wr_en_i)
            mem[wr_way_i][wr_addr_i] <= wr_data_i;
        for (int w = 0; w < WAYS; w++)
            rd_data_o[w] <= mem[w][rd_addr_i];
    end

endmodule : as_icache_data_ram
