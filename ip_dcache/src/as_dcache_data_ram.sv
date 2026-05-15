// as_dcache_data_ram.sv  –  D-Cache data store — behavioral model (FPGA / simulation)
//
// ASIC target: replace with WAYS parallel X-Fab SRAM macros, each SETS entries × LINE_BITS wide.
// Same interface and pin mapping as as_icache_data_ram.sv.
//
// Read  : rd_addr_i in cycle N → rd_data_o (all WAYS) valid in cycle N+1.
// Write : wr_en_i=1, one way selected by wr_way_i.
`timescale 1ns/1ps

module as_dcache_data_ram #(
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

endmodule : as_dcache_data_ram
