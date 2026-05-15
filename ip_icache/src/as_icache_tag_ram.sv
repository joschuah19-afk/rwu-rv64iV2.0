// as_icache_tag_ram.sv  –  I-Cache tag store — behavioral model (FPGA / simulation)
//
// ASIC target: replace with WAYS parallel X-Fab SRAM macros, each SETS entries × TAG_BITS wide.
// Same pin mapping as as_icache_data_ram.sv; see that file for details.
`timescale 1ns/1ps

module as_icache_tag_ram #(
    parameter int SETS     = 32,
    parameter int WAYS     = 4,
    parameter int TAG_BITS = 22
)(
    input  logic clk_i,
    // Read port — all WAYS, registered
    input  logic [$clog2(SETS)-1:0]       rd_addr_i,
    output logic [WAYS-1:0][TAG_BITS-1:0] rd_data_o,
    // Write port — one way at a time
    input  logic                          wr_en_i,
    input  logic [$clog2(WAYS)-1:0]       wr_way_i,
    input  logic [$clog2(SETS)-1:0]       wr_addr_i,
    input  logic [TAG_BITS-1:0]           wr_data_i
);

    logic [TAG_BITS-1:0] mem [0:WAYS-1][0:SETS-1];

    always_ff @(posedge clk_i) begin
        if (wr_en_i)
            mem[wr_way_i][wr_addr_i] <= wr_data_i;
        for (int w = 0; w < WAYS; w++)
            rd_data_o[w] <= mem[w][rd_addr_i];
    end

endmodule : as_icache_tag_ram
