// as_icache_valid_reg.sv  –  I-Cache valid flags — FF register file
//
// ASIC target: remains as standard-cell flip-flops (SETS×WAYS = 128 bits; too small for SRAM).
// The sub-module boundary is kept for design consistency.
//
// Read  : combinatorial (no latency); rd_data_o reflects valid at rd_addr_i immediately.
// Write : single way via wr_en_i / wr_way_i / wr_addr_i.
// Flush : flush_en_i clears all WAYS at flush_addr_i in one cycle (used by I-Cache FLUSH FSM state).
`timescale 1ns/1ps

module as_icache_valid_reg #(
    parameter int SETS = 32,
    parameter int WAYS = 4
)(
    input  logic clk_i,
    input  logic rst_i,
    // Combinatorial read — all WAYS at rd_addr_i
    input  logic [$clog2(SETS)-1:0] rd_addr_i,
    output logic [WAYS-1:0]         rd_data_o,
    // Single-way write
    input  logic                    wr_en_i,
    input  logic [$clog2(WAYS)-1:0] wr_way_i,
    input  logic [$clog2(SETS)-1:0] wr_addr_i,
    input  logic                    wr_data_i,
    // All-ways clear (flush)
    input  logic                    flush_en_i,
    input  logic [$clog2(SETS)-1:0] flush_addr_i
);

    logic valid_r [0:SETS-1][0:WAYS-1];

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (int s = 0; s < SETS; s++)
                for (int w = 0; w < WAYS; w++)
                    valid_r[s][w] <= '0;
        end else begin
            if (flush_en_i)
                for (int w = 0; w < WAYS; w++)
                    valid_r[flush_addr_i][w] <= '0;
            else if (wr_en_i)
                valid_r[wr_addr_i][wr_way_i] <= wr_data_i;
        end
    end

    always_comb
        for (int w = 0; w < WAYS; w++)
            rd_data_o[w] = valid_r[rd_addr_i][w];

endmodule : as_icache_valid_reg
