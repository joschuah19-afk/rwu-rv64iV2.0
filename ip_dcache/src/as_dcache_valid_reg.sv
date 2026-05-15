// as_dcache_valid_reg.sv  –  D-Cache valid + dirty flags — FF register file
//
// ASIC target: remains as standard-cell flip-flops (SETS×WAYS×2 = 256 bits; too small for SRAM).
//
// Read  : combinatorial; rd_valid_o / rd_dirty_o reflect the state at rd_addr_i immediately.
// Write : single (set,way) at a time — covers hit-write (dirty←1), fill (valid←1, dirty←wr),
//         and flush invalidation (valid←0, dirty←0).
`timescale 1ns/1ps

module as_dcache_valid_reg #(
    parameter int SETS = 32,
    parameter int WAYS = 4
)(
    input  logic clk_i,
    input  logic rst_i,
    // Combinatorial read — all WAYS at rd_addr_i
    input  logic [$clog2(SETS)-1:0] rd_addr_i,
    output logic [WAYS-1:0]         rd_valid_o,
    output logic [WAYS-1:0]         rd_dirty_o,
    // Write (one way)
    input  logic                    wr_en_i,
    input  logic [$clog2(WAYS)-1:0] wr_way_i,
    input  logic [$clog2(SETS)-1:0] wr_addr_i,
    input  logic                    wr_valid_i,
    input  logic                    wr_dirty_i
);

    logic valid_r [0:SETS-1][0:WAYS-1];
    logic dirty_r [0:SETS-1][0:WAYS-1];

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (int s = 0; s < SETS; s++)
                for (int w = 0; w < WAYS; w++) begin
                    valid_r[s][w] <= '0;
                    dirty_r[s][w] <= '0;
                end
        end else if (wr_en_i) begin
            valid_r[wr_addr_i][wr_way_i] <= wr_valid_i;
            dirty_r[wr_addr_i][wr_way_i] <= wr_dirty_i;
        end
    end

    always_comb
        for (int w = 0; w < WAYS; w++) begin
            rd_valid_o[w] = valid_r[rd_addr_i][w];
            rd_dirty_o[w] = dirty_r[rd_addr_i][w];
        end

endmodule : as_dcache_valid_reg
