`timescale 1ns/1ps
import as_pack::*;

// 4-way set-associative instruction cache (read-only, no write-back).
//
// Address decomposition (default: PA=32, 4 KB, 4-way, 32 B line):
//   [31:10] = tag (22 bits)
//   [9:5]   = set index (5 bits)
//   [4:2]   = instruction selector within cache line (3 bits)
//   [1:0]   = byte offset within 32-bit instruction (ignored)
//
// AXI4 master: ID=4'h1, ARLEN=3, ARSIZE=3, ARBURST=INCR (read channels only)
//
// SRAM sub-modules:
//   as_icache_data_ram  – data store (WAYS × LINE_BITS, registered output)
//   as_icache_tag_ram   – tag store  (WAYS × TAG_BITS,  registered output)
//   as_icache_valid_reg – valid flags (WAYS × 1, combinatorial output, FF-based)
// For ASIC: replace the three *_ram modules with X-Fab SRAM wrappers.
// Read timing: address driven in IDLE → registered output valid in LOOKUP.

module asICache #(
  parameter int CACHE_SIZE_B = 4096,
  parameter int WAYS         = 4,
  parameter int LINE_BYTES   = 32,
  parameter int PA_WIDTH     = 32,
  parameter int AXI_DW       = 64
)(
  input  logic       clk_i,
  input  logic       rst_i,
  as_icache_if.cache cpu_if,
  as_axi4_if.master  axi_if
);

  // -------------------------------------------------------------------------
  // Derived parameters
  // -------------------------------------------------------------------------
  localparam int SETS        = CACHE_SIZE_B / (WAYS * LINE_BYTES);
  localparam int OFFSET_BITS = $clog2(LINE_BYTES);
  localparam int INDEX_BITS  = $clog2(SETS);
  localparam int TAG_BITS    = PA_WIDTH - INDEX_BITS - OFFSET_BITS;
  localparam int BEATS       = LINE_BYTES / (AXI_DW / 8);
  localparam int BEAT_BITS   = $clog2(BEATS);
  localparam int LINE_BITS   = LINE_BYTES * 8;

  // -------------------------------------------------------------------------
  // Address decomposition (combinatorial, from live CPU request)
  // -------------------------------------------------------------------------
  logic [TAG_BITS-1:0]   req_tag_s;
  logic [INDEX_BITS-1:0] req_idx_s;
  logic [2:0]            req_instr_s;

  assign req_tag_s   = cpu_if.ic_addr[PA_WIDTH-1 : INDEX_BITS+OFFSET_BITS];
  assign req_idx_s   = cpu_if.ic_addr[INDEX_BITS+OFFSET_BITS-1 : OFFSET_BITS];
  assign req_instr_s = cpu_if.ic_addr[OFFSET_BITS-1 : 2];

  // -------------------------------------------------------------------------
  // SRAM / register-file output wires
  // -------------------------------------------------------------------------
  logic [WAYS-1:0][LINE_BITS-1:0] data_rdata_s;  // registered, all WAYS
  logic [WAYS-1:0][TAG_BITS-1:0]  tag_rdata_s;   // registered, all WAYS
  logic [WAYS-1:0]                valid_rdata_s;  // combinatorial, all WAYS

  // -------------------------------------------------------------------------
  // SRAM write-port control (combinatorial decode from FSM state)
  // -------------------------------------------------------------------------
  logic                    data_wr_en_s;
  logic [$clog2(WAYS)-1:0] data_wr_way_s;
  logic [INDEX_BITS-1:0]   data_wr_addr_s;
  logic [LINE_BITS-1:0]    data_wr_data_s;

  logic                    tag_wr_en_s;
  logic [$clog2(WAYS)-1:0] tag_wr_way_s;
  logic [INDEX_BITS-1:0]   tag_wr_addr_s;
  logic [TAG_BITS-1:0]     tag_wr_data_s;

  logic                    valid_wr_en_s;
  logic [$clog2(WAYS)-1:0] valid_wr_way_s;
  logic [INDEX_BITS-1:0]   valid_wr_addr_s;

  logic                    valid_flush_en_s;
  logic [INDEX_BITS-1:0]   valid_flush_addr_s;

  // Fill line assembly (combinatorial from fill_buf_r + AXI beat)
  logic [LINE_BITS-1:0] fill_line_s;

  // -------------------------------------------------------------------------
  // FSM
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    IDLE    = 3'd0,
    LOOKUP  = 3'd1,
    MISS    = 3'd2,
    FILL    = 3'd3,
    RESPOND = 3'd4,
    ERR     = 3'd5,
    FLUSH   = 3'd6
  } state_t;

  state_t state_r;

  // -------------------------------------------------------------------------
  // Latched request
  // -------------------------------------------------------------------------
  logic [TAG_BITS-1:0]   lk_tag_r;
  logic [INDEX_BITS-1:0] lk_idx_r;
  logic [2:0]            lk_instr_r;
  logic [PA_WIDTH-1:0]   lk_line_addr_r;

  // -------------------------------------------------------------------------
  // Hit detection (combinatorial on latched address + SRAM registered output)
  // -------------------------------------------------------------------------
  logic [WAYS-1:0]            hit_vec_s;
  logic                       hit_s;
  logic [$clog2(WAYS)-1:0]    hit_way_s;

  always_comb begin
    hit_vec_s = '0;
    for (int i = 0; i < WAYS; i++)
      hit_vec_s[i] = valid_rdata_s[i] & (tag_rdata_s[i] == lk_tag_r);
  end
  assign hit_s = |hit_vec_s;
  always_comb begin
    hit_way_s = '0;
    for (int i = WAYS-1; i >= 0; i--)
      if (hit_vec_s[i]) hit_way_s = i[$clog2(WAYS)-1:0];
  end

  // -------------------------------------------------------------------------
  // Victim selection: prefer invalid way; fall back to PLRU
  // -------------------------------------------------------------------------
  logic [WAYS-1:0]            inv_vec_s;
  logic                       has_inv_s;
  logic [$clog2(WAYS)-1:0]    inv_way_s;
  logic [$clog2(WAYS)-1:0]    plru_victim_s;
  logic [$clog2(WAYS)-1:0]    fill_way_s;
  logic [2:0]                 plru_r [0:SETS-1];

  always_comb begin
    inv_vec_s = '0;
    for (int i = 0; i < WAYS; i++)
      inv_vec_s[i] = ~valid_rdata_s[i];
  end
  assign has_inv_s = |inv_vec_s;
  always_comb begin
    inv_way_s = '0;
    for (int i = WAYS-1; i >= 0; i--)
      if (inv_vec_s[i]) inv_way_s = i[$clog2(WAYS)-1:0];
  end

  always_comb begin
    automatic logic [2:0] p = plru_r[lk_idx_r];
    if (!p[0])
      plru_victim_s = p[1] ? 2'd0 : 2'd1;
    else
      plru_victim_s = p[2] ? 2'd2 : 2'd3;
  end
  assign fill_way_s = has_inv_s ? inv_way_s : plru_victim_s;

  // -------------------------------------------------------------------------
  // Fill / flush registers
  // -------------------------------------------------------------------------
  logic [$clog2(WAYS)-1:0] fill_way_r;
  logic [BEAT_BITS-1:0]    beat_r;
  logic [LINE_BITS-1:0]    fill_buf_r;
  logic [INDEX_BITS-1:0]   flush_cnt_r;

  // AXI4 AR
  logic                    ar_valid_r;
  logic [PA_WIDTH-1:0]     ar_addr_r;

  // CPU outputs
  logic [31:0]  rdata_r;
  logic         rvalid_r, stall_r, flush_done_r, err_r;

  // -------------------------------------------------------------------------
  // Fill line assembly
  // -------------------------------------------------------------------------
  always_comb begin
    fill_line_s = fill_buf_r;
    fill_line_s[{beat_r, 6'd0} +: AXI_DW] = axi_if.rdata;
  end

  // -------------------------------------------------------------------------
  // SRAM write-port combinatorial decode
  // -------------------------------------------------------------------------
  always_comb begin
    data_wr_en_s   = 1'b0;
    data_wr_way_s  = fill_way_r;
    data_wr_addr_s = lk_idx_r;
    data_wr_data_s = fill_line_s;

    tag_wr_en_s   = 1'b0;
    tag_wr_way_s  = fill_way_r;
    tag_wr_addr_s = lk_idx_r;
    tag_wr_data_s = lk_tag_r;

    valid_wr_en_s   = 1'b0;
    valid_wr_way_s  = fill_way_r;
    valid_wr_addr_s = lk_idx_r;

    valid_flush_en_s   = 1'b0;
    valid_flush_addr_s = flush_cnt_r;

    if (state_r == FILL && axi_if.rvalid && axi_if.rlast && !(|axi_if.rresp)) begin
      data_wr_en_s  = 1'b1;
      tag_wr_en_s   = 1'b1;
      valid_wr_en_s = 1'b1;
    end

    if (state_r == FLUSH)
      valid_flush_en_s = 1'b1;
  end

  // -------------------------------------------------------------------------
  // Port assignments
  // -------------------------------------------------------------------------
  assign cpu_if.ic_rdata      = rdata_r;
  assign cpu_if.ic_rvalid     = rvalid_r;
  assign cpu_if.ic_stall      = stall_r;
  assign cpu_if.ic_flush_done = flush_done_r;
  assign cpu_if.ic_err        = err_r;

  assign axi_if.arvalid = ar_valid_r;
  assign axi_if.araddr  = PA_WIDTH'(ar_addr_r);
  assign axi_if.arid    = 4'h1;
  assign axi_if.arlen   = 8'h03;
  assign axi_if.arsize  = 3'b011;
  assign axi_if.arburst = 2'b01;
  assign axi_if.rready  = (state_r == FILL);

  assign axi_if.awid    = '0; assign axi_if.awaddr  = '0;
  assign axi_if.awlen   = '0; assign axi_if.awsize  = '0;
  assign axi_if.awburst = '0; assign axi_if.awvalid = '0;
  assign axi_if.wdata   = '0; assign axi_if.wstrb   = '0;
  assign axi_if.wlast   = '0; assign axi_if.wvalid  = '0;
  assign axi_if.bready  = '1;

  // -------------------------------------------------------------------------
  // PLRU update
  // -------------------------------------------------------------------------
  function automatic logic [2:0] plru_upd(
    input logic [2:0]              p,
    input logic [$clog2(WAYS)-1:0] w
  );
    logic [2:0] q = p;
    case (w)
      2'd0: begin q[0] = 1'b1; q[1] = 1'b1; end
      2'd1: begin q[0] = 1'b1; q[1] = 1'b0; end
      2'd2: begin q[0] = 1'b0; q[2] = 1'b1; end
      2'd3: begin q[0] = 1'b0; q[2] = 1'b0; end
      default: ;
    endcase
    return q;
  endfunction

  // -------------------------------------------------------------------------
  // FSM
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r      <= IDLE;
      ar_valid_r   <= '0;
      rvalid_r     <= '0;
      stall_r      <= '0;
      flush_done_r <= '0;
      err_r        <= '0;
      beat_r       <= '0;
      for (int s = 0; s < SETS; s++)
        plru_r[s] <= '0;
    end else begin
      rvalid_r     <= '0;
      flush_done_r <= '0;
      err_r        <= '0;

      case (state_r)

        IDLE: begin
          stall_r <= '0;
          if (cpu_if.ic_flush) begin
            stall_r     <= '1;
            flush_cnt_r <= '0;
            state_r     <= FLUSH;
          end else if (cpu_if.ic_req) begin
            lk_tag_r       <= req_tag_s;
            lk_idx_r       <= req_idx_s;
            lk_instr_r     <= req_instr_s;
            lk_line_addr_r <= {cpu_if.ic_addr[PA_WIDTH-1:OFFSET_BITS],
                               {OFFSET_BITS{1'b0}}};
            stall_r <= '1;
            state_r <= LOOKUP;
            // SRAM read address = req_idx_s (live); registered output available in LOOKUP.
          end
        end

        LOOKUP: begin
          // tag_rdata_s and data_rdata_s hold registered output from IDLE-phase read.
          // valid_rdata_s is combinatorial from lk_idx_r (via valid_reg rd_addr = lk_idx_r).
          if (hit_s) begin
            rdata_r  <= data_rdata_s[hit_way_s][{lk_instr_r, 5'd0} +: 32];
            rvalid_r <= '1;
            stall_r  <= '0;
            plru_r[lk_idx_r] <= plru_upd(plru_r[lk_idx_r], hit_way_s);
            state_r  <= IDLE;
          end else begin
            fill_way_r <= fill_way_s;
            ar_addr_r  <= lk_line_addr_r;
            ar_valid_r <= '1;
            beat_r     <= '0;
            fill_buf_r <= '0;
            state_r    <= MISS;
          end
        end

        MISS: begin
          if (axi_if.arready) begin
            ar_valid_r <= '0;
            state_r    <= FILL;
          end
        end

        FILL: begin
          // data/tag/valid SRAM writes driven combinatorially by write-port decode above.
          if (axi_if.rvalid) begin
            if (|axi_if.rresp) begin
              err_r   <= '1;
              stall_r <= '0;
              state_r <= ERR;
            end else if (axi_if.rlast) begin
              plru_r[lk_idx_r] <= plru_upd(plru_r[lk_idx_r], fill_way_r);
              rdata_r <= fill_line_s[{lk_instr_r, 5'd0} +: 32];
              state_r <= RESPOND;
            end else begin
              fill_buf_r <= fill_line_s;
              beat_r     <= beat_r + 1;
            end
          end
        end

        RESPOND: begin
          rvalid_r <= '1;
          stall_r  <= '0;
          state_r  <= IDLE;
        end

        ERR: begin
          state_r <= IDLE;
        end

        // valid_flush_en_s drives the all-ways clear via combinatorial decode above.
        FLUSH: begin
          plru_r[flush_cnt_r] <= '0;
          if (&flush_cnt_r) begin
            flush_done_r <= '1;
            stall_r      <= '0;
            state_r      <= IDLE;
          end else begin
            flush_cnt_r <= flush_cnt_r + 1;
          end
        end

        default: state_r <= IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // SRAM sub-module instances
  // -------------------------------------------------------------------------
  // Read address: req_idx_s is the live CPU address. Presented in IDLE so the
  // registered output is available exactly in LOOKUP (one cycle later).
  // The valid_reg rd_addr is lk_idx_r (combinatorial, no latency needed).

  as_icache_data_ram #(
    .SETS     (SETS),
    .WAYS     (WAYS),
    .LINE_BITS(LINE_BITS)
  ) data_ram (
    .clk_i    (clk_i),
    .rd_addr_i(req_idx_s),
    .rd_data_o(data_rdata_s),
    .wr_en_i  (data_wr_en_s),
    .wr_way_i (data_wr_way_s),
    .wr_addr_i(data_wr_addr_s),
    .wr_data_i(data_wr_data_s)
  );

  as_icache_tag_ram #(
    .SETS    (SETS),
    .WAYS    (WAYS),
    .TAG_BITS(TAG_BITS)
  ) tag_ram (
    .clk_i    (clk_i),
    .rd_addr_i(req_idx_s),
    .rd_data_o(tag_rdata_s),
    .wr_en_i  (tag_wr_en_s),
    .wr_way_i (tag_wr_way_s),
    .wr_addr_i(tag_wr_addr_s),
    .wr_data_i(tag_wr_data_s)
  );

  as_icache_valid_reg #(
    .SETS(SETS),
    .WAYS(WAYS)
  ) valid_reg (
    .clk_i      (clk_i),
    .rst_i      (rst_i),
    .rd_addr_i  (lk_idx_r),
    .rd_data_o  (valid_rdata_s),
    .wr_en_i    (valid_wr_en_s),
    .wr_way_i   (valid_wr_way_s),
    .wr_addr_i  (valid_wr_addr_s),
    .wr_data_i  (1'b1),
    .flush_en_i  (valid_flush_en_s),
    .flush_addr_i(valid_flush_addr_s)
  );

endmodule : asICache
