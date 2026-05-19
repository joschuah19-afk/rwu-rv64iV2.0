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
// Read timing: address driven in IDLE_ST → registered output valid in LOOKUP_ST.

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
  // 1. Derived parameters
  // -------------------------------------------------------------------------
  localparam int SETS        = CACHE_SIZE_B / (WAYS * LINE_BYTES);
  localparam int OFFSET_BITS = $clog2(LINE_BYTES);
  localparam int INDEX_BITS  = $clog2(SETS);
  localparam int TAG_BITS    = PA_WIDTH - INDEX_BITS - OFFSET_BITS;
  localparam int BEATS       = LINE_BYTES / (AXI_DW / 8);
  localparam int BEAT_BITS   = $clog2(BEATS);
  localparam int LINE_BITS   = LINE_BYTES * 8;

  // -------------------------------------------------------------------------
  // 2. ALL signal declarations
  // -------------------------------------------------------------------------

  // FSM state type
  typedef enum logic [2:0] {
    IDLE_ST    = 3'd0,
    LOOKUP_ST  = 3'd1,
    MISS_ST    = 3'd2,
    FILL_ST    = 3'd3,
    RESPOND_ST = 3'd4,
    ERR_ST     = 3'd5,
    FLUSH_ST   = 3'd6
  } state_t;

  state_t ic_state_s;
  state_t ic_nextstate_s;

  // Address decomposition (combinatorial, from live CPU request)
  logic [TAG_BITS-1:0]   req_tag_s;
  logic [INDEX_BITS-1:0] req_idx_s;
  logic [2:0]            req_instr_s;

  // SRAM / register-file output wires
  logic [WAYS-1:0][LINE_BITS-1:0] data_rdata_s;  // registered, all WAYS
  logic [WAYS-1:0][TAG_BITS-1:0]  tag_rdata_s;   // registered, all WAYS
  logic [WAYS-1:0]                valid_rdata_s;  // combinatorial, all WAYS

  // SRAM write-port control (combinatorial decode)
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

  // Fill line assembly
  logic [LINE_BITS-1:0] fill_line_s;

  // Latched request
  logic [TAG_BITS-1:0]   lk_tag_r;
  logic [INDEX_BITS-1:0] lk_idx_r;
  logic [2:0]            lk_instr_r;
  logic [PA_WIDTH-1:0]   lk_line_addr_r;

  // Hit detection
  logic [WAYS-1:0]            hit_vec_s;
  logic                       hit_s;
  logic [$clog2(WAYS)-1:0]    hit_way_s;

  // Victim selection
  logic [WAYS-1:0]            inv_vec_s;
  logic                       has_inv_s;
  logic [$clog2(WAYS)-1:0]    inv_way_s;
  logic [$clog2(WAYS)-1:0]    plru_victim_s;
  logic [$clog2(WAYS)-1:0]    fill_way_s;
  logic [2:0]                 plru_r [0:SETS-1];

  // Fill / flush registers
  logic [$clog2(WAYS)-1:0] fill_way_r;
  logic [BEAT_BITS-1:0]    beat_r;
  logic [LINE_BITS-1:0]    fill_buf_r;
  logic [INDEX_BITS-1:0]   flush_cnt_r;

  // AXI4 AR
  logic                    ar_valid_r;
  logic [PA_WIDTH-1:0]     ar_addr_r;

  // CPU output registers
  logic [31:0]  rdata_r;
  logic         rvalid_r, stall_r, flush_done_r, err_r;

  // -------------------------------------------------------------------------
  // 3. assign statements
  // -------------------------------------------------------------------------
  assign req_tag_s   = cpu_if.ic_addr[PA_WIDTH-1 : INDEX_BITS+OFFSET_BITS];
  assign req_idx_s   = cpu_if.ic_addr[INDEX_BITS+OFFSET_BITS-1 : OFFSET_BITS];
  assign req_instr_s = cpu_if.ic_addr[OFFSET_BITS-1 : 2];

  assign hit_s      = |hit_vec_s;
  assign has_inv_s  = |inv_vec_s;
  assign fill_way_s = has_inv_s ? inv_way_s : plru_victim_s;

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
  assign axi_if.rready  = (ic_state_s == FILL_ST);

  assign axi_if.awid    = '0; assign axi_if.awaddr  = '0;
  assign axi_if.awlen   = '0; assign axi_if.awsize  = '0;
  assign axi_if.awburst = '0; assign axi_if.awvalid = '0;
  assign axi_if.wdata   = '0; assign axi_if.wstrb   = '0;
  assign axi_if.wlast   = '0; assign axi_if.wvalid  = '0;
  assign axi_if.bready  = '1;

  // -------------------------------------------------------------------------
  // 4. always_comb blocks
  // -------------------------------------------------------------------------

  // Hit detection
  always_comb begin
    hit_vec_s = '0;
    for (int i = 0; i < WAYS; i++)
      hit_vec_s[i] = valid_rdata_s[i] & (tag_rdata_s[i] == lk_tag_r);
  end

  always_comb begin
    hit_way_s = '0;
    for (int i = WAYS-1; i >= 0; i--)
      if (hit_vec_s[i]) hit_way_s = i[$clog2(WAYS)-1:0];
  end

  // Victim selection
  always_comb begin
    inv_vec_s = '0;
    for (int i = 0; i < WAYS; i++)
      inv_vec_s[i] = ~valid_rdata_s[i];
  end

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

  // Fill line assembly
  always_comb begin
    fill_line_s = fill_buf_r;
    fill_line_s[{beat_r, 6'd0} +: AXI_DW] = axi_if.rdata;
  end

  // SRAM write-port combinatorial decode
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

    if (ic_state_s == FILL_ST && axi_if.rvalid && axi_if.rlast && !(|axi_if.rresp)) begin
      data_wr_en_s  = 1'b1;
      tag_wr_en_s   = 1'b1;
      valid_wr_en_s = 1'b1;
    end

    if (ic_state_s == FLUSH_ST)
      valid_flush_en_s = 1'b1;
  end

  // FSM block: input logic
  always_comb
  begin
    ic_nextstate_s = ic_state_s;
    case (ic_state_s)
      IDLE_ST:
        if      (cpu_if.ic_flush) ic_nextstate_s = FLUSH_ST;
        else if (cpu_if.ic_req)   ic_nextstate_s = LOOKUP_ST;
      LOOKUP_ST:
        if (hit_s) ic_nextstate_s = IDLE_ST;
        else       ic_nextstate_s = MISS_ST;
      MISS_ST:
        if (axi_if.arready) ic_nextstate_s = FILL_ST;
      FILL_ST:
        if (axi_if.rvalid) begin
          if (|axi_if.rresp)     ic_nextstate_s = ERR_ST;
          else if (axi_if.rlast) ic_nextstate_s = RESPOND_ST;
        end
      RESPOND_ST:  ic_nextstate_s = IDLE_ST;
      ERR_ST:      ic_nextstate_s = IDLE_ST;
      FLUSH_ST:
        if (&flush_cnt_r) ic_nextstate_s = IDLE_ST;
      default:     ic_nextstate_s = IDLE_ST;
    endcase
  end

  // -------------------------------------------------------------------------
  // 5. always_ff blocks
  // -------------------------------------------------------------------------

  // FSM block: delay
  always_ff @(posedge clk_i)
  begin
    if (rst_i)
      ic_state_s <= IDLE_ST;
    else
      ic_state_s <= ic_nextstate_s;
  end

  // FSM block: output logic
  always_ff @(posedge clk_i)
  begin
    if (rst_i) begin
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

      case (ic_state_s)

        IDLE_ST: begin
          stall_r <= '0;
          if (cpu_if.ic_flush) begin
            stall_r     <= '1;
            flush_cnt_r <= '0;
          end else if (cpu_if.ic_req) begin
            lk_tag_r       <= req_tag_s;
            lk_idx_r       <= req_idx_s;
            lk_instr_r     <= req_instr_s;
            lk_line_addr_r <= {cpu_if.ic_addr[PA_WIDTH-1:OFFSET_BITS],
                               {OFFSET_BITS{1'b0}}};
            stall_r <= '1;
            // SRAM read port driven by req_idx_s (live); registered output valid in LOOKUP_ST.
          end
        end

        LOOKUP_ST: begin
          if (hit_s) begin
            rdata_r  <= data_rdata_s[hit_way_s][{lk_instr_r, 5'd0} +: 32];
            rvalid_r <= '1;
            stall_r  <= '0;
            plru_r[lk_idx_r] <= plru_upd(plru_r[lk_idx_r], hit_way_s);
          end else begin
            fill_way_r <= fill_way_s;
            ar_addr_r  <= lk_line_addr_r;
            ar_valid_r <= '1;
            beat_r     <= '0;
            fill_buf_r <= '0;
          end
        end

        MISS_ST: begin
          if (axi_if.arready)
            ar_valid_r <= '0;
        end

        FILL_ST: begin
          if (axi_if.rvalid) begin
            if (|axi_if.rresp) begin
              err_r   <= '1;
              stall_r <= '0;
            end else if (axi_if.rlast) begin
              plru_r[lk_idx_r] <= plru_upd(plru_r[lk_idx_r], fill_way_r);
              rdata_r <= fill_line_s[{lk_instr_r, 5'd0} +: 32];
            end else begin
              fill_buf_r <= fill_line_s;
              beat_r     <= beat_r + 1;
            end
          end
        end

        RESPOND_ST: begin
          rvalid_r <= '1;
          stall_r  <= '0;
        end

        ERR_ST: ;

        FLUSH_ST: begin
          plru_r[flush_cnt_r] <= '0;
          if (&flush_cnt_r) begin
            flush_done_r <= '1;
            stall_r      <= '0;
          end else begin
            flush_cnt_r <= flush_cnt_r + 1;
          end
        end

        default: ;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // PLRU update helper
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
  // 6. Module instantiations
  // -------------------------------------------------------------------------
  // valid_reg rd_addr is lk_idx_r (combinatorial, no SRAM read latency needed).

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
    .clk_i       (clk_i),
    .rst_i       (rst_i),
    .rd_addr_i   (lk_idx_r),
    .rd_data_o   (valid_rdata_s),
    .wr_en_i     (valid_wr_en_s),
    .wr_way_i    (valid_wr_way_s),
    .wr_addr_i   (valid_wr_addr_s),
    .wr_data_i   (1'b1),
    .flush_en_i  (valid_flush_en_s),
    .flush_addr_i(valid_flush_addr_s)
  );

endmodule : asICache
