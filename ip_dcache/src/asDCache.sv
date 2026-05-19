`timescale 1ns/1ps
import as_pack::*;

// 4-way set-associative data cache (write-back, write-allocate).
//
// Address decomposition (PA=32, 4 KB, 4-way, 32 B line):
//   [31:10] = tag  (22 bits)
//   [9:5]   = set index (5 bits)
//   [4:3]   = doubleword select within line (2 bits)
//   [2:0]   = byte offset within doubleword (3 bits)
//
// AXI4 master: ARID/AWID=4'h2, ARLEN/AWLEN=3, ARSIZE/AWSIZE=3, BURST=INCR
//
// SRAM sub-modules:
//   as_dcache_data_ram  – data store (WAYS × LINE_BITS, registered output)
//   as_dcache_tag_ram   – tag store  (WAYS × TAG_BITS,  registered output)
//   as_dcache_valid_reg – valid+dirty flags (WAYS × 2, combinatorial, FF-based)
// For ASIC: replace the two *_ram modules with X-Fab SRAM wrappers.
//
// Read timing: SRAM address driven in IDLE_ST → registered output valid in LOOKUP_ST.
// Flush eviction: FLUSH_ST drives SRAM read address (flush_idx_r) → registered
//   output available in FLUSH_EVICT_PREP_ST → proceeds to EVICT_AW_ST. This adds one
//   cycle per dirty line eviction during flush versus the original combinatorial model.

module asDCache #(
  parameter int CACHE_SIZE_B = 4096,
  parameter int WAYS         = 4,
  parameter int LINE_BYTES   = 32,
  parameter int PA_WIDTH     = 32,
  parameter int AXI_DW       = 64
)(
  input  logic       clk_i,
  input  logic       rst_i,
  as_dcache_if.cache cpu_if,
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
  localparam int DW_SEL_BITS = OFFSET_BITS - 3;

  // -------------------------------------------------------------------------
  // 2. ALL signal declarations
  // -------------------------------------------------------------------------

  // FSM state type
  typedef enum logic [3:0] {
    IDLE_ST             = 4'd0,
    LOOKUP_ST           = 4'd1,
    EVICT_AW_ST         = 4'd2,
    EVICT_W_ST          = 4'd3,
    EVICT_B_ST          = 4'd4,
    MISS_ST             = 4'd5,
    FILL_ST             = 4'd6,
    FILL_RESP_ST        = 4'd7,
    FLUSH_ST            = 4'd8,
    FLUSH_EVICT_PREP_ST = 4'd9,
    ERR_ST              = 4'd10
  } state_t;

  state_t dc_state_s;
  state_t dc_nextstate_s;

  // Address decomposition (combinatorial, live CPU signals)
  logic [TAG_BITS-1:0]   req_tag_s;
  logic [INDEX_BITS-1:0] req_idx_s;

  // Latched request
  logic [TAG_BITS-1:0]    lk_tag_r;
  logic [INDEX_BITS-1:0]  lk_idx_r;
  logic [PA_WIDTH-1:0]    lk_line_addr_r;
  logic [2:0]             lk_size_r;
  logic [2:0]             lk_byte_off_r;
  logic [DW_SEL_BITS-1:0] lk_dw_sel_r;
  logic                   lk_wr_r;
  logic [63:0]            lk_wdata_r;
  logic [7:0]             lk_wstrb_r;

  // SRAM / register-file output wires
  logic [WAYS-1:0][LINE_BITS-1:0] data_rdata_s;
  logic [WAYS-1:0][TAG_BITS-1:0]  tag_rdata_s;
  logic [WAYS-1:0]                 vd_valid_s;
  logic [WAYS-1:0]                 vd_dirty_s;

  // SRAM read address mux signals
  logic [INDEX_BITS-1:0] data_rd_addr_s;
  logic [INDEX_BITS-1:0] tag_rd_addr_s;
  logic [INDEX_BITS-1:0] vd_rd_addr_s;

  // PLRU
  logic [2:0] plru_r [0:SETS-1];

  // Hit detection
  logic [WAYS-1:0]         hit_vec_s;
  logic                    hit_s;
  logic [$clog2(WAYS)-1:0] hit_way_s;

  // Victim selection
  logic [WAYS-1:0]         inv_vec_s;
  logic                    has_inv_s;
  logic [$clog2(WAYS)-1:0] inv_way_s;
  logic [$clog2(WAYS)-1:0] plru_victim_s;
  logic [$clog2(WAYS)-1:0] fill_way_s;

  // Fill / evict / flush registers
  logic [$clog2(WAYS)-1:0] fill_way_r;
  logic [BEAT_BITS-1:0]    beat_r;
  logic [LINE_BITS-1:0]    fill_buf_r;
  logic [LINE_BITS-1:0]    evict_line_r;
  logic [INDEX_BITS-1:0]   flush_idx_r;
  logic [$clog2(WAYS)-1:0] flush_way_r;
  logic                    flush_mode_r;

  // AXI4 AR / AW
  logic              ar_valid_r;
  logic [PA_WIDTH-1:0] ar_addr_r;
  logic              aw_valid_r;
  logic [PA_WIDTH-1:0] aw_addr_r;

  // CPU output registers
  logic [63:0] rdata_r;
  logic        rvalid_r, stall_r, flush_done_r, err_r;

  // Fill line assembly
  logic [LINE_BITS-1:0] fill_line_s;

  // SRAM write-port control
  logic                    data_wr_en_s;
  logic [$clog2(WAYS)-1:0] data_wr_way_s;
  logic [INDEX_BITS-1:0]   data_wr_addr_s;
  logic [LINE_BITS-1:0]    data_wr_data_s;

  logic                    tag_wr_en_s;
  logic [$clog2(WAYS)-1:0] tag_wr_way_s;
  logic [INDEX_BITS-1:0]   tag_wr_addr_s;
  logic [TAG_BITS-1:0]     tag_wr_data_s;

  logic                    vd_wr_en_s;
  logic [$clog2(WAYS)-1:0] vd_wr_way_s;
  logic [INDEX_BITS-1:0]   vd_wr_addr_s;
  logic                    vd_wr_valid_s;
  logic                    vd_wr_dirty_s;

  logic [LINE_BITS-1:0] fill_wr_data_s;

  // -------------------------------------------------------------------------
  // 3. assign statements
  // -------------------------------------------------------------------------
  assign req_tag_s  = cpu_if.dc_addr[PA_WIDTH-1 : INDEX_BITS+OFFSET_BITS];
  assign req_idx_s  = cpu_if.dc_addr[INDEX_BITS+OFFSET_BITS-1 : OFFSET_BITS];

  assign hit_s      = |hit_vec_s;
  assign has_inv_s  = |inv_vec_s;
  assign fill_way_s = has_inv_s ? inv_way_s : plru_victim_s;

  assign cpu_if.dc_rdata      = rdata_r;
  assign cpu_if.dc_rvalid     = rvalid_r;
  assign cpu_if.dc_stall      = stall_r;
  assign cpu_if.dc_flush_done = flush_done_r;
  assign cpu_if.dc_err        = err_r;

  assign axi_if.arvalid = ar_valid_r;
  assign axi_if.araddr  = PA_WIDTH'(ar_addr_r);
  assign axi_if.arid    = 4'h2;
  assign axi_if.arlen   = 8'h03;
  assign axi_if.arsize  = 3'b011;
  assign axi_if.arburst = 2'b01;
  assign axi_if.rready  = (dc_state_s == FILL_ST);

  assign axi_if.awvalid = aw_valid_r;
  assign axi_if.awaddr  = PA_WIDTH'(aw_addr_r);
  assign axi_if.awid    = 4'h2;
  assign axi_if.awlen   = 8'h03;
  assign axi_if.awsize  = 3'b011;
  assign axi_if.awburst = 2'b01;

  assign axi_if.wvalid = (dc_state_s == EVICT_W_ST);
  assign axi_if.wdata  = evict_line_r[{beat_r, 6'd0} +: AXI_DW];
  assign axi_if.wstrb  = 8'hFF;
  assign axi_if.wlast  = (beat_r == BEAT_BITS'(BEATS - 1));

  assign axi_if.bready = (dc_state_s == EVICT_B_ST);

  // -------------------------------------------------------------------------
  // 4. always_comb blocks
  // -------------------------------------------------------------------------

  // Hit detection
  always_comb begin
    hit_vec_s = '0;
    for (int i = 0; i < WAYS; i++)
      hit_vec_s[i] = vd_valid_s[i] & (tag_rdata_s[i] == lk_tag_r);
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
      inv_vec_s[i] = ~vd_valid_s[i];
  end

  always_comb begin
    inv_way_s = '0;
    for (int i = WAYS-1; i >= 0; i--)
      if (inv_vec_s[i]) inv_way_s = i[$clog2(WAYS)-1:0];
  end

  always_comb begin
    automatic logic [2:0] p = plru_r[lk_idx_r];
    if (!p[0])
      plru_victim_s = p[1] ? 2'd1 : 2'd0;
    else
      plru_victim_s = p[2] ? 2'd3 : 2'd2;
  end

  // Fill line assembly
  always_comb begin
    fill_line_s = fill_buf_r;
    fill_line_s[{beat_r, 6'd0} +: AXI_DW] = axi_if.rdata;
  end

  // SRAM read address mux
  always_comb begin
    // Data/tag SRAMs: flush path drives flush_idx_r one cycle before FLUSH_EVICT_PREP_ST.
    if (dc_state_s == FLUSH_ST)
      data_rd_addr_s = flush_idx_r;
    else
      data_rd_addr_s = req_idx_s;
    tag_rd_addr_s = data_rd_addr_s;

    // valid_reg: combinatorial output, so drive flush_idx_r during all flush states.
    if (dc_state_s == FLUSH_ST ||
        dc_state_s == FLUSH_EVICT_PREP_ST ||
        dc_state_s == EVICT_B_ST)
      vd_rd_addr_s = flush_idx_r;
    else
      vd_rd_addr_s = lk_idx_r;
  end

  // SRAM / vd_reg write-port combinatorial decode
  always_comb begin
    data_wr_en_s   = 1'b0;
    data_wr_way_s  = fill_way_r;
    data_wr_addr_s = lk_idx_r;
    data_wr_data_s = '0;

    tag_wr_en_s   = 1'b0;
    tag_wr_way_s  = fill_way_r;
    tag_wr_addr_s = lk_idx_r;
    tag_wr_data_s = lk_tag_r;

    vd_wr_en_s    = 1'b0;
    vd_wr_way_s   = fill_way_r;
    vd_wr_addr_s  = lk_idx_r;
    vd_wr_valid_s = 1'b0;
    vd_wr_dirty_s = 1'b0;

    fill_wr_data_s = lk_wr_r
      ? merge_write(fill_line_s, lk_dw_sel_r, lk_wdata_r, lk_wstrb_r)
      : fill_line_s;

    // FILL completion: write data+tag+valid to SRAM
    if (dc_state_s == FILL_ST && axi_if.rvalid && axi_if.rlast && !(|axi_if.rresp)) begin
      data_wr_en_s   = 1'b1;
      data_wr_data_s = fill_wr_data_s;
      tag_wr_en_s    = 1'b1;
      vd_wr_en_s     = 1'b1;
      vd_wr_valid_s  = 1'b1;
      vd_wr_dirty_s  = lk_wr_r;
    end

    // LOOKUP write hit: merge + mark dirty
    if (dc_state_s == LOOKUP_ST && hit_s && lk_wr_r) begin
      data_wr_en_s   = 1'b1;
      data_wr_way_s  = hit_way_s;
      data_wr_data_s = merge_write(data_rdata_s[hit_way_s],
                                   lk_dw_sel_r, lk_wdata_r, lk_wstrb_r);
      vd_wr_en_s    = 1'b1;
      vd_wr_way_s   = hit_way_s;
      vd_wr_valid_s = 1'b1;
      vd_wr_dirty_s = 1'b1;
    end

    // Invalidate: FLUSH clean/invalid line, or EVICT_B completing flush write-back
    if ((dc_state_s == FLUSH_ST &&
         !(vd_valid_s[flush_way_r] && vd_dirty_s[flush_way_r])) ||
        (dc_state_s == EVICT_B_ST &&
         axi_if.bvalid && !(|axi_if.bresp) && flush_mode_r)) begin
      vd_wr_en_s    = 1'b1;
      vd_wr_way_s   = flush_way_r;
      vd_wr_addr_s  = flush_idx_r;
      vd_wr_valid_s = 1'b0;
      vd_wr_dirty_s = 1'b0;
    end
  end

  // FSM block: input logic
  always_comb begin
    dc_nextstate_s = dc_state_s;
    case (dc_state_s)
      IDLE_ST:
        if      (cpu_if.dc_flush) dc_nextstate_s = FLUSH_ST;
        else if (cpu_if.dc_req)   dc_nextstate_s = LOOKUP_ST;
      LOOKUP_ST:
        if (hit_s)                                            dc_nextstate_s = IDLE_ST;
        else if (vd_valid_s[fill_way_s] && vd_dirty_s[fill_way_s]) dc_nextstate_s = EVICT_AW_ST;
        else                                                  dc_nextstate_s = MISS_ST;
      EVICT_AW_ST:
        if (axi_if.awready) dc_nextstate_s = EVICT_W_ST;
      EVICT_W_ST:
        if (axi_if.wready && beat_r == BEAT_BITS'(BEATS - 1))
          dc_nextstate_s = EVICT_B_ST;
      EVICT_B_ST:
        if (axi_if.bvalid) begin
          if (|axi_if.bresp)
            dc_nextstate_s = ERR_ST;
          else if (flush_mode_r)
            dc_nextstate_s =
              (flush_way_r == $clog2(WAYS)'(WAYS-1) && (&flush_idx_r))
              ? IDLE_ST : FLUSH_ST;
          else
            dc_nextstate_s = MISS_ST;
        end
      MISS_ST:
        if (axi_if.arready) dc_nextstate_s = FILL_ST;
      FILL_ST:
        if (axi_if.rvalid) begin
          if (|axi_if.rresp)     dc_nextstate_s = ERR_ST;
          else if (axi_if.rlast) dc_nextstate_s = FILL_RESP_ST;
        end
      FILL_RESP_ST:        dc_nextstate_s = IDLE_ST;
      ERR_ST:              dc_nextstate_s = IDLE_ST;
      FLUSH_ST:
        if (vd_valid_s[flush_way_r] && vd_dirty_s[flush_way_r])
          dc_nextstate_s = FLUSH_EVICT_PREP_ST;
        else if (flush_way_r == $clog2(WAYS)'(WAYS-1) && (&flush_idx_r))
          dc_nextstate_s = IDLE_ST;
      FLUSH_EVICT_PREP_ST: dc_nextstate_s = EVICT_AW_ST;
      default:             dc_nextstate_s = IDLE_ST;
    endcase
  end

  // -------------------------------------------------------------------------
  // 5. always_ff blocks
  // -------------------------------------------------------------------------

  // FSM block: delay
  always_ff @(posedge clk_i) begin
    if (rst_i)
      dc_state_s <= IDLE_ST;
    else
      dc_state_s <= dc_nextstate_s;
  end

  // FSM block: output logic
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      ar_valid_r   <= '0;
      aw_valid_r   <= '0;
      rvalid_r     <= '0;
      stall_r      <= '0;
      flush_done_r <= '0;
      err_r        <= '0;
      beat_r       <= '0;
      flush_mode_r <= '0;
      for (int s = 0; s < SETS; s++)
        plru_r[s] <= '0;
    end else begin
      rvalid_r     <= '0;
      flush_done_r <= '0;
      err_r        <= '0;

      case (dc_state_s)

        IDLE_ST: begin
          stall_r <= '0;
          if (cpu_if.dc_flush) begin
            stall_r     <= '1;
            flush_idx_r <= '0;
            flush_way_r <= '0;
          end else if (cpu_if.dc_req) begin
            lk_tag_r       <= req_tag_s;
            lk_idx_r       <= req_idx_s;
            lk_line_addr_r <= {cpu_if.dc_addr[PA_WIDTH-1 : OFFSET_BITS],
                               {OFFSET_BITS{1'b0}}};
            lk_size_r     <= cpu_if.dc_size;
            lk_byte_off_r <= cpu_if.dc_addr[2:0];
            lk_dw_sel_r   <= cpu_if.dc_addr[OFFSET_BITS-1 : 3];
            lk_wr_r       <= cpu_if.dc_wr;
            lk_wdata_r    <= cpu_if.dc_wdata;
            lk_wstrb_r    <= cpu_if.dc_wstrb;
            stall_r       <= '1;
            // SRAM read address = req_idx_s (live); registered output available in LOOKUP_ST.
          end
        end

        LOOKUP_ST: begin
          if (hit_s) begin
            plru_r[lk_idx_r] <= plru_upd(plru_r[lk_idx_r], hit_way_s);
            if (!lk_wr_r) begin
              rdata_r  <= extract_load(data_rdata_s[hit_way_s],
                                       lk_dw_sel_r, lk_byte_off_r, lk_size_r);
              rvalid_r <= '1;
            end
            stall_r <= '0;
          end else begin
            fill_way_r <= fill_way_s;
            if (vd_valid_s[fill_way_s] && vd_dirty_s[fill_way_s]) begin
              evict_line_r <= data_rdata_s[fill_way_s];
              aw_addr_r    <= {tag_rdata_s[fill_way_s], lk_idx_r, {OFFSET_BITS{1'b0}}};
              aw_valid_r   <= '1;
              beat_r       <= '0;
              flush_mode_r <= '0;
            end else begin
              ar_addr_r  <= lk_line_addr_r;
              ar_valid_r <= '1;
              beat_r     <= '0;
              fill_buf_r <= '0;
            end
          end
        end

        EVICT_AW_ST: begin
          if (axi_if.awready) begin
            aw_valid_r <= '0;
            beat_r     <= '0;
          end
        end

        EVICT_W_ST: begin
          if (axi_if.wready && beat_r != BEAT_BITS'(BEATS - 1))
            beat_r <= beat_r + 1;
        end

        EVICT_B_ST: begin
          if (axi_if.bvalid) begin
            if (|axi_if.bresp) begin
              err_r   <= '1;
              stall_r <= '0;
            end else if (flush_mode_r) begin
              // vd invalidation driven combinatorially by write-port decode above.
              if (flush_way_r == $clog2(WAYS)'(WAYS-1)) begin
                flush_way_r <= '0;
                if (&flush_idx_r) begin
                  flush_done_r <= '1;
                  stall_r      <= '0;
                end else begin
                  flush_idx_r <= flush_idx_r + 1;
                end
              end else begin
                flush_way_r <= flush_way_r + 1;
              end
            end else begin
              ar_addr_r  <= lk_line_addr_r;
              ar_valid_r <= '1;
              beat_r     <= '0;
              fill_buf_r <= '0;
            end
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
              if (!lk_wr_r)
                rdata_r <= extract_load(fill_wr_data_s,
                                        lk_dw_sel_r, lk_byte_off_r, lk_size_r);
            end else begin
              fill_buf_r <= fill_line_s;
              beat_r     <= beat_r + 1;
            end
          end
        end

        FILL_RESP_ST: begin
          if (!lk_wr_r)
            rvalid_r <= '1;
          stall_r <= '0;
        end

        ERR_ST: ;

        FLUSH_ST: begin
          if (vd_valid_s[flush_way_r] && vd_dirty_s[flush_way_r]) begin
            flush_mode_r <= '1;
          end else begin
            // Clean/invalid: vd cleared by combinatorial write-port decode above.
            if (flush_way_r == $clog2(WAYS)'(WAYS-1)) begin
              flush_way_r <= '0;
              if (&flush_idx_r) begin
                flush_done_r <= '1;
                stall_r      <= '0;
              end else begin
                flush_idx_r <= flush_idx_r + 1;
              end
            end else begin
              flush_way_r <= flush_way_r + 1;
            end
          end
        end

        FLUSH_EVICT_PREP_ST: begin
          evict_line_r <= data_rdata_s[flush_way_r];
          aw_addr_r    <= {tag_rdata_s[flush_way_r], flush_idx_r, {OFFSET_BITS{1'b0}}};
          aw_valid_r   <= '1;
          beat_r       <= '0;
        end

        default: ;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Helper functions
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

  function automatic logic [63:0] extract_load(
    input logic [LINE_BITS-1:0]   line,
    input logic [DW_SEL_BITS-1:0] dw_sel,
    input logic [2:0]             byte_off,
    input logic [2:0]             size
  );
    logic [63:0] dw = line[{dw_sel, 6'd0} +: 64];
    logic [63:0] result;
    case (size)
      3'b000: case (byte_off)
                3'd0: result = {{56{dw[7]}},   dw[7:0]};
                3'd1: result = {{56{dw[15]}},  dw[15:8]};
                3'd2: result = {{56{dw[23]}},  dw[23:16]};
                3'd3: result = {{56{dw[31]}},  dw[31:24]};
                3'd4: result = {{56{dw[39]}},  dw[39:32]};
                3'd5: result = {{56{dw[47]}},  dw[47:40]};
                3'd6: result = {{56{dw[55]}},  dw[55:48]};
                default: result = {{56{dw[63]}}, dw[63:56]};
              endcase
      3'b001: case (byte_off[2:1])
                2'd0: result = {{48{dw[15]}}, dw[15:0]};
                2'd1: result = {{48{dw[31]}}, dw[31:16]};
                2'd2: result = {{48{dw[47]}}, dw[47:32]};
                default: result = {{48{dw[63]}}, dw[63:48]};
              endcase
      3'b010: result = byte_off[2] ? {{32{dw[63]}}, dw[63:32]}
                                   : {{32{dw[31]}}, dw[31:0]};
      3'b011: result = dw;
      3'b100: case (byte_off)
                3'd0: result = {56'b0, dw[7:0]};
                3'd1: result = {56'b0, dw[15:8]};
                3'd2: result = {56'b0, dw[23:16]};
                3'd3: result = {56'b0, dw[31:24]};
                3'd4: result = {56'b0, dw[39:32]};
                3'd5: result = {56'b0, dw[47:40]};
                3'd6: result = {56'b0, dw[55:48]};
                default: result = {56'b0, dw[63:56]};
              endcase
      3'b101: case (byte_off[2:1])
                2'd0: result = {48'b0, dw[15:0]};
                2'd1: result = {48'b0, dw[31:16]};
                2'd2: result = {48'b0, dw[47:32]};
                default: result = {48'b0, dw[63:48]};
              endcase
      3'b110: result = byte_off[2] ? {32'b0, dw[63:32]}
                                   : {32'b0, dw[31:0]};
      default: result = dw;
    endcase
    return result;
  endfunction

  function automatic logic [LINE_BITS-1:0] merge_write(
    input logic [LINE_BITS-1:0]   line,
    input logic [DW_SEL_BITS-1:0] dw_sel,
    input logic [63:0]            wdata,
    input logic [7:0]             wstrb
  );
    automatic logic [LINE_BITS-1:0] r = line;
    for (int b = 0; b < 8; b++)
      if (wstrb[b])
        r[{dw_sel, 6'd0} + b*8 +: 8] = wdata[b*8 +: 8];
    return r;
  endfunction

  // -------------------------------------------------------------------------
  // 6. Module instantiations
  // -------------------------------------------------------------------------
  as_dcache_data_ram #(
    .SETS     (SETS),
    .WAYS     (WAYS),
    .LINE_BITS(LINE_BITS)
  ) data_ram (
    .clk_i    (clk_i),
    .rd_addr_i(data_rd_addr_s),
    .rd_data_o(data_rdata_s),
    .wr_en_i  (data_wr_en_s),
    .wr_way_i (data_wr_way_s),
    .wr_addr_i(data_wr_addr_s),
    .wr_data_i(data_wr_data_s)
  );

  as_dcache_tag_ram #(
    .SETS    (SETS),
    .WAYS    (WAYS),
    .TAG_BITS(TAG_BITS)
  ) tag_ram (
    .clk_i    (clk_i),
    .rd_addr_i(tag_rd_addr_s),
    .rd_data_o(tag_rdata_s),
    .wr_en_i  (tag_wr_en_s),
    .wr_way_i (tag_wr_way_s),
    .wr_addr_i(tag_wr_addr_s),
    .wr_data_i(tag_wr_data_s)
  );

  as_dcache_valid_reg #(
    .SETS(SETS),
    .WAYS(WAYS)
  ) vd_reg (
    .clk_i     (clk_i),
    .rst_i     (rst_i),
    .rd_addr_i (vd_rd_addr_s),
    .rd_valid_o(vd_valid_s),
    .rd_dirty_o(vd_dirty_s),
    .wr_en_i   (vd_wr_en_s),
    .wr_way_i  (vd_wr_way_s),
    .wr_addr_i (vd_wr_addr_s),
    .wr_valid_i(vd_wr_valid_s),
    .wr_dirty_i(vd_wr_dirty_s)
  );

endmodule : asDCache
