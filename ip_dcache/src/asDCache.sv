`timescale 1ns/1ps
import as_pack::*;

// 4-way set-associative data cache (write-back, write-allocate).
// Behavioral SRAM models for simulation; replace with SPRAM macros for synthesis.
//
// Address decomposition (PA=32, 4 KB, 4-way, 32 B line):
//   [31:10] = tag  (22 bits)
//   [9:5]   = set index (5 bits)
//   [4:3]   = doubleword select within line (2 bits)
//   [2:0]   = byte offset within doubleword (3 bits)
//
// CPU interface uses dc_wstrb[7:0] / dc_size[2:0] (byte-select protocol,
// same encoding as RISC-V func3: 000=lb, 001=lh, 010=lw, 011=ld,
// 100=lbu, 101=lhu, 110=lwu).  The CPU frontend computes wstrb and
// byte-aligned wdata before asserting dc_req.
//
// AXI4 master: ARID/AWID=4'h2, ARLEN/AWLEN=3, ARSIZE/AWSIZE=3, BURST=INCR
// States: IDLE, LOOKUP, EVICT_AW, EVICT_W, EVICT_B, MISS, FILL, FILL_RESP,
//         FLUSH, ERR

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
  // Derived parameters
  // -------------------------------------------------------------------------
  localparam int SETS        = CACHE_SIZE_B / (WAYS * LINE_BYTES);    // 32
  localparam int OFFSET_BITS = $clog2(LINE_BYTES);                     // 5
  localparam int INDEX_BITS  = $clog2(SETS);                           // 5
  localparam int TAG_BITS    = PA_WIDTH - INDEX_BITS - OFFSET_BITS;   // 22
  localparam int BEATS       = LINE_BYTES / (AXI_DW / 8);             // 4
  localparam int BEAT_BITS   = $clog2(BEATS);                          // 2
  localparam int LINE_BITS   = LINE_BYTES * 8;                         // 256
  localparam int DW_SEL_BITS = OFFSET_BITS - 3;                        // 2 (addr[4:3])

  // -------------------------------------------------------------------------
  // Address decomposition (combinatorial, live CPU signals)
  // -------------------------------------------------------------------------
  logic [TAG_BITS-1:0]    req_tag_s;
  logic [INDEX_BITS-1:0]  req_idx_s;

  assign req_tag_s = cpu_if.dc_addr[PA_WIDTH-1 : INDEX_BITS+OFFSET_BITS];
  assign req_idx_s = cpu_if.dc_addr[INDEX_BITS+OFFSET_BITS-1 : OFFSET_BITS];

  // -------------------------------------------------------------------------
  // Behavioral SRAM models
  // -------------------------------------------------------------------------
  logic                   valid_r [0:SETS-1][0:WAYS-1];
  logic                   dirty_r [0:SETS-1][0:WAYS-1];
  logic [TAG_BITS-1:0]    tag_r   [0:SETS-1][0:WAYS-1];
  logic [2:0]             plru_r  [0:SETS-1];
  logic [LINE_BITS-1:0]   data_r  [0:SETS-1][0:WAYS-1];

  // -------------------------------------------------------------------------
  // FSM
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    IDLE      = 4'd0,
    LOOKUP    = 4'd1,
    EVICT_AW  = 4'd2,
    EVICT_W   = 4'd3,
    EVICT_B   = 4'd4,
    MISS      = 4'd5,
    FILL      = 4'd6,
    FILL_RESP = 4'd7,
    FLUSH     = 4'd8,
    ERR       = 4'd9
  } state_t;

  state_t state_r;

  // -------------------------------------------------------------------------
  // Latched request
  // -------------------------------------------------------------------------
  logic [TAG_BITS-1:0]      lk_tag_r;
  logic [INDEX_BITS-1:0]    lk_idx_r;
  logic [PA_WIDTH-1:0]      lk_line_addr_r;
  logic [2:0]               lk_size_r;      // dc_size = func3
  logic [2:0]               lk_byte_off_r;  // dc_addr[2:0]
  logic [DW_SEL_BITS-1:0]   lk_dw_sel_r;   // dc_addr[4:3]
  logic                     lk_wr_r;
  logic [63:0]              lk_wdata_r;
  logic [7:0]               lk_wstrb_r;

  // -------------------------------------------------------------------------
  // Hit detection (combinatorial on latched address)
  // -------------------------------------------------------------------------
  logic [WAYS-1:0]            hit_vec_s;
  logic                       hit_s;
  logic [$clog2(WAYS)-1:0]    hit_way_s;

  always_comb begin
    hit_vec_s = '0;
    for (int i = 0; i < WAYS; i++)
      hit_vec_s[i] = valid_r[lk_idx_r][i] & (tag_r[lk_idx_r][i] == lk_tag_r);
  end
  assign hit_s = |hit_vec_s;
  always_comb begin
    hit_way_s = '0;
    for (int i = WAYS-1; i >= 0; i--)
      if (hit_vec_s[i]) hit_way_s = i[$clog2(WAYS)-1:0];
  end

  // -------------------------------------------------------------------------
  // Victim selection: prefer invalid, fall back to PLRU
  // -------------------------------------------------------------------------
  logic [WAYS-1:0]            inv_vec_s;
  logic                       has_inv_s;
  logic [$clog2(WAYS)-1:0]    inv_way_s;
  logic [$clog2(WAYS)-1:0]    plru_victim_s;
  logic [$clog2(WAYS)-1:0]    fill_way_s;

  always_comb begin
    inv_vec_s = '0;
    for (int i = 0; i < WAYS; i++)
      inv_vec_s[i] = ~valid_r[lk_idx_r][i];
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
      plru_victim_s = p[1] ? 2'd1 : 2'd0;
    else
      plru_victim_s = p[2] ? 2'd3 : 2'd2;
  end
  assign fill_way_s = has_inv_s ? inv_way_s : plru_victim_s;

  // -------------------------------------------------------------------------
  // Fill / evict / flush registers
  // -------------------------------------------------------------------------
  logic [$clog2(WAYS)-1:0]  fill_way_r;
  logic [BEAT_BITS-1:0]     beat_r;
  logic [LINE_BITS-1:0]     fill_buf_r;
  logic [LINE_BITS-1:0]     evict_line_r;   // dirty line being written back

  logic [INDEX_BITS-1:0]    flush_idx_r;
  logic [$clog2(WAYS)-1:0]  flush_way_r;
  logic                     flush_mode_r;   // 1 = eviction from FLUSH, 0 = from miss

  // AXI4 AR (read fills)
  logic                     ar_valid_r;
  logic [PA_WIDTH-1:0]      ar_addr_r;

  // AXI4 AW (write-back address)
  logic                     aw_valid_r;
  logic [PA_WIDTH-1:0]      aw_addr_r;

  // CPU outputs
  logic [63:0]  rdata_r;
  logic         rvalid_r, stall_r, flush_done_r, err_r;

  // -------------------------------------------------------------------------
  // Port assignments
  // -------------------------------------------------------------------------
  assign cpu_if.dc_rdata      = rdata_r;
  assign cpu_if.dc_rvalid     = rvalid_r;
  assign cpu_if.dc_stall      = stall_r;
  assign cpu_if.dc_flush_done = flush_done_r;
  assign cpu_if.dc_err        = err_r;

  // AR channel
  assign axi_if.arvalid = ar_valid_r;
  assign axi_if.araddr  = PA_WIDTH'(ar_addr_r);
  assign axi_if.arid    = 4'h2;
  assign axi_if.arlen   = 8'h03;
  assign axi_if.arsize  = 3'b011;
  assign axi_if.arburst = 2'b01;
  assign axi_if.rready  = (state_r == FILL);

  // AW channel
  assign axi_if.awvalid = aw_valid_r;
  assign axi_if.awaddr  = PA_WIDTH'(aw_addr_r);
  assign axi_if.awid    = 4'h2;
  assign axi_if.awlen   = 8'h03;
  assign axi_if.awsize  = 3'b011;
  assign axi_if.awburst = 2'b01;

  // W channel: wvalid/wdata/wlast driven combinatorially from state + evict_line_r + beat_r
  assign axi_if.wvalid = (state_r == EVICT_W);
  assign axi_if.wdata  = evict_line_r[{beat_r, 6'd0} +: AXI_DW];
  assign axi_if.wstrb  = 8'hFF;
  assign axi_if.wlast  = (beat_r == BEAT_BITS'(BEATS - 1));

  // B channel
  assign axi_if.bready = (state_r == EVICT_B);

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
  // Read data path: extract doubleword from line, sign/zero-extend per dc_size
  // Mirrors asDMemBack logic, using byte-select protocol (dc_size = func3).
  // -------------------------------------------------------------------------
  function automatic logic [63:0] extract_load(
    input logic [LINE_BITS-1:0]   line,
    input logic [DW_SEL_BITS-1:0] dw_sel,
    input logic [2:0]             byte_off,
    input logic [2:0]             size
  );
    logic [63:0] dw   = line[{dw_sel, 6'd0} +: 64];
    logic [63:0] result;
    case (size)
      3'b000: case (byte_off)  // lb: sign-extend byte
                3'd0: result = {{56{dw[7]}},  dw[7:0]};
                3'd1: result = {{56{dw[15]}},  dw[15:8]};
                3'd2: result = {{56{dw[23]}},  dw[23:16]};
                3'd3: result = {{56{dw[31]}},  dw[31:24]};
                3'd4: result = {{56{dw[39]}},  dw[39:32]};
                3'd5: result = {{56{dw[47]}},  dw[47:40]};
                3'd6: result = {{56{dw[55]}},  dw[55:48]};
                default: result = {{56{dw[63]}}, dw[63:56]};
              endcase
      3'b001: case (byte_off[2:1])  // lh: sign-extend halfword
                2'd0: result = {{48{dw[15]}}, dw[15:0]};
                2'd1: result = {{48{dw[31]}}, dw[31:16]};
                2'd2: result = {{48{dw[47]}}, dw[47:32]};
                default: result = {{48{dw[63]}}, dw[63:48]};
              endcase
      3'b010: result = byte_off[2] ? {{32{dw[63]}}, dw[63:32]}  // lw: sign-extend word
                                   : {{32{dw[31]}}, dw[31:0]};
      3'b011: result = dw;                                        // ld
      3'b100: case (byte_off)  // lbu: zero-extend byte
                3'd0: result = {56'b0, dw[7:0]};
                3'd1: result = {56'b0, dw[15:8]};
                3'd2: result = {56'b0, dw[23:16]};
                3'd3: result = {56'b0, dw[31:24]};
                3'd4: result = {56'b0, dw[39:32]};
                3'd5: result = {56'b0, dw[47:40]};
                3'd6: result = {56'b0, dw[55:48]};
                default: result = {56'b0, dw[63:56]};
              endcase
      3'b101: case (byte_off[2:1])  // lhu: zero-extend halfword
                2'd0: result = {48'b0, dw[15:0]};
                2'd1: result = {48'b0, dw[31:16]};
                2'd2: result = {48'b0, dw[47:32]};
                default: result = {48'b0, dw[63:48]};
              endcase
      3'b110: result = byte_off[2] ? {32'b0, dw[63:32]}          // lwu: zero-extend word
                                   : {32'b0, dw[31:0]};
      default: result = dw;
    endcase
    return result;
  endfunction

  // -------------------------------------------------------------------------
  // Write merge: apply byte-enables to the addressed doubleword in the line
  // -------------------------------------------------------------------------
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
  // FSM
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin
    if (rst_i) begin
      state_r      <= IDLE;
      ar_valid_r   <= '0;
      aw_valid_r   <= '0;
      rvalid_r     <= '0;
      stall_r      <= '0;
      flush_done_r <= '0;
      err_r        <= '0;
      beat_r       <= '0;
      flush_mode_r <= '0;
      for (int s = 0; s < SETS; s++) begin
        plru_r[s] <= '0;
        for (int w = 0; w < WAYS; w++) begin
          valid_r[s][w] <= '0;
          dirty_r[s][w] <= '0;
        end
      end
    end else begin
      // Single-cycle pulse outputs
      rvalid_r     <= '0;
      flush_done_r <= '0;
      err_r        <= '0;

      case (state_r)

        // -- IDLE -----------------------------------------------------------
        IDLE: begin
          stall_r <= '0;
          if (cpu_if.dc_flush) begin
            stall_r     <= '1;
            flush_idx_r <= '0;
            flush_way_r <= '0;
            state_r     <= FLUSH;
          end else if (cpu_if.dc_req) begin
            lk_tag_r       <= req_tag_s;
            lk_idx_r       <= req_idx_s;
            lk_line_addr_r <= {cpu_if.dc_addr[PA_WIDTH-1 : OFFSET_BITS],
                               {OFFSET_BITS{1'b0}}};
            lk_size_r      <= cpu_if.dc_size;
            lk_byte_off_r  <= cpu_if.dc_addr[2:0];
            lk_dw_sel_r    <= cpu_if.dc_addr[OFFSET_BITS-1 : 3];
            lk_wr_r        <= cpu_if.dc_wr;
            lk_wdata_r     <= cpu_if.dc_wdata;
            lk_wstrb_r     <= cpu_if.dc_wstrb;
            stall_r        <= '1;
            state_r        <= LOOKUP;
          end
        end

        // -- LOOKUP ---------------------------------------------------------
        LOOKUP: begin
          if (hit_s) begin
            plru_r[lk_idx_r] <= plru_upd(plru_r[lk_idx_r], hit_way_s);
            if (lk_wr_r) begin
              // Write hit: merge with byte enables, mark dirty
              data_r[lk_idx_r][hit_way_s] <=
                merge_write(data_r[lk_idx_r][hit_way_s],
                            lk_dw_sel_r, lk_wdata_r, lk_wstrb_r);
              dirty_r[lk_idx_r][hit_way_s] <= '1;
              stall_r <= '0;
              state_r <= IDLE;
            end else begin
              // Read hit: extract and sign/zero-extend per dc_size
              rdata_r  <= extract_load(data_r[lk_idx_r][hit_way_s],
                                       lk_dw_sel_r, lk_byte_off_r, lk_size_r);
              rvalid_r <= '1;
              stall_r  <= '0;
              state_r  <= IDLE;
            end
          end else begin
            // Miss: choose victim
            fill_way_r <= fill_way_s;
            if (valid_r[lk_idx_r][fill_way_s] && dirty_r[lk_idx_r][fill_way_s]) begin
              // Dirty victim: write back first
              automatic logic [TAG_BITS-1:0] ev_tag = tag_r[lk_idx_r][fill_way_s];
              evict_line_r <= data_r[lk_idx_r][fill_way_s];
              aw_addr_r    <= {ev_tag, lk_idx_r, {OFFSET_BITS{1'b0}}};
              aw_valid_r   <= '1;
              beat_r       <= '0;
              flush_mode_r <= '0;
              state_r      <= EVICT_AW;
            end else begin
              // Clean or invalid victim: fill directly
              ar_addr_r  <= lk_line_addr_r;
              ar_valid_r <= '1;
              beat_r     <= '0;
              fill_buf_r <= '0;
              state_r    <= MISS;
            end
          end
        end

        // -- EVICT_AW: send AW handshake for write-back ---------------------
        EVICT_AW: begin
          if (axi_if.awready) begin
            aw_valid_r <= '0;
            beat_r     <= '0;
            state_r    <= EVICT_W;
          end
        end

        // -- EVICT_W: send W beats (wdata/wlast driven combinatorially) -----
        EVICT_W: begin
          if (axi_if.wready) begin
            if (beat_r == BEAT_BITS'(BEATS - 1)) begin
              state_r <= EVICT_B;
            end else begin
              beat_r <= beat_r + 1;
            end
          end
        end

        // -- EVICT_B: wait for B response -----------------------------------
        EVICT_B: begin
          if (axi_if.bvalid) begin
            if (|axi_if.bresp) begin
              err_r   <= '1;
              stall_r <= '0;
              state_r <= ERR;
            end else if (flush_mode_r) begin
              // Write-back during flush: invalidate this line, continue FLUSH
              valid_r[flush_idx_r][flush_way_r] <= '0;
              dirty_r[flush_idx_r][flush_way_r] <= '0;
              // Advance flush pointer
              if (flush_way_r == $clog2(WAYS)'(WAYS-1)) begin
                flush_way_r <= '0;
                if (&flush_idx_r) begin
                  flush_done_r <= '1;
                  stall_r      <= '0;
                  state_r      <= IDLE;
                end else begin
                  flush_idx_r <= flush_idx_r + 1;
                  state_r     <= FLUSH;
                end
              end else begin
                flush_way_r <= flush_way_r + 1;
                state_r     <= FLUSH;
              end
            end else begin
              // Write-back during miss: now fill the new line
              ar_addr_r  <= lk_line_addr_r;
              ar_valid_r <= '1;
              beat_r     <= '0;
              fill_buf_r <= '0;
              state_r    <= MISS;
            end
          end
        end

        // -- MISS: AR handshake for fill ------------------------------------
        MISS: begin
          if (axi_if.arready) begin
            ar_valid_r <= '0;
            state_r    <= FILL;
          end
        end

        // -- FILL: receive 4 R-channel beats --------------------------------
        FILL: begin
          if (axi_if.rvalid) begin
            automatic logic [LINE_BITS-1:0] nl;
            nl = fill_buf_r;
            nl[{beat_r, 6'd0} +: AXI_DW] = axi_if.rdata;

            if (|axi_if.rresp) begin
              err_r   <= '1;
              stall_r <= '0;
              state_r <= ERR;
            end else if (axi_if.rlast) begin
              // For write miss: merge write data into filled line
              if (lk_wr_r)
                nl = merge_write(nl, lk_dw_sel_r, lk_wdata_r, lk_wstrb_r);
              data_r [lk_idx_r][fill_way_r] <= nl;
              tag_r  [lk_idx_r][fill_way_r] <= lk_tag_r;
              valid_r[lk_idx_r][fill_way_r] <= '1;
              dirty_r[lk_idx_r][fill_way_r] <= lk_wr_r;
              plru_r [lk_idx_r]             <= plru_upd(plru_r[lk_idx_r], fill_way_r);
              // Pre-compute rdata for read miss (stable before FILL_RESP asserts rvalid)
              if (!lk_wr_r)
                rdata_r <= extract_load(nl, lk_dw_sel_r, lk_byte_off_r, lk_size_r);
              state_r <= FILL_RESP;
            end else begin
              fill_buf_r <= nl;
              beat_r     <= beat_r + 1;
            end
          end
        end

        // -- FILL_RESP: present data (or write ack) to CPU ------------------
        FILL_RESP: begin
          if (!lk_wr_r)
            rvalid_r <= '1;
          stall_r <= '0;
          state_r <= IDLE;
        end

        // -- ERR: AXI4 error ------------------------------------------------
        ERR: begin
          state_r <= IDLE;
        end

        // -- FLUSH: write back dirty lines, then invalidate all -------------
        // One (set,way) cell processed per cycle.
        // Dirty cells trigger write-back via EVICT_AW/W/B with flush_mode_r=1.
        FLUSH: begin
          if (valid_r[flush_idx_r][flush_way_r] &&
              dirty_r[flush_idx_r][flush_way_r]) begin
            // Dirty: set up write-back
            automatic logic [TAG_BITS-1:0] ft = tag_r[flush_idx_r][flush_way_r];
            evict_line_r <= data_r[flush_idx_r][flush_way_r];
            aw_addr_r    <= {ft, flush_idx_r, {OFFSET_BITS{1'b0}}};
            aw_valid_r   <= '1;
            beat_r       <= '0;
            flush_mode_r <= '1;
            state_r      <= EVICT_AW;
            // Invalidation happens in EVICT_B when flush_mode_r=1
          end else begin
            // Clean or invalid: just invalidate
            valid_r[flush_idx_r][flush_way_r] <= '0;
            dirty_r[flush_idx_r][flush_way_r] <= '0;
            if (flush_way_r == $clog2(WAYS)'(WAYS-1)) begin
              flush_way_r <= '0;
              if (&flush_idx_r) begin
                flush_done_r <= '1;
                stall_r      <= '0;
                state_r      <= IDLE;
              end else begin
                flush_idx_r <= flush_idx_r + 1;
              end
            end else begin
              flush_way_r <= flush_way_r + 1;
            end
          end
        end

        default: state_r <= IDLE;
      endcase
    end
  end

endmodule : asDCache
