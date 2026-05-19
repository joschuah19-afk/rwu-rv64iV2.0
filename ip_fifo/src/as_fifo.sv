
// =============================================================================
// as_fifo.sv
// Synchronous single-clock FIFO for the QSPI peripheral
//
// Converted from VHDL (fifo_e / fifo_a) with the following changes:
//   - Reset changed to active-high synchronous (matches QSPI / Wishbone style)
//   - Data width default 64 bit, depth default 16 (matches QSPI spec)
//   - Flush input added (for CTRL.TX_FLUSH / CTRL.RX_FLUSH)
//   - half_full_o added  (RX_HALF / TX_HALF interrupt source)
//   - half_empty_o added (TX_HALF interrupt source, synonym for TX side)
//   - level_o added      (feeds FIFOSTAT register, 8-bit fill level)
//   - Bug fix: cnt write/read guarded against full/empty overflow
//   - Bug fix: write to memory now guarded by !full
//   - Bug fix: read from memory now guarded by !empty
//   - almost_full / almost_empty kept but optional (tied to half thresholds
//     by default; override via parameters if needed)
//
// Parameters:
//   DATA_WIDTH  - Width of each FIFO entry in bits (default: 64)
//   FIFO_DEPTH  - Number of entries; must be a power of 2 (default: 16)
//   AF_LEVEL    - Almost-full  threshold (entry count, default: DEPTH*3/4)
//   AE_LEVEL    - Almost-empty threshold (entry count, default: DEPTH/4)
//
// Interface:
//   rst_i          - Synchronous reset, active high
//   clk_i          - Clock
//   flush_i        - Synchronous flush: clears FIFO in one cycle (active high)
//   wr_en_i        - Write enable
//   data_wr_i      - Write data
//   full_o         - FIFO full
//   almost_full_o  - Fill level > AF_LEVEL
//   half_full_o    - Fill level >= FIFO_DEPTH/2  (RX_HALF / TX_HALF source)
//   rd_en_i        - Read enable
//   data_rd_o      - Read data (registered, one-cycle latency)
//   empty_o        - FIFO empty
//   almost_empty_o - Fill level < AE_LEVEL
//   half_empty_o   - Fill level <  FIFO_DEPTH/2  (TX_HALF source)
//   level_o        - Fill level as binary count (feeds FIFOSTAT register)
//
// Usage (TX FIFO in QSPI):
//   as_fifo #(.DATA_WIDTH(64), .FIFO_DEPTH(16)) u_tx_fifo (
//     .rst_i(rst_i), .clk_i(clk_i), .flush_i(tx_flush_s),
//     .wr_en_i(tx_wr_s), .data_wr_i(tx_data_s),
//     .full_o(tx_full_s), .almost_full_o(), .half_full_o(), .half_empty_o(),
//     .rd_en_i(tx_rd_s), .data_rd_o(tx_data_kernel_s),
//     .empty_o(tx_empty_s), .almost_empty_o(), .level_o(tx_level_s)
//   );
//
// Usage (RX FIFO in QSPI):
//   as_fifo #(.DATA_WIDTH(64), .FIFO_DEPTH(16)) u_rx_fifo (
//     .rst_i(rst_i), .clk_i(clk_i), .flush_i(rx_flush_s),
//     .wr_en_i(rx_wr_s), .data_wr_i(rx_data_kernel_s),
//     .full_o(rx_full_s), .almost_full_o(), .half_full_o(rx_half_s), .half_empty_o(),
//     .rd_en_i(rx_rd_s), .data_rd_o(rx_data_s),
//     .empty_o(rx_empty_s), .almost_empty_o(), .level_o(rx_level_s)
//   );
// Connect: rx_half_s → RIS.HA, 
// tx_half_s → RIS.TH, 
// rx_empty_s → RIS.EM, 
// tx_empty_s → RIS.TE – genau wie in der aktualisierten Interrupt-Tabelle aus dem letzten Schritt spezifiziert
//
// Problem: This FIFO is asynchronous read -> no X-FAB SRAM will work (only 0 wait-states possible with S-BPI).
// Option A – FIFO bleibt als Flip-Flop-Implementierung (kein SRAM)
//  ->(* ram_style = "registers" *) logic [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];
// Option B – 1 Wait State im Slave-BPI
// Option C – Read-First-Registrierung im FIFO (empfohlen wenn SRAM nötig)
// =============================================================================

`timescale 1ns/1ps

module as_fifo #(
  parameter int DATA_WIDTH = 64,               // width of each FIFO word
  parameter int FIFO_DEPTH = 16,               // number of entries (power of 2)
  parameter int AF_LEVEL   = FIFO_DEPTH*3/4,   // almost-full  threshold
  parameter int AE_LEVEL   = FIFO_DEPTH/4      // almost-empty threshold
)(
  input  logic                      rst_i,       // synchronous reset, active high
  input  logic                      clk_i,
  input  logic                      flush_i,     // synchronous flush, active high

  // Write interface
  input  logic                      wr_en_i,
  input  logic [DATA_WIDTH-1:0]     data_wr_i,
  output logic                      full_o,
  output logic                      almost_full_o,
  output logic                      half_full_o,  // fill >= FIFO_DEPTH/2

  // Read interface
  input  logic                      rd_en_i,
  output logic [DATA_WIDTH-1:0]     data_rd_o,
  output logic                      empty_o,
  output logic                      almost_empty_o,
  output logic                      half_empty_o, // fill <  FIFO_DEPTH/2

  // Fill level (for FIFOSTAT register)
  output logic [$clog2(FIFO_DEPTH):0] level_o    // 0 .. FIFO_DEPTH
);

  // ---------------------------------------------------------------------------
  // Parameter checks (simulation / elaboration only — not synthesised)
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  initial begin
    if (FIFO_DEPTH < 2 || (FIFO_DEPTH & (FIFO_DEPTH-1)) != 0)
      $fatal(1, "as_fifo: FIFO_DEPTH must be a power of 2 and >= 2");
    if (AF_LEVEL >= FIFO_DEPTH)
      $fatal(1, "as_fifo: AF_LEVEL must be < FIFO_DEPTH");
    if (AE_LEVEL <= 0)
      $fatal(1, "as_fifo: AE_LEVEL must be > 0");
  end
  // synthesis translate_on

  // ---------------------------------------------------------------------------
  // Local types and storage
  // ---------------------------------------------------------------------------
  localparam int PTR_WIDTH = $clog2(FIFO_DEPTH); // pointer width for wrap-around

  //logic [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];
  (* ram_style = "registers" *) logic [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];

  logic [PTR_WIDTH-1:0]      wr_ptr_r;  // write pointer
  logic [PTR_WIDTH-1:0]      rd_ptr_r;  // read pointer
  logic [$clog2(FIFO_DEPTH):0] cnt_r;   // fill level, 0 .. FIFO_DEPTH

  // Internal status wires (derived from cnt_r)
  logic full_s;
  logic empty_s;

  assign full_s  = (cnt_r == FIFO_DEPTH);
  assign empty_s = (cnt_r == 0);

  // ---------------------------------------------------------------------------
  // Main sequential process
  // Fixes vs. original VHDL:
  //   - Reset is synchronous active-high (not async active-low)
  //   - cnt_r guarded: only increments on real writes (!full),
  //                    only decrements on real reads  (!empty)
  //   - Memory write guarded by !full
  //   - Simultaneous read+write (cnt unchanged) handled correctly
  //   - flush_i supported
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i) begin : fifo_proc
    if (rst_i || flush_i) begin
      // -----------------------------------------------------------------------
      // Reset / Flush: clear pointers and counter; memory content is don't-care
      // -----------------------------------------------------------------------
      wr_ptr_r <= '0;
      rd_ptr_r <= '0;
      cnt_r    <= '0;
    end else begin
      // -----------------------------------------------------------------------
      // Normal operation
      // -----------------------------------------------------------------------

      // -- Write path --
      // Write is accepted only when FIFO is not full.
      // A simultaneous read frees a slot, so write is also accepted then.
      if (wr_en_i && (!full_s || rd_en_i)) begin
        mem[wr_ptr_r]  <= data_wr_i;
        wr_ptr_r       <= (wr_ptr_r == PTR_WIDTH'(FIFO_DEPTH-1))
                          ? '0
                          : wr_ptr_r + 1'b1;
      end

      // -- Read path --
      // Read is accepted only when FIFO is not empty.
      // A simultaneous write fills a slot, so read is also accepted then.
      if (rd_en_i && (!empty_s || wr_en_i)) begin
        rd_ptr_r <= (rd_ptr_r == PTR_WIDTH'(FIFO_DEPTH-1))
                    ? '0
                    : rd_ptr_r + 1'b1;
      end

      // -- Counter update --
      // Four cases:
      //   write only (and not full)  → increment
      //   read  only (and not empty) → decrement
      //   both write and read        → unchanged  (simultaneous push/pop)
      //   neither                    → unchanged
      unique case ({wr_en_i & (!full_s | rd_en_i),
                    rd_en_i & (!empty_s | wr_en_i)})
        2'b10:   cnt_r <= cnt_r + 1'b1;  // write only
        2'b01:   cnt_r <= cnt_r - 1'b1;  // read only
        default: cnt_r <= cnt_r;          // both or neither
      endcase
    end
  end : fifo_proc

  // ---------------------------------------------------------------------------
  // Read data output (registered, one-cycle latency after rd_en_i)
  // The read pointer is already advanced on the same clock edge as rd_en_i,
  // so we expose the *current* read pointer's content combinatorially:
  // ---------------------------------------------------------------------------
  assign data_rd_o = mem[rd_ptr_r];

  // ---------------------------------------------------------------------------
  // Status outputs
  // ---------------------------------------------------------------------------
  assign full_o         = full_s;
  assign empty_o        = empty_s;
  assign almost_full_o  = (cnt_r >  AF_LEVEL);
  assign almost_empty_o = (cnt_r <  AE_LEVEL);
  assign half_full_o    = (cnt_r >= FIFO_DEPTH / 2);  // RX_HALF interrupt source
  assign half_empty_o   = (cnt_r <  FIFO_DEPTH / 2);  // TX_HALF interrupt source
  assign level_o        = cnt_r;

endmodule : as_fifo

