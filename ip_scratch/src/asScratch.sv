`timescale 1ns/1ps

module asScratch #(
    parameter int SP_DEPTH = 1024,   // 64-bit words; must be power of 2
    parameter int PA_WIDTH = 32      // physical address width (unused; for doc)
) (
    input  logic clk_i,
    input  logic rst_i,
    as_dcache_if.cache cpu_if
);

  localparam int ADDR_W = $clog2(SP_DEPTH);

  // -------------------------------------------------------------------------
  // 2. ALL signal declarations
  // -------------------------------------------------------------------------
  typedef enum logic {
    IDLE_ST = 1'b0,
    BUSY_ST = 1'b1
  } state_t;

  state_t sc_state_s;
  state_t sc_nextstate_s;

  logic [ADDR_W-1:0] waddr_s;
  logic [2:0]        size_r;
  logic [2:0]        boff_r;
  logic [63:0]       sram_rdata_s;

  // -------------------------------------------------------------------------
  // 3. assign statements
  // -------------------------------------------------------------------------
  assign waddr_s = cpu_if.dc_addr[ADDR_W+2:3];

  // MEALY: dc_stall depends on state AND dc_req/dc_wr inputs
  assign cpu_if.dc_stall      = cpu_if.dc_req & ~cpu_if.dc_wr & (sc_state_s == IDLE_ST);
  assign cpu_if.dc_rvalid     = (sc_state_s == BUSY_ST);
  assign cpu_if.dc_flush_done = 1'b0;
  assign cpu_if.dc_err        = 1'b0;

  // -------------------------------------------------------------------------
  // 4. always_comb blocks
  // -------------------------------------------------------------------------

  // FSM block: input logic
  always_comb begin
    sc_nextstate_s = sc_state_s;
    case (sc_state_s)
      IDLE_ST:
        if (cpu_if.dc_req && !cpu_if.dc_wr)
          sc_nextstate_s = BUSY_ST;
      BUSY_ST:
        sc_nextstate_s = IDLE_ST;
      default: sc_nextstate_s = IDLE_ST;
    endcase
  end

  // Read data sign/zero extension (RISC-V dc_size encoding)
  always_comb begin
    automatic logic [5:0] bsel = {boff_r,        3'b000};
    automatic logic [5:0] hsel = {boff_r[2:1], 4'b0000};
    automatic logic [5:0] wsel = {boff_r[2],  5'b00000};
    case (size_r)
      3'b000: cpu_if.dc_rdata = {{56{sram_rdata_s[bsel+7]}},  sram_rdata_s[bsel +: 8 ]};  // lb
      3'b001: cpu_if.dc_rdata = {{48{sram_rdata_s[hsel+15]}}, sram_rdata_s[hsel +: 16]};  // lh
      3'b010: cpu_if.dc_rdata = {{32{sram_rdata_s[wsel+31]}}, sram_rdata_s[wsel +: 32]};  // lw
      3'b011: cpu_if.dc_rdata = sram_rdata_s;                                               // ld
      3'b100: cpu_if.dc_rdata = {56'h0, sram_rdata_s[bsel +: 8 ]};                        // lbu
      3'b101: cpu_if.dc_rdata = {48'h0, sram_rdata_s[hsel +: 16]};                        // lhu
      3'b110: cpu_if.dc_rdata = {32'h0, sram_rdata_s[wsel +: 32]};                        // lwu
      default: cpu_if.dc_rdata = sram_rdata_s;
    endcase
  end

  // -------------------------------------------------------------------------
  // 5. always_ff blocks
  // -------------------------------------------------------------------------

  // FSM block: delay
  always_ff @(posedge clk_i or posedge rst_i) begin
    if (rst_i)
      sc_state_s <= IDLE_ST;
    else
      sc_state_s <= sc_nextstate_s;
  end

  // FSM block: output logic
  always_ff @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
      size_r <= '0;
      boff_r <= '0;
    end else begin
      case (sc_state_s)
        IDLE_ST:
          if (cpu_if.dc_req && !cpu_if.dc_wr) begin
            size_r <= cpu_if.dc_size;
            boff_r <= cpu_if.dc_addr[2:0];
          end
        default: ;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // 6. Module instantiations
  // -------------------------------------------------------------------------
  // To target ASIC: replace as_sram_sp64 with the X-Fab SRAM wrapper.
  as_sram_sp64 #(.DEPTH(SP_DEPTH)) sram (
    .clk_i  (clk_i),
    .cen_i  (cpu_if.dc_req & (sc_state_s == IDLE_ST)),
    .we_i   (cpu_if.dc_wr),
    .wbe_i  (cpu_if.dc_wstrb),
    .addr_i (waddr_s),
    .wdata_i(cpu_if.dc_wdata),
    .rdata_o(sram_rdata_s)
  );

endmodule : asScratch
