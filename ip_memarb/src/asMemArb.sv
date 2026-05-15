
// asMemArb.sv
`timescale 1ns/1ps

import as_pack::*;

// Memory Arbiter
//
// Serialises AXI4 read bursts from two cache controllers onto the single
// QSPI controller AXI4 slave port.
//
// Policy: fixed priority D-Cache (high) > I-Cache (low).
//         An active burst is never interrupted: the arbiter holds the grant
//         from the AR handshake through to RLAST acknowledgement.
//
// Data path: purely combinatorial mux (no pipeline registers added).
// State:     one 2-bit register tracking which master currently owns the bus.
//
// Write channels (AW/W/B): not used.
//   NOR Flash is read-only; D-Cache writes go to Scratchpad SRAM which
//   bypasses the AXI4 bus entirely (MEM-01 resolution).
//   All write-channel outputs are tied to safe defaults.
//
// Ports:
//   icache_axi4  -- AXI4 slave port (I-Cache is the AXI4 master)
//   dcache_axi4  -- AXI4 slave port (D-Cache is the AXI4 master)
//   qspi_axi4    -- AXI4 master port (QSPI controller is the AXI4 slave)
//
// Spec: 03_arch_memory_concept.tex, Section "Memory Arbiter"

module asMemArb (
  input  logic      clk_i,
  input  logic      rst_i,
  as_axi4_if.slave  icache_axi4,
  as_axi4_if.slave  dcache_axi4,
  as_axi4_if.master qspi_axi4
);

  typedef enum logic [1:0] {
    ARB_IDLE_ST    = 2'b00,
    ARB_GRANT_D_ST = 2'b01,  // D-Cache owns the bus
    ARB_GRANT_I_ST = 2'b10   // I-Cache owns the bus
  } statetype_t;
  statetype_t state_s, nextstate_s;

  // Three block FSM for education purpose:
  // FSM block: delay
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
      state_s <= ARB_IDLE_ST;
    else
      state_s <= nextstate_s;
  end
  
  // FSM block: Input logic -> calculates state_s
  // ---------------------------------------------------------------------------
  // State Logic
  //
  // Transitions:
  //   IDLE     → GRANT_D  when D-Cache presents arvalid (D has priority)
  //   IDLE     → GRANT_I  when only I-Cache presents arvalid
  //   GRANT_D  → IDLE     when QSPI delivers RLAST and D-Cache accepts it
  //   GRANT_I  → IDLE     when QSPI delivers RLAST and I-Cache accepts it
  // ---------------------------------------------------------------------------
  always_comb 
  begin
    nextstate_s = state_s; // self arc
    case(state_s)
      ARB_IDLE_ST:     begin
                       if (dcache_axi4.arvalid)
                         nextstate_s = ARB_GRANT_D_ST;
                       else if (icache_axi4.arvalid)
                         nextstate_s = ARB_GRANT_I_ST;
                       end
      ARB_GRANT_D_ST:  if (qspi_axi4.rvalid & qspi_axi4.rlast & dcache_axi4.rready)
                         nextstate_s = ARB_IDLE_ST;
      ARB_GRANT_I_ST:  if (qspi_axi4.rvalid & qspi_axi4.rlast & icache_axi4.rready)
                         nextstate_s = ARB_IDLE_ST;
      default: nextstate_s = ARB_IDLE_ST;
    endcase
  end
  
  // FSM block: Output logic -> calculates outputs
  // ---------------------------------------------------------------------------
  // Combinatorial grant signals
  //
  // grant_d_s is high when D-Cache owns or immediately wins the bus:
  //   - We are in GRANT_D (burst in progress), OR
  //   - We are in IDLE and D-Cache has a pending AR request
  //     (D-Cache has higher priority, so it preempts I-Cache in IDLE)
  //
  // grant_i_s is high when I-Cache owns or immediately wins the bus:
  //   - We are in GRANT_I (burst in progress), OR
  //   - We are in IDLE, D-Cache has no pending request, and I-Cache does
  // ---------------------------------------------------------------------------
  logic grant_d_s, grant_i_s;

  assign grant_d_s = (state_s == ARB_GRANT_D_ST) | (state_s == ARB_IDLE_ST & dcache_axi4.arvalid);
  assign grant_i_s = (state_s == ARB_GRANT_I_ST) | (state_s == ARB_IDLE_ST & ~dcache_axi4.arvalid & icache_axi4.arvalid);

  // ---------------------------------------------------------------------------
  // AR channel: mux selected master → QSPI
  // ---------------------------------------------------------------------------
  assign qspi_axi4.arvalid = grant_d_s ? dcache_axi4.arvalid : grant_i_s ? icache_axi4.arvalid : '0;
  assign qspi_axi4.arid    = grant_d_s ? dcache_axi4.arid    : grant_i_s ? icache_axi4.arid    : '0;
  assign qspi_axi4.araddr  = grant_d_s ? dcache_axi4.araddr  : grant_i_s ? icache_axi4.araddr  : '0;
  assign qspi_axi4.arlen   = grant_d_s ? dcache_axi4.arlen   : grant_i_s ? icache_axi4.arlen   : '0;
  assign qspi_axi4.arsize  = grant_d_s ? dcache_axi4.arsize  : grant_i_s ? icache_axi4.arsize  : '0;
  assign qspi_axi4.arburst = grant_d_s ? dcache_axi4.arburst : grant_i_s ? icache_axi4.arburst : '0;

  // ARREADY: QSPI → selected master; non-selected master sees 0
  assign dcache_axi4.arready = grant_d_s ? qspi_axi4.arready : '0;
  assign icache_axi4.arready = grant_i_s ? qspi_axi4.arready : '0;

  // ---------------------------------------------------------------------------
  // R channel: QSPI → selected master; non-selected master sees idle values
  // ---------------------------------------------------------------------------
  assign dcache_axi4.rvalid = grant_d_s ? qspi_axi4.rvalid : '0;
  assign dcache_axi4.rid    = grant_d_s ? qspi_axi4.rid    : '0;
  assign dcache_axi4.rdata  = grant_d_s ? qspi_axi4.rdata  : '0;
  assign dcache_axi4.rresp  = grant_d_s ? qspi_axi4.rresp  : '0;
  assign dcache_axi4.rlast  = grant_d_s ? qspi_axi4.rlast  : '0;

  assign icache_axi4.rvalid = grant_i_s ? qspi_axi4.rvalid : '0;
  assign icache_axi4.rid    = grant_i_s ? qspi_axi4.rid    : '0;
  assign icache_axi4.rdata  = grant_i_s ? qspi_axi4.rdata  : '0;
  assign icache_axi4.rresp  = grant_i_s ? qspi_axi4.rresp  : '0;
  assign icache_axi4.rlast  = grant_i_s ? qspi_axi4.rlast  : '0;

  // RREADY: selected master → QSPI
  assign qspi_axi4.rready = grant_d_s ? dcache_axi4.rready : grant_i_s ? icache_axi4.rready : '0;

  // ---------------------------------------------------------------------------
  // Write channels: tied off (Flash is read-only; no AXI4 write path)
  // ---------------------------------------------------------------------------
  assign qspi_axi4.awvalid = '0;
  assign qspi_axi4.awid    = '0;
  assign qspi_axi4.awaddr  = '0;
  assign qspi_axi4.awlen   = '0;
  assign qspi_axi4.awsize  = '0;
  assign qspi_axi4.awburst = '0;
  assign qspi_axi4.wvalid  = '0;
  assign qspi_axi4.wdata   = '0;
  assign qspi_axi4.wstrb   = '0;
  assign qspi_axi4.wlast   = '0;
  assign qspi_axi4.bready  = '1;

  assign dcache_axi4.awready = '1;
  assign dcache_axi4.wready  = '1;
  assign dcache_axi4.bid     = '0;
  assign dcache_axi4.bresp   = '0;
  assign dcache_axi4.bvalid  = '0;

  assign icache_axi4.awready = '1;
  assign icache_axi4.wready  = '1;
  assign icache_axi4.bid     = '0;
  assign icache_axi4.bresp   = '0;
  assign icache_axi4.bvalid  = '0;

endmodule : asMemArb
