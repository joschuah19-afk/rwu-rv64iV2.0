
// asTopMem.sv – Cache version (RV_NoPipelineCache)
//
// Differences from RV_NoPipeline/src/asTopMem.sv:
//   - asIMem and asDMemTop removed; replaced by asMemTop (I-Cache + D-Cache + Scratchpad).
//   - Both as_master_bpi removed; CPU uses icpu_if / dcpu_if directly.
//   - I-Mem JTAG scan chain (dr_reg) removed; instructions come from flash via QSPI.
//   - as_qspi_top instantiated; exposes physical QSPI pins (sck, cs, data[3:0]).
//   - Peripheral bridge: CPU data accesses with dc_addr[32]=1 are routed to WB bus
//     (GPIO, QSPI ctrl, CGU) with 1-cycle dc_rvalid.
//   - SP_BASE=0 so scratchpad covers 0x0000_0000–0x0000_1FFF (existing test data addrs).
`timescale 1ns/1ps

import as_pack::*;

module as_top_mem (
    input  logic                    clk_i,
    input  logic                    rst_i,
    // JTAG
    input  logic                    tck_i,
    input  logic                    trst_i,
    input  logic                    tms_i,
    input  logic                    tdi_i,
    output logic                    tdo_o,
    // GPIO
    inout  tri [nr_gpios-1:0]       gpio_io,
    output logic                    cs_o,
    // QSPI flash (physical pins)
    output logic                    sck_o,
    output logic                    flash_cs_o,
    inout  tri   [3:0]              flash_data_io,
    // Clock for testbench (= clk_div_s = clk_core_s)
    output logic                    clk_div_o
);

  // ── Signal declarations ─────────────────────────────────────────
  localparam int QSPI_AW = 8;
  logic clk_core_s, clk_qspi_s, clk_bus1_s, clk_bus2_s;
  logic dr_cap_s;
  logic clk_div_s;

  // JTAG
  logic tap_rst_s;
  logic sc01_tdo_s, sc01_tdi_s, sc01_shift_s, sc01_clock_s;
  logic im_tdo_s, im_tdi_s, im_shift_s, im_clock_s, im_upd_s, im_mode_s;
  logic bs_tdo_s, bs_tdi_s, bs_shift_s, bs_clock_s, bs_upd_s, bs_mode_s;

  // Peripheral bridge
  logic [chipsel-1:0]     csx_s;
  logic                   wbdwe_s;
  logic [wbdSel-1:0]      sel_s;
  logic [daddr_width-1:0] dBusAddr_periph_s;
  logic [reg_width-1:0]   dBusDataWr_periph_s;
  logic [reg_width-1:0]   dBusDataRdGpio_s;
  logic [reg_width-1:0]   dBusDataRdQspi_s;
  logic [reg_width-1:0]   dBusDataRdCgu_s;
  logic [reg_width-1:0]   periph_rdata_s;
  logic                   wbdstbGpio_s, wbdstbQspi_s, wbdstbCgu_s;
  // WB ACK signals intentionally unconnected: bridge uses fixed 1-cycle dc_rvalid
  logic                   is_periph_req_s;
  logic                   is_periph_r;
  logic                   is_periph_s;
  logic                   periph_rvalid_r;

  // GPIO IRQ, cs
  logic irq_gpiox_s;
  logic asGpioCs_s;

  // QSPI IRQ
  logic qspi_irq_s;

  // IRQ
  logic [irq_total_num_ext_c-1:0] irq_external_s;

  // ── Internal interfaces ─────────────────────────────────────────
  as_icache_if icpu_if_s   (.clk_i(clk_div_s), .rst_i(rst_i));
  as_dcache_if dcpu_if_s   (.clk_i(clk_div_s), .rst_i(rst_i));
  as_dcache_if dcpu_mem_if_s(.clk_i(clk_div_s), .rst_i(rst_i));
  as_axi4_if #(.ADDR_W(32)) qspi_if_s (.clk_i(clk_qspi_s), .rst_i(rst_i));

  // ── Clock mux (scan test) ───────────────────────────────────────
  assign clk_div_s  = dr_cap_s ? clk_i : clk_core_s;
  assign clk_div_o  = clk_div_s;
  // All CGU domain clocks tied to core clock — no CDC needed until fast/slow
  // peripherals are added to the SoC and the CGU outputs are re-enabled.
  assign clk_qspi_s = clk_div_s;
  assign clk_bus1_s = clk_div_s;
  assign clk_bus2_s = clk_div_s;

  assign cs_o = asGpioCs_s;

  // ── Address decode (peripheral routing) ─────────────────────────
  assign dBusAddr_periph_s   = dcpu_if_s.dc_addr;
  assign dBusDataWr_periph_s = dcpu_if_s.dc_wdata;
  assign wbdwe_s             = dcpu_if_s.dc_wr;
  assign sel_s               = '1;  // slave BPI requires all-bytes-set; matches original master BPI behaviour

  as_decode addressDecode (dBusAddr_periph_s, csx_s);

  // Peripheral read-data mux
  always_comb
    case(csx_s)
      4'd2:    periph_rdata_s = dBusDataRdGpio_s;
      4'd4:    periph_rdata_s = dBusDataRdQspi_s;
      4'd8:    periph_rdata_s = dBusDataRdCgu_s;
      default: periph_rdata_s = '0;
    endcase

  // ── Peripheral bridge ───────────────────────────────────────────
  assign is_periph_req_s = dcpu_if_s.dc_req && dcpu_if_s.dc_addr[32];

  always_ff @(posedge clk_div_s, posedge rst_i) begin
    if(rst_i) is_periph_r <= 1'b0;
    else if(dcpu_if_s.dc_req) is_periph_r <= dcpu_if_s.dc_addr[32];
  end

  assign is_periph_s = dcpu_if_s.dc_req ? dcpu_if_s.dc_addr[32] : is_periph_r;

  // dc_rvalid for peripheral = 1 cycle after dc_req pulse
  always_ff @(posedge clk_div_s, posedge rst_i) begin
    if(rst_i) periph_rvalid_r <= 1'b0;
    else       periph_rvalid_r <= is_periph_req_s;
  end

  assign wbdstbGpio_s = is_periph_req_s & csx_s[1];
  assign wbdstbQspi_s = is_periph_req_s & csx_s[2];
  assign wbdstbCgu_s  = is_periph_req_s & csx_s[3];

  // ── Route CPU → asMemTop (non-peripheral only) ───────────────────
  assign dcpu_mem_if_s.dc_addr  = dcpu_if_s.dc_addr;
  assign dcpu_mem_if_s.dc_req   = dcpu_if_s.dc_req & ~dcpu_if_s.dc_addr[32];
  assign dcpu_mem_if_s.dc_wr    = dcpu_if_s.dc_wr;
  assign dcpu_mem_if_s.dc_size  = dcpu_if_s.dc_size;
  assign dcpu_mem_if_s.dc_wdata = dcpu_if_s.dc_wdata;
  assign dcpu_mem_if_s.dc_wstrb = dcpu_if_s.dc_wstrb;
  assign dcpu_mem_if_s.dc_flush = dcpu_if_s.dc_flush;

  // ── dcpu_if_s cache-side: mux from asMemTop + peripheral bridge ──
  assign dcpu_if_s.dc_rvalid     = is_periph_s ? periph_rvalid_r    : dcpu_mem_if_s.dc_rvalid;
  assign dcpu_if_s.dc_rdata      = is_periph_s ? periph_rdata_s     : dcpu_mem_if_s.dc_rdata;
  assign dcpu_if_s.dc_stall      = is_periph_s ? 1'b0               : dcpu_mem_if_s.dc_stall;
  assign dcpu_if_s.dc_err        = 1'b0;
  assign dcpu_if_s.dc_flush_done = 1'b0;

  // ── CGU ─────────────────────────────────────────────────────────
  as_cgu_top #(cgu_addr_width) cgu (
    .clk_i(clk_i), .rst_i(rst_i),
    .wbdAddr_i(dBusAddr_periph_s[cgu_addr_width-1:0]),
    .wbdDat_i(dBusDataWr_periph_s),
    .wbdDat_o(dBusDataRdCgu_s),
    .wbdWe_i(wbdwe_s),
    .wbdSel_i(sel_s),
    .wbdStb_i(wbdstbCgu_s),
    .wbdAck_o(),
    .wbdCyc_i(is_periph_req_s),
    .clk_bus1_o( /* unused: clk_bus1_s tied to clk_div_s below */ ),
    .clk_bus2_o( /* unused: clk_bus2_s tied to clk_div_s below */ ),
    .clk_qspi_o( /* unused: clk_qspi_s tied to clk_div_s below */ ),
    .clk_core_o(clk_core_s)
  );

  // ── GPIO ─────────────────────────────────────────────────────────
  as_gpio_top #(gpio_addr_width, reg_width) asGpio (
    .rst_i(rst_i), .clk_i(clk_div_s),
    .wbdAddr_i(dBusAddr_periph_s[gpio_addr_width-1:0]),
    .wbdDat_i(dBusDataWr_periph_s),
    .wbdDat_o(dBusDataRdGpio_s),
    .wbdWe_i(wbdwe_s),
    .wbdSel_i(sel_s),
    .wbdStb_i(wbdstbGpio_s),
    .wbdAck_o(),
    .wbdCyc_i(is_periph_req_s),
    .gpio_irq_o(irq_gpiox_s),
    .gpio_io(gpio_io),
    .cs_o(asGpioCs_s)
  );

  // ── QSPI controller ──────────────────────────────────────────────
  // QSPI peripheral WB address base: 0x1_0000_0200 (csx[2])
  // Register offsets 0x00–0x88 fit in 8-bit address (256 > 136 = OFF_STATUS).
  as_qspi_top #(
    .QSPI_ADDR_WIDTH(QSPI_AW),
    .QSPI_DATA_WIDTH(reg_width),
    .FIFO_DEPTH(16)
  ) qspi_ctrl (
    .clk_i      (clk_qspi_s),
    .rst_i      (rst_i),
    .wbdAddr_i  (dBusAddr_periph_s[QSPI_AW-1:0]),
    .wbdDat_i   (dBusDataWr_periph_s),
    .wbdDat_o   (dBusDataRdQspi_s),
    .wbdWe_i    (wbdwe_s),
    .wbdSel_i   (sel_s),
    .wbdStb_i   (wbdstbQspi_s),
    .wbdAck_o   (),
    .wbdCyc_i   (is_periph_req_s),
    .axi4_if    (qspi_if_s),
    .sck_o      (sck_o),
    .cs_o       (flash_cs_o),
    .data_io    (flash_data_io),
    .qspi_irq_o (qspi_irq_s)
  );

  // ── IRQ ──────────────────────────────────────────────────────────
  assign irq_external_s[7]   = irq_gpiox_s;
  assign irq_external_s[6]   = qspi_irq_s;
  assign irq_external_s[5:0] = 6'b0;

  // ── JTAG ─────────────────────────────────────────────────────────
  assign bs_tdo_s = 1'b0;
  assign im_tdo_s = 1'b0;   // no I-Mem scan chain in cache version

  jtag as_jtag (
    .tck_i(tck_i), .trst_i(trst_i), .tms_i(tms_i), .tdi_i(tdi_i), .tdo_o(tdo_o),
    .tap_rst_o(tap_rst_s),
    .dr_cap_o(dr_cap_s),
    .sc01_tdo_i(sc01_tdo_s), .sc01_tdi_o(sc01_tdi_s),
    .sc01_shift_o(sc01_shift_s), .sc01_clock_o(sc01_clock_s),
    .im_tdo_i(im_tdo_s),   .im_tdi_o(im_tdi_s),
    .im_shift_o(im_shift_s), .im_clock_o(im_clock_s),
    .im_upd_o(im_upd_s),   .im_mode_o(im_mode_s),
    .bs_tdo_i(bs_tdo_s),   .bs_tdi_o(bs_tdi_s),
    .bs_shift_o(bs_shift_s), .bs_clock_o(bs_clock_s),
    .bs_upd_o(bs_upd_s),   .bs_mode_o(bs_mode_s)
  );

  // ── CPU ──────────────────────────────────────────────────────────
  as_cpux cpu (
    .clk_i(clk_div_s), .rst_i(rst_i), .tck_i(tck_i),
    .ir_o(),
    .dr_cap_i(dr_cap_s),
    .sc01_tdo_o(sc01_tdo_s), .sc01_tdi_i(sc01_tdi_s),
    .sc01_shift_i(sc01_shift_s), .sc01_clock_i(sc01_clock_s),
    .icpu_if(icpu_if_s),
    .dcpu_if(dcpu_if_s),
    .irq_ext_i(irq_external_s)
  );

  // ── Memory subsystem ─────────────────────────────────────────────
  // SP_BASE=0: scratchpad covers 0x0000_0000–0x0000_1FFF = same as old D-Mem
  asMemTop #(
    .SP_BASE  (32'h0000_0000),
    .SP_DEPTH (1024)
  ) memtop (
    .clk_i  (clk_div_s),
    .rst_i  (rst_i),
    .icpu_if(icpu_if_s),
    .dcpu_if(dcpu_mem_if_s),
    .qspi_if(qspi_if_s)
  );

endmodule : as_top_mem
