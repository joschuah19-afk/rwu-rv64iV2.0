
// asTopMem.sv
`timescale 1ns/1ps

import as_pack::*;

//--------------------------------------------
// 2Do:
// - Insert a dummy BS chain. It should consist of one I, one O and one I/O
// - Insert WB bus for instruction and for data.                                (Done)
// - Design WB master BPI and WB slave BPI. Implement WB.                       (Done)
// - Design a GPIO peripheral.                                                  (Done)
// - implement IRQs
//--------------------------------------------


module as_top_mem (input logic                       clk_i,
                  input  logic                       rst_i,
                  // JTAG
                  input  logic                       tck_i,      // Test Clock
                  input  logic                       trst_i,     // TAPC reset
                  input  logic                       tms_i,      // Test Mode Select
                  input  logic                       tdi_i,      // Test Data In
                  output logic                       tdo_o,      // Test Data Out
                  // GPIO
                  inout tri [nr_gpios-1:0]        gpio_io,     // data of data bus (write to dmem)
                  //output logic [gpio_addr_width-1:0] gpioAddr_o, // addr of data bus
                  output logic                       cs_o
                 );

  // data bus
  //logic [reg_width-1:0]   dBusDataRdDmemxxx_s; // data out from dmem
  logic [reg_width-1:0]   dBusDataRdDmem_s; // data out from dmem
  logic [reg_width-1:0]   dBusDataRdGpio_s; // data out from GPIO
  logic [reg_width-1:0]   dBusDataRdQspi_s; // data out from QSPI
  logic [reg_width-1:0]   dBusDataRdCgu_s;  // data out from CGU
  logic [reg_width-1:0]   dBusDataRd_s; // data out of MUX
  logic [reg_width-1:0]   dBusDataWr_s; // data in to dmem
  logic [daddr_width-1:0] dBusAddr_s;   // address for dmem
  //logic			  dMemRd_s;     // read enable for dmem
  //logic			  dMemWr_s;     // write enable for dmem
  // D-Mem MBPI - CPU
  logic [reg_width-1:0]   dBusDataRd3_s;
  logic [reg_width-1:0]	  dBusDataWr3_s;
  logic [daddr_width-1:0] dBusAddr3_s;
  logic                   dMemWr3_s;
  
  // instruction bus
  logic [instr_width-1:0] iBusDataRd_s; // data out from imem = Instruction
  logic [instr_width-1:0] iBusDataWr_s; // data in to imem            -- not connected
  logic [iaddr_width-1:0] iBusAddr_s;   // address for imem
  logic			  iMemWr_s;     // write enable for imem      -- not connected
  logic [instr_width-1:0] ir_s;
  
  logic [iaddr_width-1:0] iBusAddr3_s;
  logic [instr_width-1:0] iBusDataRd3_s;
  

  // JTAG
  logic tap_rst_s;
  logic	sc01_tdo_s, sc01_tdi_s, sc01_shift_s, sc01_clock_s;
  logic im_tdo_s, im_tdi_s, im_shift_s, im_clock_s, im_upd_s, im_upd_del_s, im_mode_s;
  logic bs_tdo_s, bs_tdi_s, bs_shift_s, bs_clock_s, bs_upd_s, bs_mode_s;
  // JTAG: I-Mem scan chain
  logic [im_scan_length-1:0] im_datai_s, im_datao_s;
  // JTAG: I-Mem Address MUX
  logic [iaddr_width-1:0] iBusAddr2_s;   // address for imem
  logic [iaddr_width-1:0] im_addr_s;   // address for imem from chain
  logic			  mux_sel_s;
  logic [6:0]		  opcode_s;
  logic [2:0]		  func3_s;

  logic wbdwe_s;
  logic wbdstb1_s;
  logic wbdstbGpio_s;
  logic wbdstbCgu_s;
  logic wbdcyc_s;
  logic [wbdSel-1:0]          sel_s;
  //tri	[nr_gpios-1:0]        asGpio_s;
  //logic [gpio_addr_width-1:0] asGpioAdr_s;
  logic asGpioCs_s;
  logic	wdbAckGpio_s;
  logic	wdbAckQspi_s;
  logic	wdbAckCgu_s;

  logic	wbdstDMem_s;
  logic	wdbAckDmem_s;
  //logic	wdbAckDmemxxx_s;
  //logic [dmem_addr_width-1:0] adr_bpi_2_dmem_s;
  //logic [reg_width-1:0]       dat_dmem_2_bpi_s;
  //logic [reg_width-1:0]       dat_bpi_2_dmem_s;
  //logic	wr_bpi_2_dmem_s;

  logic	wdbAckAll_s;

  logic [chipsel-1:0] csx_s; // one bit per peripheral

  logic clk_core_s, clk_qspi_s, clk_bus1_s, clk_bus2_s;

  // arbiter
  logic	gnt0_s, wbdcycarb_s;

  // IRQs
  logic [irq_total_num_ext_c-1:0] irq_external_s;
  //logic	[nr_gpios-1:0]            irq_gpio_s;
  logic				  irq_gpiox_s;
  //logic				  stall_ir_s;

  logic				  dr_cap_s;
  logic				  clk_div_s;
  
  
  
  assign cs_o        = asGpioCs_s;

  //--------------------------------------------
  // Memory Map Data Bus:
  // - see package
  //--------------------------------------------

  //--------------------------------------------
  // Address Decoder
  //--------------------------------------------
  as_decode addressDecode (dBusAddr_s, csx_s);
  
  //--------------------------------------------
  // Data Mux: Peripherals to Master
  //--------------------------------------------
  assign dBusDataRdQspi_s = {reg_width{1'b0}}; // xxxxxxxxxxxxxxxxxxxxxxxxxxxxx remove when QSPI implemented
  always_comb 
  begin
    case(csx_s) // one hot code
      0  :       dBusDataRd_s = {reg_width{1'b0}};
      1  :       dBusDataRd_s = dBusDataRdDmem_s;
      2  :       dBusDataRd_s = dBusDataRdGpio_s;
      4  :       dBusDataRd_s = dBusDataRdQspi_s;
      8  :       dBusDataRd_s = dBusDataRdCgu_s;
      default:   dBusDataRd_s = {reg_width{1'b0}};
    endcase
  end // always_comb
  
  //--------------------------------------------
  // GPIO Peripheral
  //--------------------------------------------
  assign wbdstbGpio_s = wbdstb1_s & wbdcycarb_s & csx_s[1];

  as_gpio_top #(gpio_addr_width,reg_width) 
                            asGpio(.rst_i(rst_i),
                                   .clk_i(clk_div_s),
                                   .wbdAddr_i(dBusAddr_s[gpio_addr_width-1:0]),
                                   .wbdDat_i(dBusDataWr_s),
                                   .wbdDat_o(dBusDataRdGpio_s), 
                                   .wbdWe_i(wbdwe_s),
                                   .wbdSel_i(sel_s),
                                   .wbdStb_i(wbdstbGpio_s),
                                   .wbdAck_o(wdbAckGpio_s),
                                   .wbdCyc_i(wbdcycarb_s),
				   .gpio_irq_o(irq_gpiox_s),
                                   .gpio_io(gpio_io), //asGpio_s
                                   .cs_o(asGpioCs_s)
                                  );

  //--------------------------------------------
  // CGU
  //--------------------------------------------
  assign wbdstbCgu_s = wbdstb1_s & wbdcycarb_s & csx_s[3];

  as_cgu_top #(cgu_addr_width) 
         cgu (.clk_i(clk_i),
              .rst_i(rst_i),
              .wbdAddr_i(dBusAddr_s[cgu_addr_width-1:0]),
              .wbdDat_i(dBusDataWr_s),
              .wbdDat_o(dBusDataRdCgu_s),
              .wbdWe_i(wbdwe_s),
              .wbdSel_i(sel_s),
              .wbdStb_i(wbdstbCgu_s),
              .wbdAck_o(wdbAckCgu_s),
              .wbdCyc_i(wbdcycarb_s),
	      .clk_bus1_o(clk_bus1_s),
              .clk_bus2_o(clk_bus2_s),
              .clk_qspi_o(clk_qspi_s),
              .clk_core_o(clk_core_s)
             );
  
  assign clk_div_s = (dr_cap_s) ? clk_i : clk_core_s; // During scan-test, the core clock must be controllable from the tester
  
  //--------------------------------------------
  // CPU
  //--------------------------------------------
  assign irq_external_s[7]   = irq_gpiox_s;
  assign irq_external_s[6:0] = 7'b0;
  
  assign wdbAckQspi_s = 1'b0;
  
  assign wdbAckAll_s = (wdbAckDmem_s || wdbAckGpio_s || wdbAckQspi_s || wdbAckCgu_s) && gnt0_s;
  
  as_cpux cpu (.clk_i(clk_div_s), // divided functional clock or external TB clock (scan)
              .rst_i(rst_i),
              .tck_i(tck_i),
              .ir_o(ir_s),
              .dr_cap_i(dr_cap_s),
	       // scan path
              .sc01_tdo_o(sc01_tdo_s),
              .sc01_tdi_i(sc01_tdi_s),
              .sc01_shift_i(sc01_shift_s),
              .sc01_clock_i(sc01_clock_s),
	       // instruction bus
              .iBusAddr_o(iBusAddr3_s),
              .iBusDataRd_i(iBusDataRd3_s),
	       // data bus
              .dBusAddr_o(dBusAddr3_s),
              .dBusDataWr_o(dBusDataWr3_s),
              .dBusDataRd_i(dBusDataRd3_s),
              .dBusWe_o(dMemWr3_s),
	       // IRQ
              .irq_ext_i(irq_external_s)
             );

  //--------------------------------------------
  // Arbiter
  //--------------------------------------------
  assign wbdcycarb_s = wbdcyc_s;
  assign gnt0_s      = wbdcyc_s;

  //--------------------------------------------
  // Instruction memory
  //--------------------------------------------
  // M-BPI
  as_master_bpi #(64, 32) mInstrBpi(
                                   .rst_i(rst_i),
                                   .clk_i(clk_i),
                                   .addr_i(iBusAddr3_s),          // Address: between BPI and CPU
                                   .dat_from_core_i('b0),         // not connected
                                   .dat_to_core_o(iBusDataRd3_s), // Data (Instruction): between BPI and CPU
                                   .wr_i(1'b0),                   // not connected
                                   .wb_m_addr_o(iBusAddr_s),      // Address: between BPI and I-Mem
                                   .wb_m_dat_i(iBusDataRd_s),     // Data (Instruction): between BPI and I-Mem
                                   .wb_m_dat_o(),                 // not connected
                                   .wb_m_we_o(),                  // not connected
                                   .wb_m_sel_o(),                 // not connected
                                   .wb_m_stb_o(),                 // not connected
                                   .wb_m_ack_i(1'b1),             // not connected
                                   .wb_m_cyc_o()                  // not connected
                                  );
  // 2Do: I-Mem needs a slave BPI.
  
  // delay of im_upd
  always_ff @(posedge tck_i, posedge tap_rst_s) 
  begin
    if(tap_rst_s == 1)
    begin
      im_upd_del_s <= 1'b0;
    end
    else
    begin
      im_upd_del_s <= im_upd_s; // nonblocking assignment
    end
  end
  
  assign im_addr_s    = im_datao_s[im_scan_length-1:instr_width+1]; // address part of scan chain
  assign iMemWr_s     = im_datao_s[0] & im_upd_del_s;               // wr_e is the lsb of the scan chain AND im_upd_del_s
  assign iBusDataWr_s = im_datao_s[instr_width:1];                  // data part of scan chain
  assign mux_sel_s    = im_datao_s[0] & im_upd_del_s;               // address mux select; activated when we is on
  assign iBusAddr2_s = (mux_sel_s == 0) ? iBusAddr_s[imem_addr_width-1:0] : im_addr_s; // address is either PC or scan chain
  as_imem imem (.tck_i(tck_i),                             // writing to I-Mem with TCK (tck_i, clk_core_s)
                .clk_i(clk_div_s),
                .addr_i(iBusAddr2_s[imem_addr_width-1:0]), // PC or scan-chain
                .data_i(iBusDataWr_s),                     // scan-chain
                .wr_i(iMemWr_s),                           // scan-chain
                .data_o(iBusDataRd_s)                      // Instruction
               );

  //--------------------------------------------
  // data memory
  //--------------------------------------------
  // M-BPI
  as_master_bpi #(64, 64) mDataBpi(
                                   .rst_i(rst_i),
                                   .clk_i(clk_i),
                                   .addr_i(dBusAddr3_s),            // X Address to D-Mem (Logic - BPI)
                                   .dat_from_core_i(dBusDataWr3_s), // X Data to D-Mem (Logic - BPI)
                                   .dat_to_core_o(dBusDataRd3_s),   // X Data from D-Mem (BPI - Logic)
                                   .wr_i(dMemWr3_s),// X Write enable (= not read)
                                   .wb_m_addr_o(dBusAddr_s),     // X Address to D-Mem (BPI - Mem)
                                   .wb_m_dat_i(dBusDataRd_s),    // X Data from Mem (Mem - BPI)
                                   .wb_m_dat_o(dBusDataWr_s),    // X Data to D-Mem (BPI - Mem)
                                   .wb_m_we_o(wbdwe_s),
                                   .wb_m_sel_o(sel_s),
                                   .wb_m_stb_o(wbdstb1_s),
                                   .wb_m_ack_i(wdbAckAll_s),
                                   .wb_m_cyc_o(wbdcyc_s)
                                  );
  
  assign wbdstDMem_s = wbdstb1_s & wbdcycarb_s & csx_s[0];
  assign opcode_s    = ir_s[6:0];
  assign func3_s     = ir_s[14:12];

  as_dmem_top #(dmem_addr_width)
              DMem (.clk_i(clk_div_s),
                    .rst_i(rst_i),
                    .wbdAddr_i(dBusAddr_s[dmem_addr_width-1:0]),
                    .wbdDat_i(dBusDataWr_s),
                    .wbdDat_o(dBusDataRdDmem_s),
                    .wbdWe_i(wbdwe_s),
                    .wbdSel_i(sel_s),
                    .wbdStb_i(wbdstDMem_s),
                    .wbdAck_o(wdbAckDmem_s),
                    .wbdCyc_i(wbdcycarb_s),
                    .opcode_i(opcode_s), // iBusDataRd_s[6:0]
                    .func3_i(func3_s)    // iBusDataRd_s[14:12]
                   );
  
  //--------------------------------------------
  // JTAG
  //--------------------------------------------
  assign bs_tdo_s = 1'b0;
  
  jtag as_jtag (.tck_i(tck_i),
                .trst_i(trst_i),
                .tms_i(tms_i),
                .tdi_i(tdi_i),
                .tdo_o(tdo_o),
                .tap_rst_o(tap_rst_s),
                .dr_cap_o(dr_cap_s),
                .sc01_tdo_i(sc01_tdo_s),
                .sc01_tdi_o(sc01_tdi_s),
                .sc01_shift_o(sc01_shift_s),
                .sc01_clock_o(sc01_clock_s),
                .im_tdo_i(im_tdo_s),
                .im_tdi_o(im_tdi_s),
                .im_shift_o(im_shift_s),
                .im_clock_o(im_clock_s),
                .im_upd_o(im_upd_s),
                .im_mode_o(im_mode_s),
                .bs_tdo_i(bs_tdo_s),
                .bs_tdi_o(bs_tdi_s),
                .bs_shift_o(bs_shift_s),
                .bs_clock_o(bs_clock_s),
                .bs_upd_o(bs_upd_s),
                .bs_mode_o(bs_mode_s)
               );

  //--------------------------------------------
  // Scan chain for I-Mem
  //--------------------------------------------
  assign im_datai_s  = {im_scan_length{1'b0}}; // paralell load; all zero
  
  dr_reg #(.dr_width(im_scan_length)) 
       imem_load (.tck_i(tck_i),
                  .trst_i(tap_rst_s),
                  .mode_i(im_mode_s),
                  .dr_shift_i(im_shift_s),
                  .dr_clock_i(im_clock_s),
                  .dr_upd_i(im_upd_s),
                  .data_i(im_datai_s),// parallel in to chain
                  .ser_i(im_tdi_s),
                  .data_o(im_datao_s),// to I-Mem, parallel access of scan chain
                  .ser_o(im_tdo_s)
		 );
  
 
endmodule : as_top_mem

