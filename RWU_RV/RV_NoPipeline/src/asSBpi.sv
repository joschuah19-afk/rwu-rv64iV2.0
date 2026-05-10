
// asSBpi.sv - Wishbone Classic Compliant Slave BPI
// Simple single read/write transfers, no extra clock cycles
// Spec: Wishbone B4 Classic

`timescale 1ns/1ps

import as_pack::*;

//-----------------------------------------------
// Wishbone slave BPI - Spec Compliant
// - Implements Wishbone Classic Single Read/Write
// - Zero wait states (combinatorial response)
// - One clock cycle per transfer with ACK
// - Call: as_slave_bpi #(64,64) myBpi ( all ports );
//-----------------------------------------------
module as_slave_bpi #( parameter addr_width = 64,
                       parameter data_width = 64 )  // passed by callers; not used internally (uses reg_width from package)
                     ( input  logic                  rst_i,
                       input  logic                  clk_i,
                       // kernel side
                       output logic [addr_width-1:0] addr_o,
                       input  logic [reg_width-1:0]  dat_from_core_i,
                       output logic [reg_width-1:0]  dat_to_core_o,
                       output logic                  wr_o,
                       output logic                  rd_o,
                       // wishbone side
                       input  logic [addr_width-1:0] wb_s_addr_i,
                       input  logic [reg_width-1:0]  wb_s_dat_i,
                       output logic [reg_width-1:0]  wb_s_dat_o,
                       input  logic                  wb_s_we_i,
                       input  logic [wbdSel-1:0]     wb_s_sel_i, // which byte is valid
                       input  logic                  wb_s_stb_i, // valid cycle
                       output logic                  wb_s_ack_o, // normal transaction
                       input  logic                  wb_s_cyc_i  // high for complete bus cycle
                     );

  //===========================================
  // Wishbone Classic Protocol Implementation
  //===========================================
  
  // Valid transaction when CYC and STB are both asserted
  logic valid_transfer_s;
  assign valid_transfer_s = wb_s_cyc_i & wb_s_stb_i;
  
  // Check if all SEL bits are set (full word transfer)
  logic all_sel_s;
  assign all_sel_s = &wb_s_sel_i;
  
  //===========================================
  // To Core/Peripheral
  //===========================================
  
  // Address is passed directly
  assign addr_o = wb_s_addr_i;
  
  // Write data is passed directly
  assign dat_to_core_o = wb_s_dat_i;
  
  // Write enable: valid transfer + WE + all bytes selected
  assign wr_o = valid_transfer_s & wb_s_we_i & all_sel_s;
  
  // Read enable: valid transfer + not WE + all bytes selected
  assign rd_o = valid_transfer_s & ~wb_s_we_i & all_sel_s;
  
  //===========================================
  // From Core/Peripheral
  //===========================================
  
  // Read data directly from peripheral
  assign wb_s_dat_o = dat_from_core_i;
  
  // ACK: Immediate response for valid transfer with all bytes selected
  // For zero wait-state operation, ACK is combinatorial
  assign wb_s_ack_o = valid_transfer_s & all_sel_s;
  
  //===========================================
  // Timing Explanation
  //===========================================
  /* 
   * Wishbone Classic Slave Response:
   * 
   * Clock:    __|‾‾|__|‾‾|__|‾‾|__
   *           
   * CYC_I:    ____‾‾‾‾‾‾‾‾‾‾‾‾____
   * STB_I:    ____‾‾‾‾‾‾‾‾‾‾‾‾____
   * ADR_I:    ====< ADDR >========
   * DAT_I:    ====< WDAT >========  (write)
   * WE_I:     ____‾‾‾‾‾‾‾‾‾‾‾‾____  (write) or LOW (read)
   * SEL_I:    ____‾‾‾‾‾‾‾‾‾‾‾‾____
   * 
   * WR_O:     ____‾‾‾‾____         (to peripheral, write enable)
   * RD_O:     ________‾‾‾‾____     (to peripheral, read enable)
   * 
   * ACK_O:    ____‾‾‾‾‾‾‾‾____     (to master, immediate)
   * DAT_O:    ========< RDAT >     (read data from peripheral)
   * 
   * Zero wait-state operation:
   * - Slave decodes STB & CYC combinatorially
   * - Peripheral must provide read data combinatorially
   * - ACK asserted in same cycle as STB
   * 
   * For registered peripherals:
   * - Read data should be registered for timing
   * - ACK can be delayed (insert wait states)
   * - This simple BPI assumes combinatorial peripherals
   */


endmodule : as_slave_bpi

