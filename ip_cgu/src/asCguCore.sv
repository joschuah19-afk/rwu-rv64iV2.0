
// asCguCore.sv
`timescale 1ns/1ps

import as_pack::*;

module as_cgucore (input  logic clk_i, // external clock (Zybo: 125 MHz)
                   input  logic rst_i,
                   output logic clk_bus1_o,
                   output logic clk_bus2_o,
                   output logic clk_qspi_o,
                   output logic clk_core_o);
  int cnt1_r,cnt2_r,cnt3_r,cnt4_r;

  // core clock
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
    begin
      cnt1_r <= 0;
    end
    else
    begin
      if( cnt1_r >= (clk_core_div-1) )
        cnt1_r <= 0;
      else
	cnt1_r <= cnt1_r + 1;
    end
  end // always_ff @ (posedge clk_i, posedge rst_i)
  
  assign clk_core_o  = (cnt1_r < (clk_core_div/2)) ? 1 : 0;

  // qspi clock
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
    begin
      cnt2_r <= 0;
    end
    else
    begin
      if( cnt2_r >= (clk_qspi_div-1) )
        cnt2_r <= 0;
      else
	cnt2_r <= cnt2_r + 1;
    end
  end // always_ff @ (posedge clk_i, posedge rst_i)
  
  assign clk_qspi_o  = (cnt2_r < ((clk_qspi_div-1)/2)) ? 1 : 0;

  // bus1 clock
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
    begin
      cnt3_r <= 0;
    end
    else
    begin
      if( cnt3_r >= (clk_bus1_div-1) )
        cnt3_r <= 0;
      else
	cnt3_r <= cnt3_r + 1;
    end
  end // always_ff @ (posedge clk_i, posedge rst_i)
  
  assign clk_bus1_o  = (cnt3_r < (clk_bus1_div/2)) ? 1 : 0;

  // bus2 clock
  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
    begin
      cnt4_r <= 0;
    end
    else
    begin
      if( cnt4_r >= (clk_bus2_div-1) )
        cnt4_r <= 0;
      else
	cnt4_r <= cnt4_r + 1;
    end
  end // always_ff @ (posedge clk_i, posedge rst_i)
  
  assign clk_bus2_o  = (cnt4_r < (clk_bus2_div/2)) ? 1 : 0;
  
endmodule : as_cgucore


