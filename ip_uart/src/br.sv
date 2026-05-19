`timescale 1ns/1ps

import as_pack::*;

module as_br (input  logic clk_i,
              input  logic rst_i,
              input  logic start_i,
              output logic br_o,
              output logic br2_o
             );
  int cnt_r;

  always_ff @(posedge clk_i, posedge rst_i)
  begin
    if(rst_i == 1)
    begin
      cnt_r <= 0;
    end
    else
    begin
      if( (cnt_r >= br_cnt_max) | (start_i == 1) )
        cnt_r <= 0;
      else
	cnt_r <= cnt_r + 1;
    end
  end // always_ff @ (posedge clk_i, posedge rst_i)

  assign br_o  = (cnt_r == 0) ? 1 : 0;
  assign br2_o = (cnt_r == br2_cnt_max) ? 1 : 0;  

endmodule : as_br

