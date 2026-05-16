`timescale 1ns/1ps

import as_pack::*;

module tb_rv64i ();
  parameter tclk_2_t = 20;
  parameter clk_2_t  = 5;
  parameter clk_80_t = 400;

  logic clk_s, clk_core_s, clk_div_s;
  logic rst_s;
  logic tck_s, trst_s, tms_s, tdi_s, tdo_s;
  tri [nr_gpios-1:0] gpio_s;
  logic              cs_s;
  int fd;

  localparam int FLASH_WORDS = 16384;
  logic [31:0] flash_mem_s [0:FLASH_WORDS-1];
  initial $readmemh("riscvtest.mem", flash_mem_s);

  logic       sck_s;
  logic       flash_cs_s;
  wire  [3:0] flash_data_s;
  logic [3:0] flash_drive_s = 4'b0;
  logic       flash_oe_s    = 1'b0;
  assign flash_data_s = flash_oe_s ? flash_drive_s : 4'bzzzz;

  as_top_mem DUT (
    .clk_i(clk_s), .rst_i(rst_s),
    .tck_i(tck_s), .trst_i(trst_s), .tms_i(tms_s), .tdi_i(tdi_s), .tdo_o(tdo_s),
    .gpio_io(gpio_s),
    .cs_o(cs_s),
    .sck_o(sck_s),
    .flash_cs_o(flash_cs_s),
    .flash_data_io(flash_data_s),
    .clk_div_o(clk_div_s)
  );

  initial begin rst_s <= 1; #(10*2*clk_2_t); rst_s <= 0; end
  initial begin fd = $fopen("./error.txt", "a"); end
  always begin clk_s <= 1; #clk_2_t; clk_s <= 0; #clk_2_t; end
  always begin clk_core_s <= 1; #clk_80_t; clk_core_s <= 0; #clk_80_t; end
  initial begin tck_s <= 0; tms_s <= 0; tdi_s <= 0; trst_s <= 1; end

  initial begin #2000000000; $display("WATCHDOG: 2ms timeout"); $finish; end

  //------------------------------------------
  // QSPI NOR flash model (W25Q-style, Quad Output Fast Read 0x6B)
  // Sequence per CS# cycle (cmd=0x6B, quad=1, 24-bit addr, 8 dummy cycles):
  //   8 SCK posedges  : CMD on io[0]   (single, MSB first)
  //   6 SCK posedges  : ADDR on io[3:0] (quad,   MSB first → 24-bit)
  //   8 SCK posedges  : DUMMY
  //  16 SCK negedges  : DATA driven on io[3:0] (quad, MSB first → 64-bit)
  // The AXI4 FSM in as_qspi_top issues 4 separate CS# cycles per 32-byte cache line.
  //------------------------------------------
  always @(negedge flash_cs_s) begin flash_oe_s = 1'b0; flash_drive_s = 4'b0; end
  always begin
    @(posedge flash_cs_s);
    begin
      automatic logic [23:0] faddr = '0;
      automatic logic [63:0] fword = '0;
      automatic int          widx  = 0;
      automatic int          cnt   = 0;
      automatic int          idx   = 0;
      flash_oe_s    = 1'b0;
      flash_drive_s = 4'b0;
      while (flash_cs_s) begin
        @(posedge sck_s); if (!flash_cs_s) break;
        cnt++;
        if (cnt >= 9 && cnt <= 14)
          faddr = {faddr[19:0], flash_data_s[3:0]};
        if (cnt >= 22 && idx < 16) begin
          @(negedge sck_s); if (!flash_cs_s) break;
          if (idx == 0) begin
            widx  = int'(faddr) >> 2;
            fword = {flash_mem_s[widx+1], flash_mem_s[widx]};
          end
          flash_oe_s    = 1'b1;
          flash_drive_s = fword[63:60];
          fword         = {fword[59:0], 4'b0};
          idx++;
          if (idx == 16) begin @(posedge sck_s); break; end
        end
      end
      flash_oe_s    = 1'b0;
      flash_drive_s = 4'b0;
    end
  end

  always @(negedge clk_core_s) begin
    if(cs_s === 1) begin
      $display("CS detected");
      // direction[7]=1 → gpio_io[7]=Z; casez treats Z in expression as don't-care
      casez(gpio_s)
          8'b?000_0001 : begin $display("GPIO ID Read Passed!: 0x%0h", gpio_s);
                        $display("Simulation readgpioid succeeded"); #100; #(1*2*clk_2_t);
                        $fdisplay(fd,"%s - readgpioid: Test ok", get_time());
                        $fclose(fd); $stop; end
          default : begin $display("Unexpected GPIO: 0x%0h", gpio_s);
                          $fdisplay(fd,"%s - readgpioid: Test fail", get_time());
                          $fclose(fd); $stop; end
      endcase
    end
  end
  function string get_time();
    int file_pointer;
    void'($system("date +%x > sys_time"));
    file_pointer = $fopen("sys_time","r");
    void'($fscanf(file_pointer,"%s",get_time));
    $fclose(file_pointer);
    void'($system("rm sys_time"));
  endfunction

endmodule : tb_rv64i
