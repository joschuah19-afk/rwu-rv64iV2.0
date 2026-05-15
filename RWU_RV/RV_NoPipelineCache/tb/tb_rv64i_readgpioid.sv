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

  logic [3:0]  qspi_arid_s;
  logic [31:0] qspi_araddr_s;
  logic [7:0]  qspi_arlen_s;
  logic [2:0]  qspi_arsize_s;
  logic [1:0]  qspi_arburst_s;
  logic        qspi_arvalid_s;
  logic        qspi_arready_s;
  logic [3:0]  qspi_rid_s;
  logic [63:0] qspi_rdata_s;
  logic [1:0]  qspi_rresp_s;
  logic        qspi_rlast_s;
  logic        qspi_rvalid_s;
  logic        qspi_rready_s;

  initial begin
    qspi_arready_s = 1'b0;
    qspi_rid_s     = '0;
    qspi_rdata_s   = '0;
    qspi_rresp_s   = 2'b00;
    qspi_rlast_s   = 1'b0;
    qspi_rvalid_s  = 1'b0;
  end

  as_top_mem DUT (
    .clk_i(clk_s), .rst_i(rst_s),
    .tck_i(tck_s), .trst_i(trst_s), .tms_i(tms_s), .tdi_i(tdi_s), .tdo_o(tdo_s),
    .gpio_io(gpio_s),
    .cs_o(cs_s),
    .qspi_arid_o(qspi_arid_s),   .qspi_araddr_o(qspi_araddr_s),
    .qspi_arlen_o(qspi_arlen_s), .qspi_arsize_o(qspi_arsize_s),
    .qspi_arburst_o(qspi_arburst_s), .qspi_arvalid_o(qspi_arvalid_s),
    .qspi_arready_i(qspi_arready_s),
    .qspi_rid_i(qspi_rid_s),     .qspi_rdata_i(qspi_rdata_s),
    .qspi_rresp_i(qspi_rresp_s), .qspi_rlast_i(qspi_rlast_s),
    .qspi_rvalid_i(qspi_rvalid_s), .qspi_rready_o(qspi_rready_s),
    .clk_div_o(clk_div_s)
  );

  initial begin rst_s <= 1; #(10*2*clk_2_t); rst_s <= 0; end
  initial begin fd = $fopen("./error.txt", "a"); end
  always begin clk_s <= 1; #clk_2_t; clk_s <= 0; #clk_2_t; end
  always begin clk_core_s <= 1; #clk_80_t; clk_core_s <= 0; #clk_80_t; end
  initial begin tck_s <= 0; tms_s <= 0; tdi_s <= 0; trst_s <= 1; end

  initial begin #2000000000; $display("WATCHDOG: 2ms timeout"); $finish; end

  //------------------------------------------
  // AXI4 slave: serves cache-line fill bursts from flash_mem_s
  // CDC: DUT master at clk_div_s (slow); slave clocked by clk_s gated on slow rising edge
  //------------------------------------------
  typedef enum logic [1:0] {AXI_IDLE, AXI_AR_HOLD, AXI_RDATA} axi_st_t;
  axi_st_t     axi_st_r;
  logic [31:0] axi_araddr_r;
  logic [7:0]  axi_arlen_r;
  logic [3:0]  axi_arid_r;
  logic [7:0]  axi_beat_r;

  // Detect slow-clock rising edge by sampling clk_div_s at fast-clock rate
  logic clk_div_d_s;
  logic clk_div_rise_s;
  always_ff @(posedge clk_s) clk_div_d_s <= clk_div_s;
  assign clk_div_rise_s = clk_div_s & ~clk_div_d_s;

  always_ff @(posedge clk_s, posedge rst_s) begin
    if (rst_s) begin
      axi_st_r       <= AXI_IDLE;
      qspi_arready_s <= 1'b0;
      qspi_rvalid_s  <= 1'b0;
      qspi_rlast_s   <= 1'b0;
      qspi_rid_s     <= '0;
      qspi_rdata_s   <= '0;
      qspi_rresp_s   <= 2'b00;
    end else if (clk_div_rise_s) begin
      case (axi_st_r)
        AXI_IDLE: begin
          qspi_arready_s <= 1'b0;
          qspi_rvalid_s  <= 1'b0;
          if (qspi_arvalid_s) begin
            axi_araddr_r   <= qspi_araddr_s;
            axi_arlen_r    <= qspi_arlen_s;
            axi_arid_r     <= qspi_arid_s;
            axi_beat_r     <= 8'd0;
            qspi_arready_s <= 1'b1;
            axi_st_r       <= AXI_AR_HOLD;
          end
        end
        AXI_AR_HOLD: begin
          // Hold arready until master drops arvalid (registered on master's slow clock)
          if (!qspi_arvalid_s) begin
            qspi_arready_s <= 1'b0;
            qspi_rid_s     <= axi_arid_r;
            qspi_rresp_s   <= 2'b00;
            begin
              automatic int widx = int'(axi_araddr_r >> 2);
              qspi_rdata_s <= {flash_mem_s[widx+1], flash_mem_s[widx]};
            end
            qspi_rlast_s  <= (8'd0 == axi_arlen_r);
            qspi_rvalid_s <= 1'b1;
            axi_st_r      <= AXI_RDATA;
          end

        end
        AXI_RDATA: begin
          qspi_arready_s <= 1'b0;
          qspi_rvalid_s  <= 1'b1;
          qspi_rid_s     <= axi_arid_r;
          qspi_rresp_s   <= 2'b00;
          if (axi_beat_r == axi_arlen_r) begin
            axi_st_r      <= AXI_IDLE;
            qspi_rvalid_s <= 1'b0;
            qspi_rlast_s  <= 1'b0;
          end else if (qspi_rready_s) begin
            axi_beat_r   <= axi_beat_r + 8'd1;
            axi_araddr_r <= axi_araddr_r + 32'd8;
            begin
              automatic int widx = int'((axi_araddr_r + 32'd8) >> 2);
              qspi_rdata_s <= {flash_mem_s[widx+1], flash_mem_s[widx]};
            end
            qspi_rlast_s <= ((axi_beat_r + 8'd1) == axi_arlen_r);
          end
        end
      endcase
    end
  end

  always @(negedge clk_core_s) begin
    if(cs_s === 1) begin
      $display("CS detected");
      case(gpio_s)
          1, 129   : begin $display("GPIO ID Read Passed!: 0x%0h", gpio_s);
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
