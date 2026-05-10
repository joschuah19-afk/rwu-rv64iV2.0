
// as_pack.sv
`timescale 1ns/1ps

package as_pack;
  //typedef enum bit [1:0] {RED, YELLOW, GREEN, RDYEL} e_signal;

  //function common();
  //  $display("as Function.");
  //endfunction // common

  // typedef enum for opcode
  typedef enum logic [6:0]
  {
    OP_LOAD      = 7'b0000011, // 3,   I-type, loads
    OP_OP_IMM    = 7'b0010011, // 19,  I-type (ADDI, ORI, etc.)
    OP_AUIPC     = 7'b0010111, // 23,  U-type, auipc
    OP_OP_IMMW   = 7'b0011011, // 27,  I-type, addiw, slliw, ...
    OP_STORE     = 7'b0100011, // 35,  S-type, stores
    OP_OP        = 7'b0110011, // 51,  R-type (ADD, SUB, etc.)
    OP_LUI       = 7'b0110111, // 55,  U-type, lui
    OP_OPW       = 7'b0111011, // 59,  R-type, addw, subw, ...
    OP_BRANCH    = 7'b1100011, // 99,  B-type, branches
    OP_JALR      = 7'b1100111, // 103, I-type, jalr
    OP_JAL       = 7'b1101111, // 111, J-type, jal
    OP_SYSTEM    = 7'b1110011  // 115, I-type, CSRs, mret
  } opcode_t;

  // typedef enum for ALU
  typedef enum logic [5:0]
  {
    ALU_ADD,
    ALU_SUB,
    ALU_AND,
    ALU_OR,
    ALU_XOR,
    ALU_SLT,
    ALU_SLTU,
    ALU_SLL,
    ALU_SRL,
    ALU_SRA,
    ALU_ADDW,
    ALU_SUBW,
    ALU_SLLW,
    ALU_SRLW,
    ALU_SRAW
  } alu_op_t;

  typedef enum logic [2:0]
  {
    BR_NONE,
    BR_EQ,
    BR_NE,
    BR_LT,
    BR_GE,
    BR_LTU,
    BR_GEU
  } br_op_t;
  
  typedef enum logic [2:0]
  {
    IMM_NONE,   // keine Immediate (R-type)
    IMM_I,      // I-type (addi, lw, jalr, csr…)
    IMM_S,      // S-type (stores)
    IMM_B,      // B-type (branches)
    IMM_U,      // U-type (lui, auipc)
    IMM_J       // J-type (jal)
  } imm_src_t;

  typedef enum logic [1:0]
  {
    RES_ALU,    // ALU-Ergebnis
    RES_MEM,    // Load-Daten
    RES_PC4,    // PC + 4 (jal / jalr)
    RES_CSR     // CSR-Read
  } result_src_t;

  typedef enum logic [1:0]
  {
    PC_PLUS4,
    PC_BRANCH,
    PC_JUMP,
    PC_TRAP
  } pc_src_t;

  typedef enum logic [1:0]
  {
    SRC_REGA,
    SRC_PC,
    SRC_ZERO
  } mux_a_t;

  typedef struct packed
  {
    logic       addr_len;   // bit 11 → ctrl_reg_i[8]
    logic       cpol;       // bit 10 → ctrl_reg_i[7]
    logic       cpha;       // bit  9 → ctrl_reg_i[6]
    logic       dual;       // bit  8 → ctrl_reg_i[5]
    logic       cs_hold;    // bit  7 → ctrl_reg_i[4]
    logic       xip;        // bit  6 → ctrl_reg_i[3]
    logic       ddr;        // bit  5 → ctrl_reg_i[2]
    logic       quad;       // bit  4 → ctrl_reg_i[1]
  } qspi_ctrl_t;
  // Im Top/BPI, Zuweisung aus dem 64-bit CTRL-Register:
  // qspi_ctrl_t ctrl_s;
  // assign ctrl_s = qspi_ctrl_t'(ctrl_reg_r[11:4]);
  //                                        ^^^^^ Bit 11=addr_len ... Bit 3=start
  
  
  // general
  localparam int       reg_width        = 64; // = data width
  //localparam int       reg_2_width      = 32; // = word width  // unused (Verilator UNUSEDPARAM)
  localparam int       iaddr_width      = 64; // must be = reg_width
  localparam int       daddr_width      = 64;
  localparam int       instr_width      = 32;
  // controls
  //localparam int       alusel_width     = 4; // ALU according Hennessy Pat.  // unused (Verilator UNUSEDPARAM)
  //localparam int       aluselrv_width   = 5; // ALU according Harris           // unused (Verilator UNUSEDPARAM)
  //localparam int       dmuxsel_width    = 2;                                   // unused (Verilator UNUSEDPARAM)
  //localparam int       immsrc_width     = 3;                                   // unused (Verilator UNUSEDPARAM)
  //localparam int       aluop_width      = 2;                                   // unused (Verilator UNUSEDPARAM)
  //localparam int       controls01_width = 14; // asMainDec                     // unused (Verilator UNUSEDPARAM)
  // instruction fields
  //localparam int       func7_width      = 7;                                   // unused (Verilator UNUSEDPARAM)
  localparam int       func3_width      = 3;
  localparam int       opcode_width     = 7;
  // register file
  localparam int       rwaddr_width     = 5;
  localparam int       nr_regs          = 32;

  // memories & peripherals
  localparam int       dmemdepth        = 1024; // amount of double words (if reg_width = 64); 1024 doubles = 8192 bytes => addr_width = 13
  localparam int       dmem_addr_width  = 13;   // address for all bytes (8192 double words * 8 Bytes = 65536 Bytes => 16 bit address)
  localparam int       imemdepth        = 8192; // 12 bit address, but the lower 2 will not be used; word alligned
  localparam int       imem_addr_width  = 15;   // (8192 words accessible => 15 - 2 bits address)
  localparam int       cgu_addr_width   = 8;
  

  // external
  localparam int       nr_gpios         = 8; // 0 - 255
  localparam int       gpio_addr_width  = 8;
  //localparam int       cs_width         = 2;

  // tapc
  localparam int       ir_width = 8;
  //localparam int       dr1_width = 8;
  localparam int       id_width = 32;
  localparam int       nr_drs = 5; // BY, BS, I-Mem, Scan, USERCODE
  localparam int       im_addr_width = imem_addr_width; // #address lines
  localparam int       im_data_width = instr_width;     // #data lines
  localparam int       im_scan_length = im_addr_width + im_data_width + 1; // +1 for w_en; only writing

  localparam int       chipsel = 4;
  localparam int       wbdSel = 8;

  // CGU
  // Division factors: f_zybo = 125 MHz
  //                   div = 2:   f = 62.5 MHz
  //                   div = 4:   f = 31.25 MHz
  //                   div = 5:   f = 25 MHz
  //                   div = 25:  f = 5 MHz
  //                   div = 200: f = 625 kHz
  //                   div = 400: f = 312.5 kHz
  //localparam int       clk_zybo_per = 8;    // ns, f = 125 MHz  // unused (Verilator UNUSEDPARAM)
  //localparam int       clk_core_per = 1600; // ns, f = 625 kHz
  localparam int       clk_core_div = 80;
  //localparam int       clk_qspi_per = 200;  // ns, 20 Mbit/s -> f = 5 MHz -> div = 25
  localparam int       clk_qspi_div = 4;
  localparam int       clk_bus1_div = 80;
  localparam int       clk_bus2_div = 100;

  // Memory map
  //localparam logic [63:0]       dmem_start_address_c           = 64'h00000000_00000000; // byte address  // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       dmem_end_address_c             = 64'h00000000_0000FFFF;               // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       gpio_start_address_c           = 64'h00000001_00000000;               // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       gpio_end_address_c             = 64'h00000001_0000005F; // 12 regs     // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       qspi_start_address_c           = 64'h00000001_00000060;               // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       qspi_end_address_c             = 64'h00000001_000000BF; // 12 regs     // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       cgu_start_address_c            = 64'h00000001_000000C0;               // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       cgu_end_address_c              = 64'h00000001_0000011F; // 12 regs     // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       uart0_start_address_c          = 64'h00000001_00000120;               // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       uart0_end_address_c            = 64'h00000001_0000017F; // 12 regs     // unused (Verilator UNUSEDPARAM)

  // register addresses
  //localparam int       gpio_base_addr_c               = 64'h00000001_00000000; // byte address
  //localparam int       gpio_nr_regs_c                 = 2; // 8 bytes each
  //localparam int       gpio_end_addr_c                = gpio_base_addr_c + gpio_nr_regs_c*8 - 1;
  // GPIO
  localparam logic [63:0]       gpio_id_reg_addr_offs_c        =  0;
  localparam logic [63:0]       gpio_id_reg_addr_rst_c         = 64'h00000000_00000001;
  localparam logic [63:0]       gpio_direction_reg_addr_offs_c =  8;
  localparam logic [63:0]       gpio_direction_reg_addr_rst_c  = '0;
  localparam logic [63:0]       gpio_data_reg_addr_offs_c      = 16;
  localparam logic [63:0]       gpio_data_reg_addr_rst_c       = '0;
  localparam logic [63:0]       gpio_irqss_reg_addr_offs_c     = 24;
  localparam logic [63:0]       gpio_irqss_reg_rst_c           = '0;
  localparam logic [63:0]       gpio_irqsm_reg_addr_offs_c     = 32;
  localparam logic [63:0]       gpio_irqsm_reg_rst_c           =  '1;
  localparam logic [63:0]       gpio_irqsc_reg_addr_offs_c     = 40;
  localparam logic [63:0]       gpio_irqsc_reg_rst_c           = '0;
  localparam logic [63:0]       gpio_isr_reg_addr_offs_c       = 48;
  localparam logic [63:0]       gpio_isr_reg_rst_c             =  '0;
  localparam logic [63:0]       gpio_ris_reg_addr_offs_c       = 56;
  localparam logic [63:0]       gpio_ris_reg_rst_c             =  '0;
  localparam logic [63:0]       gpio_imsc_reg_addr_offs_c      = 64;
  localparam logic [63:0]       gpio_imsc_reg_rst_c            =  '1; // change to 0 before tape-out
  localparam logic [63:0]       gpio_mis_reg_addr_offs_c       = 72;
  localparam logic [63:0]       gpio_mis_reg_rst_c             =  '0;
  // QSPI  // unused (Verilator UNUSEDPARAM) — QSPI IP not yet instantiated in RV_NoPipeline
  //localparam logic [63:0]       qspi_id_reg_addr_offs_c        =  0;
  //localparam logic [63:0]       qspi_id_reg_addr_rst_c         = 64'h00000000_00000010;
  //localparam logic [63:0]       qspi_ctrl_reg_addr_offs_c      =  8; // check
  //localparam logic [63:0]       qspi_ctrl_reg_addr_rst_c       = '0; // check
  //localparam logic [63:0]       qspi_cmd_reg_addr_offs_c       = 16; // check
  //localparam logic [63:0]       qspi_cmd_reg_addr_rst_c        = '0; // check
  //localparam logic [63:0]       qspi_addr_reg_addr_offs_c      = 24; // check
  //localparam logic [63:0]       qspi_addr_reg_rst_c            = '0; // check
  //localparam logic [63:0]       qspi_len_reg_addr_offs_c       = 32; // check
  //localparam logic [63:0]       qspi_len_reg_rst_c             = '0; // check
  //localparam logic [63:0]       qspi_dummy_reg_addr_offs_c     = 40; // check
  //localparam logic [63:0]       qspi_dummy_reg_rst_c           = '0; // check
  //localparam logic [63:0]       qspi_clkdiv_reg_addr_offs_c    = 48; // check
  //localparam logic [63:0]       qspi_clkdiv_reg_rst_c          = '0; // check
  //localparam logic [63:0]       qspi_timeout_reg_addr_offs_c   = 56; // check
  //localparam logic [63:0]       qspi_timeout_reg_rst_c         = '0; // check
  //localparam logic [63:0]       qspi_isr_reg_addr_offs_c       = 64; // check
  //localparam logic [63:0]       qspi_isr_reg_rst_c             = '0;
  //localparam logic [63:0]       qspi_ris_reg_addr_offs_c       = 72; // check
  //localparam logic [63:0]       qspi_ris_reg_rst_c             = '0;
  //localparam logic [63:0]       qspi_imsc_reg_addr_offs_c      = 80; // check
  //localparam logic [63:0]       qspi_imsc_reg_rst_c            = '1;
  //localparam logic [63:0]       qspi_mis_reg_addr_offs_c       = 88; // check
  //localparam logic [63:0]       qspi_mis_reg_rst_c             = '0;
  //localparam logic [63:0]       qspi_icr_reg_addr_offs_c       = 96; // check
  //localparam logic [63:0]       qspi_icr_reg_rst_c             = '0;
  //localparam logic [63:0]       qspi_rx_reg_addr_offs_c        = 104; // check
  //localparam logic [63:0]       qspi_rx_reg_rst_c              = '0;
  //localparam logic [63:0]       qspi_tx_reg_addr_offs_c        = 112; // check
  //localparam logic [63:0]       qspi_tx_reg_rst_c              = '0;
  //localparam logic [63:0]       qspi_fifost_reg_addr_offs_c    = 120; // check
  //localparam logic [63:0]       qspi_fifost_reg_rst_c          = '0;
  //localparam logic [63:0]       qspi_xip_reg_addr_offs_c       = 128; // check
  //localparam logic [63:0]       qspi_xip_reg_rst_c             = 64'h00000000_000000A0;
  //localparam logic [63:0]       qspi_stat_reg_addr_offs_c      = 136; // check
  //localparam logic [63:0]       qspi_stat_reg_rst_c            = '0;

  localparam logic [63:0]       cgu_id_reg_addr_offs_c         =  0;
  localparam logic [63:0]       cgu_id_reg_addr_rst_c          = 64'h00000000_00000010;

  //localparam logic [63:0]       dmem_id_reg_addr_offs_c        =  0;                       // unused (Verilator UNUSEDPARAM)
  //localparam logic [63:0]       dmem_id_reg_addr_rst_c         = 64'h00000000_00000011;    // unused (Verilator UNUSEDPARAM)

  //IRQ
  //localparam int       irq_total_num_c                =  8;  // unused (Verilator UNUSEDPARAM)
  localparam int       irq_total_num_ext_c            =  8;
  
endpackage

