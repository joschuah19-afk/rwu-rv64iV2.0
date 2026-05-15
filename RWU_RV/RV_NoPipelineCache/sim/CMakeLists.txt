cmake_minimum_required(VERSION 3.20)
project(rv64i_cache_sim LANGUAGES NONE)

# -------------------------------------------------
# Required variables from top-level cmake
# -------------------------------------------------
foreach(var PROJECT_ROOT IP_ROOT NP_ROOT RV64_MEM TB_FILE)
    if(NOT DEFINED ${var})
        message(FATAL_ERROR "${var} not set")
    endif()
endforeach()

if(NOT EXISTS ${RV64_MEM})
    message(FATAL_ERROR "RV64_MEM does not exist: ${RV64_MEM}")
endif()

if(NOT EXISTS ${TB_FILE})
    message(FATAL_ERROR "TB_FILE does not exist: ${TB_FILE}")
endif()

if(NOT DEFINED SIMULATOR)
    set(SIMULATOR "xsim")
endif()

set(TB_TOP tb_rv64i)

# -------------------------------------------------
# SystemVerilog sources
# Shared RTL from RV_NoPipeline is referenced via NP_ROOT.
# Cache-specific RTL is in PROJECT_ROOT/src.
# -------------------------------------------------
set(SOURCES_SV
    # Package + shared RTL (unchanged from RV_NoPipeline)
    ${NP_ROOT}/src/as_pack.sv
    ${NP_ROOT}/src/asRegFile.sv
    ${NP_ROOT}/src/asAlu.sv
    ${NP_ROOT}/src/asAluBr.sv
    ${NP_ROOT}/src/asInstrDecode.sv
    ${NP_ROOT}/src/asDecode.sv
    ${NP_ROOT}/src/asSBpi.sv

    # JTAG IP
    ${IP_ROOT}/ip_jtag/src/tap_fsm.sv
    ${IP_ROOT}/ip_jtag/src/ir_cell.sv
    ${IP_ROOT}/ip_jtag/src/dr_cell.sv
    ${IP_ROOT}/ip_jtag/src/scan_cell.sv
    ${IP_ROOT}/ip_jtag/src/by_cell.sv
    ${IP_ROOT}/ip_jtag/src/ir_reg.sv
    ${IP_ROOT}/ip_jtag/src/dr_reg.sv
    ${IP_ROOT}/ip_jtag/src/ir_decode.sv
    ${IP_ROOT}/ip_jtag/src/jtag.sv

    # GPIO IP
    ${IP_ROOT}/ip_gpio/src/asGpio.sv
    ${IP_ROOT}/ip_gpio/src/asGpioTop.sv

    # CGU IP
    ${IP_ROOT}/ip_cgu/src/asCguCore.sv
    ${IP_ROOT}/ip_cgu/src/asCguTop.sv

    # FIFO IP (needed by QSPI)
    ${IP_ROOT}/ip_fifo/src/as_fifo.sv

    # QSPI controller IP
    ${IP_ROOT}/ip_qspi/src/as_qspi.sv
    ${IP_ROOT}/ip_qspi/src/asQspiTop.sv

    # Memory subsystem IPs
    ${IP_ROOT}/ip_icache/src/as_icache_data_ram.sv
    ${IP_ROOT}/ip_icache/src/as_icache_tag_ram.sv
    ${IP_ROOT}/ip_icache/src/as_icache_valid_reg.sv
    ${IP_ROOT}/ip_icache/src/asICache.sv
    ${IP_ROOT}/ip_dcache/src/as_dcache_data_ram.sv
    ${IP_ROOT}/ip_dcache/src/as_dcache_tag_ram.sv
    ${IP_ROOT}/ip_dcache/src/as_dcache_valid_reg.sv
    ${IP_ROOT}/ip_dcache/src/asDCache.sv
    ${IP_ROOT}/ip_scratch/src/as_sram_sp64.sv
    ${IP_ROOT}/ip_scratch/src/asScratch.sv
    ${IP_ROOT}/ip_memarb/src/asMemArb.sv
    ${IP_ROOT}/ip_memsyst/src/asMemTop.sv

    # Cache-version RTL (only files that differ from RV_NoPipeline)
    ${PROJECT_ROOT}/src/asCPUx.sv
    ${PROJECT_ROOT}/src/asTopMem.sv

    # Testbench
    ${TB_FILE}
)

# =================================================
# XSIM
# =================================================
if(SIMULATOR STREQUAL "xsim")

    add_custom_command(
        OUTPUT compile.stamp
        COMMAND xvlog --sv --incr --relax -d LOGIC_SV ${SOURCES_SV}
        COMMAND ${CMAKE_COMMAND} -E copy ${RV64_MEM} riscvtest.mem
        COMMAND ${CMAKE_COMMAND} -E touch compile.stamp
        DEPENDS ${RV64_MEM}
        COMMENT "Compiling SystemVerilog (xvlog)"
    )
    add_custom_target(compile DEPENDS compile.stamp)

    add_custom_command(
        OUTPUT elaborate.stamp
        COMMAND xelab -debug all -top ${TB_TOP} -snapshot ${TB_TOP}_snapshot
        COMMAND ${CMAKE_COMMAND} -E touch elaborate.stamp
        DEPENDS compile.stamp
        COMMENT "Elaborating (xelab)"
    )
    add_custom_target(elaborate DEPENDS elaborate.stamp)

    add_custom_target(simulate
        COMMAND xsim ${TB_TOP}_snapshot --tclbatch ${NP_ROOT}/sim/xsim_cfg.tcl
        DEPENDS elaborate.stamp
        COMMENT "Simulating (xsim)"
    )

    add_custom_target(waves
        COMMAND xsim --gui ${TB_TOP}_snapshot --tclbatch ${NP_ROOT}/sim/xsim_cfg2.tcl
        DEPENDS elaborate.stamp
        COMMENT "Simulating with Vivado Waveform GUI"
    )

# =================================================
# VERILATOR
# =================================================
elseif(SIMULATOR STREQUAL "verilator")

    set(VERILATOR_OUT ${CMAKE_CURRENT_BINARY_DIR}/obj_dir/V${TB_TOP})

    add_custom_command(
        OUTPUT ${VERILATOR_OUT}
        COMMAND verilator
                --cc
                --exe ${PROJECT_ROOT}/sim/sim_main.cpp
                --build
                --sv
                --timing
                -Wall
                -Wno-fatal
                -Wno-DECLFILENAME
                -Wno-PINCONNECTEMPTY
                -Wno-PINNOCONNECT
                -Wno-UNUSEDSIGNAL
                -Wno-IMPORTSTAR
                -Wno-MULTITOP
                -Wno-UNUSEDPARAM
                --Mdir ${CMAKE_CURRENT_BINARY_DIR}/obj_dir
                -DLOGIC_SV
                --trace-fst
                --top-module ${TB_TOP}
                ${PROJECT_ROOT}/sim/tb_waivers.vlt
                ${SOURCES_SV}
        DEPENDS ${RV64_MEM}
        COMMENT "Verilating and building ${TB_TOP}"
    )
    add_custom_target(verilate DEPENDS ${VERILATOR_OUT})

    set(RUN_SIM_SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/run_sim.cmake)
    file(WRITE ${RUN_SIM_SCRIPT}
        "execute_process(COMMAND \"${VERILATOR_OUT}\")\n")

    add_custom_target(simulate
        COMMAND ${CMAKE_COMMAND} -E copy ${RV64_MEM} riscvtest.mem
        COMMAND ${CMAKE_COMMAND} -P ${RUN_SIM_SCRIPT}
        DEPENDS ${VERILATOR_OUT}
        COMMENT "Running Verilator simulation"
    )

    add_custom_target(waves
        COMMAND ${CMAKE_COMMAND} -E copy ${RV64_MEM} riscvtest.mem
        COMMAND ${CMAKE_COMMAND} -P ${RUN_SIM_SCRIPT}
        COMMAND gtkwave dump.fst &
        DEPENDS ${VERILATOR_OUT}
        COMMENT "Running Verilator simulation + GTKWave"
    )

else()
    message(FATAL_ERROR "Unknown SIMULATOR '${SIMULATOR}'. Choose: xsim | verilator")
endif()
