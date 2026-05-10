cmake_minimum_required(VERSION 3.20)
project(rv64i_sim LANGUAGES NONE)

# -------------------------------------------------
# Pflicht-Variablen (müssen vom Root-CMake kommen)
# -------------------------------------------------
foreach(var PROJECT_ROOT IP_ROOT RV64_MEM TB_FILE)
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

# Simulator-Backend (Default: xsim)
if(NOT DEFINED SIMULATOR)
    set(SIMULATOR "xsim")
endif()

set(TB_TOP tb_rv64i)

# -------------------------------------------------
# SystemVerilog Sources
# -------------------------------------------------
set(SOURCES_SV
    ${PROJECT_ROOT}/src/as_pack.sv
    ${PROJECT_ROOT}/src/asRegFile.sv
    #${PROJECT_ROOT}/src/asAdd.sv

    ${IP_ROOT}/ip_dmem/src/asDMemBack.sv
    ${IP_ROOT}/ip_dmem/src/asDMemFront.sv
    ${IP_ROOT}/ip_dmem/src/asDMemCore.sv
    ${IP_ROOT}/ip_dmem/src/asDMem.sv
    ${IP_ROOT}/ip_dmem/src/asDMemTop.sv

    ${PROJECT_ROOT}/src/asIMem.sv
    #${PROJECT_ROOT}/src/asImmGen.sv
    #${PROJECT_ROOT}/src/asMux2.sv
    #${PROJECT_ROOT}/src/asMux3.sv
    #${PROJECT_ROOT}/src/asPC.sv
    #${PROJECT_ROOT}/src/asAluRV.sv
    ${PROJECT_ROOT}/src/asAlu.sv
    ${PROJECT_ROOT}/src/asAluBr.sv
    ${PROJECT_ROOT}/src/asControlAll.sv
    ${PROJECT_ROOT}/src/asInstrDecode.sv

    ${IP_ROOT}/ip_jtag/src/tap_fsm.sv
    ${IP_ROOT}/ip_jtag/src/ir_cell.sv
    ${IP_ROOT}/ip_jtag/src/dr_cell.sv
    ${IP_ROOT}/ip_jtag/src/scan_cell.sv
    ${IP_ROOT}/ip_jtag/src/by_cell.sv
    ${IP_ROOT}/ip_jtag/src/ir_reg.sv
    ${IP_ROOT}/ip_jtag/src/dr_reg.sv
    ${IP_ROOT}/ip_jtag/src/ir_decode.sv
    ${IP_ROOT}/ip_jtag/src/jtag.sv

    ${PROJECT_ROOT}/src/asMBpi.sv
    ${PROJECT_ROOT}/src/asSBpi.sv

    ${IP_ROOT}/ip_gpio/src/asGpio.sv
    ${IP_ROOT}/ip_gpio/src/asGpioTop.sv

    #${PROJECT_ROOT}/src/asCPU.sv
    ${PROJECT_ROOT}/src/asCPUx.sv
    ${PROJECT_ROOT}/src/asDecode.sv

    ${IP_ROOT}/ip_cgu/src/asCguCore.sv
    ${IP_ROOT}/ip_cgu/src/asCguTop.sv

    ${PROJECT_ROOT}/src/asTopMem.sv
    ${TB_FILE}
)

# =================================================
# XSIM (Vivado Simulator)
# =================================================
if(SIMULATOR STREQUAL "xsim")

    # ---------------- Compile ----------------
    add_custom_command(
        OUTPUT compile.stamp
        COMMAND xvlog --sv --incr --relax -d LOGIC_SV ${SOURCES_SV}
        COMMAND ${CMAKE_COMMAND} -E copy ${RV64_MEM} riscvtest.mem
        COMMAND ${CMAKE_COMMAND} -E touch compile.stamp
        DEPENDS ${RV64_MEM}
        COMMENT "Compiling SystemVerilog (xvlog)"
    )
    add_custom_target(compile DEPENDS compile.stamp)

    # ---------------- Elaborate ----------------
    add_custom_command(
        OUTPUT elaborate.stamp
        COMMAND xelab -debug all -top ${TB_TOP} -snapshot ${TB_TOP}_snapshot
        COMMAND ${CMAKE_COMMAND} -E touch elaborate.stamp
        DEPENDS compile.stamp
        COMMENT "Elaborating (xelab)"
    )
    add_custom_target(elaborate DEPENDS elaborate.stamp)

    # ---------------- Simulate (kein GUI) ----------------
    add_custom_target(simulate
        COMMAND xsim ${TB_TOP}_snapshot --tclbatch ${PROJECT_ROOT}/sim/xsim_cfg.tcl
        DEPENDS elaborate.stamp
        COMMENT "Simulating (xsim)"
    )

    # ---------------- Waves (Vivado GUI) ----------------
    add_custom_target(waves
        COMMAND xsim --gui ${TB_TOP}_snapshot --tclbatch ${PROJECT_ROOT}/sim/xsim_cfg2.tcl
        DEPENDS elaborate.stamp
        COMMENT "Simulating with Vivado Waveform GUI (xsim --gui)"
    )

# =================================================
# VERILATOR
# =================================================
elseif(SIMULATOR STREQUAL "verilator")

    set(VERILATOR_OUT ${CMAKE_CURRENT_BINARY_DIR}/obj_dir/V${TB_TOP})

    # ---------------- Verilate + Build ----------------
    # --trace aktiviert FST-Waveform-Ausgabe (dump.fst)
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
                -Wno-UNUSEDPARAM
                -Wno-IMPORTSTAR
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

    # Hilfsskript: execute_process ignoriert den Exit-Code automatisch.
    # Verilator beendet sich mit Exit 1 bei $stop — das ist kein echter Fehler.
    # Echte Fehler werden durch die Testbench in error.txt geschrieben.
    set(RUN_SIM_SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/run_sim.cmake)
    file(WRITE ${RUN_SIM_SCRIPT}
        "execute_process(COMMAND \"${VERILATOR_OUT}\")\n")

    # ---------------- Simulate ----------------
    add_custom_target(simulate
        COMMAND ${CMAKE_COMMAND} -E copy ${RV64_MEM} riscvtest.mem
        COMMAND ${CMAKE_COMMAND} -P ${RUN_SIM_SCRIPT}
        DEPENDS ${VERILATOR_OUT}
        COMMENT "Running Verilator simulation"
    )

    # ---------------- Waves (GTKWave) ----------------
    # Simulation läuft zuerst, dann öffnet GTKWave die erzeugte dump.fst
    add_custom_target(waves
        COMMAND ${CMAKE_COMMAND} -E copy ${RV64_MEM} riscvtest.mem
        COMMAND ${CMAKE_COMMAND} -P ${RUN_SIM_SCRIPT}
        COMMAND gtkwave dump.fst &
        DEPENDS ${VERILATOR_OUT}
        COMMENT "Running Verilator simulation + GTKWave"
    )

else()
    message(FATAL_ERROR "Unbekannter SIMULATOR '${SIMULATOR}'. Wähle: xsim | verilator")
endif()
