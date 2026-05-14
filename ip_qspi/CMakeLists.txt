cmake_minimum_required(VERSION 3.20)
project(ip_qspi_sim LANGUAGES NONE)

set(TB_TOP asQspiTop_tb)
set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(RV_ROOT ${PROJECT_ROOT}/../RWU_RV/RV_NoPipeline)

set(SOURCES_SV
    ${RV_ROOT}/src/as_pack.sv
    ${RV_ROOT}/src/asSBpi.sv
    ${PROJECT_ROOT}/../ip_fifo/src/as_fifo.sv
    ${PROJECT_ROOT}/src/as_qspi.sv
    ${PROJECT_ROOT}/src/asQspiTop.sv
    ${PROJECT_ROOT}/tb/${TB_TOP}.sv
)

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
            ${SOURCES_SV}
    DEPENDS ${SOURCES_SV}
    COMMENT "Verilating and building ${TB_TOP}"
)
add_custom_target(verilate DEPENDS ${VERILATOR_OUT})

set(RUN_SIM_SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/run_sim.cmake)
file(WRITE ${RUN_SIM_SCRIPT}
    "execute_process(COMMAND \"${VERILATOR_OUT}\")\n")

add_custom_target(simulate
    COMMAND ${CMAKE_COMMAND} -P ${RUN_SIM_SCRIPT}
    DEPENDS ${VERILATOR_OUT}
    COMMENT "Running asQspiTop simulation"
)

add_custom_target(waves
    COMMAND ${CMAKE_COMMAND} -P ${RUN_SIM_SCRIPT}
    COMMAND gtkwave dump_qspi.fst &
    DEPENDS ${VERILATOR_OUT}
    COMMENT "Running asQspiTop simulation + GTKWave"
)
