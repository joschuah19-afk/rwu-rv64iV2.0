cmake_minimum_required(VERSION 3.20)
project(ip_icache_sim LANGUAGES NONE)

set(TB_TOP tb_asICache)
set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(AS_PACK ${PROJECT_ROOT}/../RWU_RV/RV_NoPipeline/src/as_pack.sv)

set(SOURCES_SV
    ${AS_PACK}
    ${PROJECT_ROOT}/src/as_icache_data_ram.sv
    ${PROJECT_ROOT}/src/as_icache_tag_ram.sv
    ${PROJECT_ROOT}/src/as_icache_valid_reg.sv
    ${PROJECT_ROOT}/src/asICache.sv
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
    COMMENT "Running asICache simulation"
)

add_custom_target(waves
    COMMAND ${CMAKE_COMMAND} -P ${RUN_SIM_SCRIPT}
    COMMAND gtkwave dump_icache.fst &
    DEPENDS ${VERILATOR_OUT}
    COMMENT "Running asICache simulation + GTKWave"
)
