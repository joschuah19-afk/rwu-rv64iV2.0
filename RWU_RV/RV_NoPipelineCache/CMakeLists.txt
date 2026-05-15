cmake_minimum_required(VERSION 3.20)
project(rv64i_cache_regression LANGUAGES NONE)

# -------------------------------------------------
# Roots
# -------------------------------------------------
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})
set(IP_ROOT ${PROJECT_ROOT}/../../ CACHE PATH "Root of shared IPs")
set(NP_ROOT ${PROJECT_ROOT}/../RV_NoPipeline CACHE PATH "RV_NoPipeline shared sources")

# -------------------------------------------------
# Simulator
#   cmake -B build -DSIMULATOR=xsim        (Default)
#   cmake -B build -DSIMULATOR=verilator
# -------------------------------------------------
set(SIMULATOR "xsim" CACHE STRING "Simulator: xsim | verilator")
set_property(CACHE SIMULATOR PROPERTY STRINGS xsim verilator)

option(WAVES "Open waveform viewer (single test only, not for regression)" OFF)

if(WAVES)
    set(SIM_TARGET "waves")
    message(STATUS "WAVES=ON: waveform GUI active")
else()
    set(SIM_TARGET "simulate")
endif()

message(STATUS "SIMULATOR  = ${SIMULATOR}")
message(STATUS "SIM_TARGET = ${SIM_TARGET}")

# -------------------------------------------------
# Test list  (JTAG-scan tests excluded — no I-Mem scan chain in cache version)
# -------------------------------------------------
set(ASM_TESTS
    readgpioid
    instr01loadbyte
    instr02loadhalf
    instr03loadword
    instr04loadbyteu
    instr05loadhalfu
    instr06addi
    instr19add
    instr07slli
    instr29lui
)

add_custom_target(regression)

foreach(TEST ${ASM_TESTS})

    set(TEST_BUILD_DIR ${CMAKE_BINARY_DIR}/${TEST})

    # ---------------- ASM ----------------
    add_custom_command(
        OUTPUT ${TEST_BUILD_DIR}/asm/rv64i.mem
        COMMAND ${CMAKE_COMMAND} -E make_directory ${TEST_BUILD_DIR}/asm
        COMMAND ${CMAKE_COMMAND}
                -DASM_FILE=${NP_ROOT}/rv64Sim/${TEST}.asm
                -S ${NP_ROOT}/rv64Sim
                -B ${TEST_BUILD_DIR}/asm
        COMMAND ${CMAKE_COMMAND}
                --build ${TEST_BUILD_DIR}/asm --target mem
        DEPENDS ${NP_ROOT}/rv64Sim/${TEST}.asm
        COMMENT "Assembling ${TEST}"
    )

    # ---------------- SIM ----------------
    add_custom_target(sim_${TEST}
        COMMAND ${CMAKE_COMMAND}
                -DPROJECT_ROOT=${PROJECT_ROOT}
                -DIP_ROOT=${IP_ROOT}
                -DNP_ROOT=${NP_ROOT}
                -DRV64_MEM=${TEST_BUILD_DIR}/asm/rv64i.mem
                -DTB_FILE=${PROJECT_ROOT}/tb/tb_rv64i_${TEST}.sv
                -DSIMULATOR=${SIMULATOR}
                -S ${PROJECT_ROOT}/sim
                -B ${TEST_BUILD_DIR}/sim
        COMMAND ${CMAKE_COMMAND} --build ${TEST_BUILD_DIR}/sim --target ${SIM_TARGET}
        DEPENDS ${TEST_BUILD_DIR}/asm/rv64i.mem
        COMMENT "Simulating ${TEST} [${SIMULATOR}]"
    )

    add_dependencies(regression sim_${TEST})

endforeach()

# -------------------------------------------------
# Error collection
# -------------------------------------------------
list(JOIN ASM_TESTS "," TESTS_CSV)
add_custom_target(collect_errors
    COMMAND ${CMAKE_COMMAND}
        -DPROJECT_ROOT=${PROJECT_ROOT}
        -DBUILD_ROOT=${CMAKE_BINARY_DIR}
        -DTESTS=${TESTS_CSV}
        -P ${NP_ROOT}/collect_errors.cmake
    COMMENT "Collecting regression error files"
)
add_dependencies(collect_errors regression)
