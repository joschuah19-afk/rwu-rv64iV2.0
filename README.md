Single-cycle RV64I.
- I-Mem and D-Mem with synchronous write and synchronous read
- with GPIP peripheral

RWU_RV/RV_NoPipeline/CMakeLists.txt : start the simulation
RWU_RV/RV_NoPipeline/src : SystemVerilog sources
RWU_RV/RV_NoPipeline/tb : SystemVerilog test benches
RWU_RV/RV_NoPipeline/rv64Sim : Assembler files for generating the <test case>.mem files for Verilog (Vivado) simulations

Assembler:
- adapt PATH in ...RWU_RV/RV_NoPipeline/rv64Sim/CMakeLists.txt
-   ... set(RISCV_PREFIX /usr/bin/riscv64-linux-gnu) to your needs

Simulation:
- cd RWU_RV/RV_NoPipeline
- open a terminal
- cmake -S . -B build # generate source directory and build directory
- cmake --build build --target sim_readgpioid # executes a single test
- cmake --build build --target collect_errors # executes all simulations as regression and collects the errors
- rm -rf build # removes the simulation builds
