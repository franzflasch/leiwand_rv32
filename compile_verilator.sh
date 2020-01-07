#!/bin/bash

set -e

rm -rf verilator_isa_test
verilator -Wall --cc --trace --Mdir verilator_isa_test leiwand_rv32_soc_tb_verilator.v --exe leiwand_rv32_soc_tb_verilator.cpp
make -j -C verilator_isa_test/ -f Vleiwand_rv32_soc_tb_verilator.mk Vleiwand_rv32_soc_tb_verilator

rm -rf verilator_soc_tb
verilator -Wall --cc --trace --Mdir verilator_soc_tb leiwand_rv32_soc_tb_verilator.v --exe leiwand_rv32_soc_tb_verilator_run.cpp
make -j -C verilator_soc_tb/ -f Vleiwand_rv32_soc_tb_verilator.mk Vleiwand_rv32_soc_tb_verilator