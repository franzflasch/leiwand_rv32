#!/bin/bash

set -e

rm -rf obj_dir
verilator -Wall --cc --trace leiwand_rv32_soc_tb_verilator.v --exe leiwand_rv32_soc_tb_verilator.cpp
make -j -C obj_dir/ -f Vleiwand_rv32_soc_tb_verilator.mk Vleiwand_rv32_soc_tb_verilator 
