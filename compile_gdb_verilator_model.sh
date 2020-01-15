#!/bin/bash

set -e

rm -rf verilator_soc_tb
verilator -Wall --cc --trace --Mdir verilator_soc_tb leiwand_rv32_soc_tb_verilator.v --exe leiwand_rv32_soc_tb_verilator_run.cpp \
    -CFLAGS "-I/working_dir/Desktop/work/priv/gdb_server/gdbserver_sim/build/__installed/usr/inc/" \
    -LDFLAGS "-L/working_dir/Desktop/work/priv/gdb_server/gdbserver_sim/build/__installed/usr/lib/ -lgdbserver_stub"
make -j -C verilator_soc_tb/ -f Vleiwand_rv32_soc_tb_verilator.mk Vleiwand_rv32_soc_tb_verilator
