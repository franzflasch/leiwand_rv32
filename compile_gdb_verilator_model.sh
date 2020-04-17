#!/bin/bash

set -e

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <ARCH>" >&2
	exit 1
fi

ARCH="$1"

rm -rf verilator_soc_tb
verilator -DRV${ARCH} -Wall --cc --trace --Mdir verilator_soc_tb leiwand_rv32_soc_tb_verilator.v --exe leiwand_rv32_soc_tb_verilator_run.cpp \
    -CFLAGS "-I/working_dir/Desktop/work/private/gdbserver_sim/build/__installed/usr/inc/" \
    -LDFLAGS "-L/working_dir/Desktop/work/private/gdbserver_sim/build/__installed/usr/lib/ -lgdbserver_stub"
make CXXFLAGS="-DRV${ARCH}" -j -C verilator_soc_tb/ -f Vleiwand_rv32_soc_tb_verilator.mk Vleiwand_rv32_soc_tb_verilator
