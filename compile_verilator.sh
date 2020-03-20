#!/bin/bash

set -e

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <ARCH>" >&2
	exit 1
fi

ARCH="$1"

rm -rf verilator_isa_test
verilator -DRV${ARCH} -Wall --cc --trace --Mdir verilator_isa_test leiwand_rv32_soc_tb_verilator.v --exe leiwand_rv32_soc_tb_verilator.cpp
make CXXFLAGS="-DRV${ARCH}" -j -C verilator_isa_test/ -f Vleiwand_rv32_soc_tb_verilator.mk Vleiwand_rv32_soc_tb_verilator
