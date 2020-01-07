#!/bin/bash

set -e

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <bin_to_load>" >&2
	echo "Example (leiwandrv32): ./build.sh isa_test/test_output/addi_leiwandrv32"
	exit 1
fi

BINARY_TO_LOAD=$1.bin
ELF_FILE=$1.elf

if [ ! -f "$BINARY_TO_LOAD" ]
then
	echo "$BINARY_TO_LOAD not found."
    exit 1
fi

SUCCESS_PC="$(riscv32-none-elf-objdump -S $ELF_FILE | grep "<pass>:" | awk '{print $1}')"

./verilator_isa_test/Vleiwand_rv32_soc_tb_verilator $BINARY_TO_LOAD $SUCCESS_PC
