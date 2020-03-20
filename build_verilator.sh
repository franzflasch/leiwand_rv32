#!/bin/bash

set -e

if [ "$#" -ne 2 ]; then
	echo "Usage: $0 <bin_to_load> <ARCH>" >&2
	echo "Example (leiwandrv32): ./build.sh isa_test/test_output/addi_leiwandrv32 64"
	exit 1
fi

BINARY_TO_LOAD=$1.bin
ELF_FILE=$1.elf
ARCH=$2

if [ ! -f "$BINARY_TO_LOAD" ]
then
	echo "$BINARY_TO_LOAD not found."
    exit 1
fi

SUCCESS_PC="$(riscv${ARCH}-none-elf-objdump -S $ELF_FILE | grep "<pass>:" | awk '{print $1}')"

./verilator_isa_test/Vleiwand_rv32_soc_tb_verilator $BINARY_TO_LOAD $SUCCESS_PC
