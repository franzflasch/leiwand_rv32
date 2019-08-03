#!/bin/bash

set -e

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <bin_to_load>" >&2
	echo "Example (leiwandrv32): ./build.sh isa_test/test_output/addi_leiwandrv32.bin"
	exit 1
fi

BINARY_TO_LOAD=$1

if [ ! -f "$BINARY_TO_LOAD" ]
then
	echo "$BINARY_TO_LOAD not found."
    exit 1
fi

iverilog -sleiwand_rv32_core_tb -DBINARY_TO_LOAD=\"$BINARY_TO_LOAD\" -Wall -o leiwand_rv32 helper.v leiwand_rv32_constants.v leiwand_rv32_core.v leiwand_rv32_ram.v leiwand_rv32_soc_tb.v
vvp leiwand_rv32
