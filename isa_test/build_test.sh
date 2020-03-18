#!/bin/bash

TESTS_DIR="riscv-tests/isa/rv32ui"
ENV_DIR="riscv-tests/env"

if [ "$#" -ne 4 ]; then
	echo "Usage: $0 <test_name> <linker_script> <output_filename> <output_dir>" >&2
	echo "Example (leiwandrv32): ./build_test.sh addi link_leiwandrv32.ld leiwandrv32 test_output"
	echo "Example (qemu): ./build_test.sh addi link_qemu.ld qemu test_output"
	exit 1
fi

TEST_NAME="$1"
LINKER_SCRIPT="$2"
OUTPUT_FILE="$3"
OUTPUT_DIR="$4"

mkdir -p $OUTPUT_DIR

riscv32-none-elf-gcc -march=rv32i -g \
	-I. -I${TESTS_DIR}/../macros/scalar/ -I${ENV_DIR}/ -Wl,-T,$LINKER_SCRIPT,-Bstatic -ffreestanding -nostdlib \
	-o ${OUTPUT_DIR}/${TEST_NAME}_${OUTPUT_FILE}.elf ${TESTS_DIR}/${TEST_NAME}.S
riscv32-none-elf-objcopy -O binary ${OUTPUT_DIR}/${TEST_NAME}_${OUTPUT_FILE}.elf ${OUTPUT_DIR}/${TEST_NAME}_${OUTPUT_FILE}.bin
