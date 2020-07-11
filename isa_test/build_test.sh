#!/bin/bash

if [ "$#" -ne 6 ]; then
	echo "Usage: $0 <test_name> <linker_script> <output_filename> <output_dir> <ARCH> <EXTENSION>" >&2
	echo "Example (leiwandrv32): ./build_test.sh addi link_leiwandrv32.ld leiwandrv32 test_output"
	echo "Example (qemu): ./build_test.sh addi link_qemu.ld qemu test_output"
	exit 1
fi

TEST_NAME="$1"
LINKER_SCRIPT="$2"
OUTPUT_FILE="$3"
OUTPUT_DIR="$4"
ARCH="$5"
EXTENSION="$6"

TESTS_DIR="riscv-tests/isa/rv${ARCH}${EXTENSION}"
ENV_DIR="riscv-tests/env"

mkdir -p $OUTPUT_DIR

riscv${ARCH}-none-elf-gcc -march=rv${ARCH}ia -g \
	-I. -I${TESTS_DIR}/../macros/scalar/ -I${ENV_DIR}/ -Wl,-T,$LINKER_SCRIPT,-Bstatic -ffreestanding -nostdlib \
	-o ${OUTPUT_DIR}/${TEST_NAME}_${OUTPUT_FILE}.elf ${TESTS_DIR}/${TEST_NAME}.S
riscv${ARCH}-none-elf-objcopy -O binary ${OUTPUT_DIR}/${TEST_NAME}_${OUTPUT_FILE}.elf ${OUTPUT_DIR}/${TEST_NAME}_${OUTPUT_FILE}.bin
