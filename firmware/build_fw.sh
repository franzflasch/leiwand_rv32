#/bin/bash

set -e

riscv32-none-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.lds,--strip-debug -ffreestanding -nostdlib -nostartfiles -o c_blinky.elf init.S main.c
riscv32-none-elf-objcopy -O binary c_blinky.elf c_blinky.bin

# print text section
#riscv32-none-elf-objdump -d hx8kdemo_fw.elf | awk '{print "0x"$2","}'

# dump all
#riscv32-none-elf-objdump -d hx8kdemo_fw.elf 

