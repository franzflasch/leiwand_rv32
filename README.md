# leiwand_rv32
RISC-V CPU written in verilog

This is my first attempt of writing an RV32I core. The name comes from the Viennese word "leiwand" which means something 
like awesome or great.

This is actually an educational project for me. I wanted to learn a bit about FPGAs, Verilog and the risc-v ISA.
The CPU is neither size nor performance optimized, but afterall it fits into an iCE40hx8k FPGA, altough not nearly as good as
the picorv32 from Clifford Wolf.

## Testbench and CPU core verification

The testbench is in leiwand_rv32_soc.v. This should be used for development.
I've added a script to extract the CPU core states from qemu emulating the HiFive1 microcontroller.
To use it you need to have these 2 tools installed:

* riscv32-*-elf-gcc
* qemu-system-riscv32

The tests simply compare the qemu register states with the leiwand_rv32 register states. If they are 100% identical, then
I assume that the core works correctly.

To run the tests do the following:

1. Run the qemu testscript to get the qemu HiFive1 register states:

```
cd isa_test
```

```
./build_qemu_tests.sh
```

This will produce all register states needed to test and verify the leiwand_rv32 core. 

2. Once the qemu register states are ready, the tests for the leiwand_rv32 core can be executed with:

```
./build_leiwandrv32_tests.sh
```

## Synthesizing:
There is also a design which can be synthesized into an iCE40 FPGA using the opensource toolchain Yosys.

Tools needed:
* yosys
* nextpnr
* iceprog

To build and upload the iCE40 design just do:

```
./build_ice40.sh
sudo iceprog leiwand_rv32_soc_hx8k.bin
```

You can also build the same design with iverilog for simulating and debugging with gtkwave. To build the 
testbench version of the design just do:

```
.build_ice40_tb.sh
```

## Firmware

When building the iCE40 design, the firmware uses the same spi flash as used for the bitstream. Entry point is at 0x100000.
The spimemio.v design is stolen from Clifford Wolf (thanks!).

There is a simple blinky example in the firmware folder. To build it issue:

```
riscv32-none-elf-gcc -march=rv32i -Wl,-Bstatic,--strip-debug -ffreestanding -nostdlib -o blinky.elf blinky.S
riscv32-none-elf-objcopy -O binary blinky.elf blinky.bin
```

To upload it to the spi flash do:

```
sudo iceprog -o 1M blinky.bin
```

The blinky is written in assembler. A full featured C example should also be possible, however the current design already uses ~75% of the iCE40hx8k FPGA,
and the internal RAM is only 40 Bytes (8x4 Bytes or 8x32 Bits). So there is more optimization work needed to get an actual usable C environment.

License is GPLv3, feel free to send me PRs.
