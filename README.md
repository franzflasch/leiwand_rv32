# leiwand_rv32
RISC-V CPU written in verilog

This is my first attempt of writing an RV32I core. The name comes from the Viennese word "leiwand" which means something 
like awesome or great.

This is actually an educational project for me. I wanted to learn a bit about FPGAs, Verilog and the risc-v ISA.
The CPU is neither size nor performance optimized, but afterall it fits into an iCE40hx8k FPGA, altough not nearly as good as
the picorv32 from Clifford Wolf.

Currently only a subset of the RV32I ISA is implemented. The following instructions are currently available:

* lui
* auipc
* jal
* jalr
* beq
* bne
* blt
* bge
* bltu
* bgeu
* lb
* lh
* lw
* lbu
* lhu
* sb
* sh
* sw
* addi
* slti
* sltiu
* xori
* ori
* andi
* slli
* srli
* srai
* add
* sub
* sll
* slt
* sltu
* xor
* srl
* sra
* or
* and

The instructions above are the most important ones for an risc-v core. Currently not implemented:

* fence
* fence_i
* ecall
* ebreak
* csrrw
* csrrs
* csrrc
* csrrwi
* csrrsi
* csrrci

However for an absolute minimal RV32I CPU they are not needed anyway.

## Interrupt support
There is now a rudimentary support for interrupts. Most of the IRQ work is done in SW.
The cpu core was modified in an absolute minimal manner to support this.
Behavior is like this: The CPU reads a 32 bit wide interrupt status register. So currently 32 different IRQ sources are possible.
If this register is not zero and the internal IRQ flag within the CPU is zero, the CPU jumps to the instruction located at
PC_START_VAL + 4. Here should be a jump to the interrupt service routine. If the CPU already has marked the internal flag with 1,
the CPU will just continuing with the IRQ service routine. So a jump to the IRQ vector is triggered if (irq_status_reg && !irq_internal_flag).
The IRQ service routine has to delete the status register at the end, otherwise the CPU will jump to the interrupt vector again and again. Furthermore
the IRQ service routine has to call the csrrw instruction at the very end of the routine to clear the internal irq flag, otherwise undefined behavior will occur!!

The c_blinky example was updated to support interrupts. Within the init.S the current context will be saved, then the IRQ handler is called, 
and after that the context is restored again. Just have a look at the implementation to get an idea how it works.

Currently MISSING: register to enable the interrupts, to prevent spurious interrupts during bootup.

## Testbench and CPU core verification

The testbench is in leiwand_rv32_soc.v. This should be used for development.
I've added a script to extract the CPU core states from qemu emulating the HiFive1 microcontroller.
To use it you need to have these 2 tools installed:

* riscv32-*-elf-gcc
* qemu-system-riscv32
* iverilog

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

For more details about the testing please see:
https://franzflasch.github.io/debugging/risc-v/verilog/2019/07/31/riscv-core-debugging-with-qemu.html

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
.build_ice40_tb.sh firmware.bin
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


There is also a C blinky example in the firmware folder. You can build it with:
```
./build_fw.sh
```

To upload it to the spi flash do:

```
sudo iceprog -o 1M c_blinky.bin
```

License is GPLv3, feel free to send me PRs.
