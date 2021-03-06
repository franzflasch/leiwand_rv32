#!/bin/bash


tests=(
lui
auipc
jal
jalr
beq
bne
blt
bge
bltu
bgeu
lb
lh
lw
lbu
lhu
#sb
#sh
#sw
addi
slti
sltiu
xori
ori
andi
slli
srli
srai
add
sub
sll
slt
sltu
xor
srl
sra
or
and
#fence
#fence_i
#ecall
#ebreak
#csrrw
#csrrs
#csrrc
#csrrwi
#csrrsi
#csrrci
)

mkdir -p qemu_compiled_files
mkdir -p qemu_traces
mkdir -p qemu_register_states

for i in "${tests[@]}"
do
    ./build_test.sh $i link_qemu.ld qemu qemu_compiled_files
done

for i in "${tests[@]}"
do
    SUCCESS_PC="$(riscv32-none-elf-objdump -S qemu_compiled_files/${i}_qemu.elf | grep "<pass>:" | awk '{print $1}')"
    echo $SUCCESS_PC
    END_PC=$(echo $((16#$SUCCESS_PC)))
    echo $END_PC

    qemu-system-riscv32 -nographic -machine sifive_e -kernel qemu_compiled_files/${i}_qemu.elf -d in_asm,cpu -s -S 2> qemu_traces/${i}_trace.txt &
    
    # sleep here to give qemu time to start up
    sleep 1

    riscv32-none-elf-gdb -ex "set arch riscv:rv32" -ex "target remote localhost:1234" -ex "source step_mult.gdb" -ex "step_mult $END_PC" qemu_compiled_files/${i}_qemu.elf

    kill -9 $(pidof qemu-system-riscv32)

    # sleep here to give linux some time to kill
    sleep 1
done

for i in "${tests[@]}"
do
    SUCCESS_PC="$(riscv32-none-elf-objdump -S qemu_compiled_files/${i}_qemu.elf | grep "<pass>:" | awk '{print $1}')"

    ./convert_qemu_output.sh qemu_traces/${i}_trace.txt > qemu_register_states/${i}_states.txt
    # delete first two lines as this is some jump in qemu which is not in the elf
    sed -i -e '1,2d' qemu_register_states/${i}_states.txt

    # now delete all lines after the success symbol, as qemu runs freely after we quit gdb
    last_line_number=$(cat qemu_register_states/${i}_states.txt | awk '{print $1}' | grep -n $SUCCESS_PC | cut -d : -f 1)
    last_line_number=$((last_line_number+1))
    sed -i ${last_line_number}',$d' qemu_register_states/${i}_states.txt
done
