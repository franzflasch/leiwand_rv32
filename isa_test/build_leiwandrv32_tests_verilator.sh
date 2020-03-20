#!/bin/bash

set -e

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <ARCH>" >&2
	exit 1
fi

ARCH="$1"

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
# #sb
# #sh
# #sw
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
# #fence
# #fence_i
# #ecall
# #ebreak
# #csrrw
# #csrrs
# #csrrc
# #csrrwi
# #csrrsi
# #csrrci
)

if [ "${ARCH}" = "64" ]; then
    tests+=(
        addiw
        slliw
        srliw
        sraiw
        addw
        subw
        sllw
        srlw
        sraw
    )
fi;

mkdir -p leiwandrv32_compiled_files
mkdir -p leiwandrv32_traces
mkdir -p leiwandrv32_register_states

for i in "${tests[@]}"
do
    ./build_test.sh $i link_leiwandrv32.ld leiwandrv32 leiwandrv32_compiled_files ${ARCH}
done

cur_dir=$(pwd)

cd ${cur_dir}/..
./compile_verilator.sh ${ARCH}
cd $cur_dir

for i in "${tests[@]}"
do
    cd ${cur_dir}/..
    ./build_verilator.sh isa_test/leiwandrv32_compiled_files/${i}_leiwandrv32 > isa_test/leiwandrv32_traces/${i}_trace.txt ${ARCH}
    cd $cur_dir
    ./convert_leiwandrv32_output.sh leiwandrv32_traces/${i}_trace.txt > leiwandrv32_register_states/${i}_states.txt
    cmp --silent leiwandrv32_register_states/${i}_states.txt qemu_register_states/${i}_states.txt && echo "### SUCCESS: ${i}_states.txt Are Identical! ###" || echo "### ERROR: ${i}_states.txt Are Different! ###"
done
