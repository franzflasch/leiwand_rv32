#!/bin/bash

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <ARCH>" >&2
	exit 1
fi

ARCH="$1"

source "${BASH_SOURCE%/*}/tests.sh"

mkdir -p qemu_compiled_files
mkdir -p qemu_traces
mkdir -p qemu_register_states

for i in "${tests_ui[@]}"
do
    ./build_test.sh $i linker_script.ld qemu qemu_compiled_files ${ARCH} ui
done

for i in "${tests_ua[@]}"
do
    ./build_test.sh $i linker_script.ld qemu qemu_compiled_files ${ARCH} ua
done

tests=("${tests_ui[@]}" "${tests_ua[@]}")

for i in "${tests[@]}"
do
    SUCCESS_PC="$(riscv${ARCH}-none-elf-objdump -S qemu_compiled_files/${i}_qemu.elf | grep "<pass>:" | awk '{print $1}')"
    echo $SUCCESS_PC
    END_PC=$(echo $((16#$SUCCESS_PC)))
    echo $END_PC

    qemu-system-riscv${ARCH} -nographic -machine virt -m 128M -kernel qemu_compiled_files/${i}_qemu.elf -d in_asm,cpu -s -S 2> qemu_traces/${i}_trace.txt &

    # sleep here to give qemu time to start up
    sleep 1

    riscv${ARCH}-none-elf-gdb -ex "set arch riscv:rv${ARCH}" -ex "target remote localhost:1234" -ex "source step_mult.gdb" -ex "step_mult $END_PC" qemu_compiled_files/${i}_qemu.elf

    kill -9 $(pidof qemu-system-riscv${ARCH})

    # sleep here to give linux some time to kill
    sleep 1
done

for i in "${tests[@]}"
do
    SUCCESS_PC="$(riscv${ARCH}-none-elf-objdump -S qemu_compiled_files/${i}_qemu.elf | grep "<pass>:" | awk '{print $1}')"

    ./convert_qemu_output.sh qemu_traces/${i}_trace.txt > qemu_register_states/${i}_states.txt
    # delete first five lines as this is some jump in qemu which is not in the elf
    sed -i -e '1,5d' qemu_register_states/${i}_states.txt

    # now delete all lines after the success symbol, as qemu runs freely after we quit gdb
    last_line_number=$(cat qemu_register_states/${i}_states.txt | awk '{print $1}' | grep -n $SUCCESS_PC | cut -d : -f 1)
    last_line_number=$((last_line_number+1))
    sed -i ${last_line_number}',$d' qemu_register_states/${i}_states.txt
done
