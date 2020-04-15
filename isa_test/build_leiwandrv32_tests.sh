#!/bin/bash

set -e

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <ARCH>" >&2
	exit 1
fi

ARCH="$1"

source "${BASH_SOURCE%/*}/tests.sh"

mkdir -p leiwandrv32_compiled_files
mkdir -p leiwandrv32_traces
mkdir -p leiwandrv32_register_states

for i in "${tests[@]}"
do
    ./build_test.sh $i link_leiwandrv32.ld leiwandrv32 leiwandrv32_compiled_files ${ARCH}
done

cur_dir=$(pwd)

for i in "${tests[@]}"
do
    cd ${cur_dir}/..
    ./build.sh isa_test/leiwandrv32_compiled_files/${i}_leiwandrv32 > isa_test/leiwandrv32_traces/${i}_trace.txt ${ARCH}
    cd $cur_dir
    ./convert_leiwandrv32_output.sh leiwandrv32_traces/${i}_trace.txt > leiwandrv32_register_states/${i}_states.txt
    cmp --silent leiwandrv32_register_states/${i}_states.txt qemu_register_states/${i}_states.txt && echo "### SUCCESS: ${i}_states.txt Are Identical! ###" || echo "### ERROR: ${i}_states.txt Are Different! ###"
done
