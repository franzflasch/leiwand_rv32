#!/bin/bash

set -e

yosys -q -p 'synth_ice40 -top leiwandrv32_soc_hx8k -json leiwand_rv32_soc_hx8k.json' helper.v leiwand_rv32_constants.v leiwand_rv32_core.v simple_mem.v spimemio.v leiwand_rv32_soc_hx8k.v
nextpnr-ice40 --hx8k --json leiwand_rv32_soc_hx8k.json --pcf ice40hx8k-evb.pcf --asc leiwand_rv32_soc_hx8k.asc 
icepack leiwand_rv32_soc_hx8k.asc leiwand_rv32_soc_hx8k.bin  
