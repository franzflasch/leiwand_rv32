#!/bin/bash

set -e

yosys -p 'synth_ice40 -top top -json leiwand_rv32_soc_hx8k.json' helper.v leiwand_rv32_constants.v leiwand_rv32_core.v leiwand_rv32_ram.v leiwand_rv32_soc_hx8k.v
nextpnr-ice40 --hx8k --json leiwand_rv32_soc_hx8k.json --pcf ice40hx8k-evb.pcf --asc leiwand_rv32_soc_hx8k.asc 
