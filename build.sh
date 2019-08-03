#!/bin/bash

set -e

iverilog -Wall -o leiwand_rv32 helper.v leiwand_rv32_constants.v leiwand_rv32_core.v leiwand_rv32_ram.v leiwand_rv32_soc_tb.v
vvp leiwand_rv32
