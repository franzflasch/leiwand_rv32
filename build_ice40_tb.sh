BINARY_TO_LOAD=$1
iverilog -sleiwandrv32_soc_hx8k -Wall -o leiwand_rv32_hx8k_tb -DTESTBENCH_MODE -DBINARY_TO_LOAD=\"$BINARY_TO_LOAD\" helper.v leiwand_rv32_constants.v leiwand_rv32_core.v simple_mem.v spimemio.v leiwand_rv32_soc_hx8k.v && vvp leiwand_rv32_hx8k_tb
