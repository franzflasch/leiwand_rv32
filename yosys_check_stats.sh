yosys -p "synth -top leiwandrv32_soc_hx8k; stat" leiwand_rv32_core.v simple_mem.v leiwand_rv32_constants.v helper.v leiwand_rv32_soc_hx8k.v
