#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <byteswap.h>
#include <Vleiwand_rv32_soc_tb_verilator.h>
#include <verilated.h>
#include <verilated_vcd_c.h>

int main(int argc, char **argv) {

    int tick = 0;
    int cycle = 0;
    int i,j = 0;
    FILE *fptr;
    int file_size = 0;
    int prev = 0;
    uint32_t success_pc = 0;
    VerilatedVcdC *tfp;

    // Initialize Verilators variables
    Verilated::commandArgs(argc, argv);

    // Create an instance of our module under test
    Vleiwand_rv32_soc_tb_verilator *tb = new Vleiwand_rv32_soc_tb_verilator;

    Verilated::traceEverOn(true);
    tfp = new VerilatedVcdC;
    tb->trace (tfp, 99);
    tfp->open ("leiwand_rv32_soc_tb_verilator.vcd");

    if(argc < 2)
    {
        printf("ERROR: please specify binary!\n");
        exit(1);
    }

    if(argc < 3)
    {
        printf("ERROR: please specify success PC!\n");
        exit(1);
    }

    if ((fptr = fopen(argv[1],"rb")) == NULL){
        printf("ERROR: opening file\n");
        exit(1);
    }

    prev=ftell(fptr);
    fseek(fptr, 0, SEEK_END);
    file_size = ftell(fptr);
    fseek(fptr,prev,SEEK_SET);

    success_pc = strtol(argv[2], NULL, 16);

    //printf("file_size: %d success pc: %x\n", file_size, success_pc);

    fread(&tb->leiwand_rv32_soc_tb_verilator__DOT__internal_rom__DOT__mem[0], sizeof(tb->leiwand_rv32_soc_tb_verilator__DOT__internal_rom__DOT__mem), 1, fptr);

    // for(i=0;i<(sizeof(tb->leiwand_rv32_soc_tb_verilator__DOT__internal_rom__DOT__mem)/sizeof(tb->leiwand_rv32_soc_tb_verilator__DOT__internal_rom__DOT__mem[0]));i++)
    // {
    //     //tb->leiwand_rv32_soc_tb_verilator__DOT__internal_rom__DOT__mem[i] = __bswap_32(tb->leiwand_rv32_soc_tb_verilator__DOT__internal_rom__DOT__mem[i]);
    //     printf("mem: %x\n", tb->leiwand_rv32_soc_tb_verilator__DOT__internal_rom__DOT__mem[i]);
    // }

    tb->i_rst = 0;

    for(i=0;i<10;i++)
    {
        tfp->dump (2*tick);
        tb->i_clk = 1;
        tb->eval();
        tick++;

        tfp->dump (2*tick);
        tb->i_clk = 0;
        tb->eval();
        tick++;
    }

    tb->i_rst = 1;

    for(i=0;i<10;i++)
    {
        tfp->dump (tick);
        tb->i_clk = 1;
        tb->eval();
        tick++;

        tfp->dump (tick);
        tb->i_clk = 0;
        tb->eval();
        tick++;
    }

    tb->i_rst = 0;

    for(i=0;i<100000;i++)
    {
        if(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__cpu_stage == 2)
        {
            printf("\n\n");
            printf("cycle: %d\n", cycle);
            printf("stage: %d\n", tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__cpu_stage);
            printf("pc: %x\n", tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__pc);
            printf("instr: %08x\n", tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__instruction);
            for(j=0;j<32;j++)
                printf("x[%2d]: %08x\n", j, tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__x[j]);

            cycle++;
        }

        if(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__pc == success_pc+4)
        {
            break;
        }

        tfp->dump (tick);
        tb->i_clk = 1;
        tb->eval();
        tick++;

        tfp->dump (tick);
        tb->i_clk = 0;
        tb->eval();
        tick++;
    }

    if(i==100000)
        printf("SOMETHING WENT WRONG!\n");
    else
    {
        printf("SUCCESS!\n");
    }

    tfp->close();
    tfp = NULL;

    exit(EXIT_SUCCESS);
}
