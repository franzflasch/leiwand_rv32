#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <byteswap.h>
#include <Vleiwand_rv32_soc_tb_verilator.h>
#include <verilated.h>
#include <verilated_vcd_c.h>

#include <byteswap.h>
#include <gdb_rsp.h>

#define RISCV_32_REGS 32

void riscv_cpu_g_packet(sds *buf, void *priv)
{
    int i = 0;
    Vleiwand_rv32_soc_tb_verilator *tb = (Vleiwand_rv32_soc_tb_verilator *)priv;
    for(i=0;i<(sizeof(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__x)/
               sizeof(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__x[0]));i++)
    {
        *buf = sdscatprintf(*buf, "%08x", __bswap_32(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__x[i]));
    }
    *buf = sdscatprintf(*buf, "%08x", __bswap_32(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__pc));
}

void riscv_cpu_read_mem(sds *buf, size_t no_bytes, void *priv)
{
    size_t i = 0;
    for(i=0;i<no_bytes;i++)
    {
        *buf = sdscatprintf(*buf, "%02x", 0);
    }
}

void cpu_step(Vleiwand_rv32_soc_tb_verilator *tb)
{
    if(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__cpu_stage != 0)
    {
        printf("ERR: CPU state!\n");
        while(1);
    }

    while(1)
    {
        tb->i_clk = 1;
        tb->eval();

        tb->i_clk = 0;
        tb->eval();

        if(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__cpu_stage == 0)
        {
            //printf("Step done\n");
            break;
        }
    }
}


int main(int argc, char **argv)
{
    // Initialize Verilators variables
    Verilated::commandArgs(argc, argv);

    // Create an instance of our module under test
    Vleiwand_rv32_soc_tb_verilator *tb = new Vleiwand_rv32_soc_tb_verilator;

    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    tb->trace (tfp, 99);
    tfp->open ("leiwand_rv32_soc_tb_verilator.vcd");

    if(argc < 2)
    {
        printf("ERROR: please specify binary!\n");
        exit(1);
    }

    int cycle = 0;
    int i,j = 0;
    FILE *fptr;
    int file_size = 0;
    int prev = 0;

    if ((fptr = fopen(argv[1],"rb")) == NULL){
        printf("ERROR: opening file\n");
        exit(1);
    }

    prev=ftell(fptr);
    fseek(fptr, 0, SEEK_END);
    file_size = ftell(fptr);
    fseek(fptr,prev,SEEK_SET);

    fread(&tb->leiwand_rv32_soc_tb_verilator__DOT__internal_rom__DOT__mem[0], sizeof(tb->leiwand_rv32_soc_tb_verilator__DOT__internal_rom__DOT__mem), 1, fptr);

    int tick = 0;

    /* Reset procedure */
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

    int counter = 0;
    gdb_cpu_interface_td riscv32_test_cpu = {
        .g_packet = riscv_cpu_g_packet,
        .read_mem = riscv_cpu_read_mem,
    };
    gdb_rsp_td *rsp = (gdb_rsp_td *)gdb_rsp.init(&riscv32_test_cpu, (void *)tb);
    gdb_rsp.run_as_thread(rsp);

    for(;;)
    {
        //cpu_step(tb);
        // printf("\n\n");
        // printf("cycle: %d\n", cycle);
        // printf("stage: %d\n", tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__cpu_stage);
        // printf("pc: %x\n", tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__pc);
        // printf("instr: %08x\n", tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__instruction);
        // for(j=0;j<(sizeof(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__x)/sizeof(tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__x[0]));j++)
        //     printf("x[%2d]: %08x\n", j, tb->leiwand_rv32_soc_tb_verilator__DOT__cpu_core__DOT__x[j]);
    }

    gdb_rsp.stop_thread(rsp);
    gdb_rsp.deinit(rsp);

    tfp->close();
    tfp = NULL;

    exit(EXIT_SUCCESS);
}
