`timescale 1ns/1ps 

`include "helper.v"
`include "leiwand_rv32_constants.v"

module leiwand_rv32_core_tb();

    parameter MEMORY_SIZE = 128;

    reg clk = 0;
    reg reset = 0;

    reg [(`MEM_WIDTH-1):0] pc;

    wire wb_ack;
    wire [(`MEM_WIDTH-1):0] wb_data_in;
    wire wb_stall;
    wire wb_we;
    wire wb_cyc;
    wire wb_stb;
    wire [(`MEM_WIDTH-1):0] wb_addr;
    wire [(`MEM_WIDTH-1):0] wb_data_out;

    wire [(`MEM_WIDTH-1):0] wb_data_in_sram;
    wire [(`MEM_WIDTH-1):0] wb_data_in_rom;
    wire wb_stb_internal_sram;
    wire wb_stb_internal_rom;
    wire wb_ack_sram;
    wire wb_ack_rom;
    wire wb_stall_sram;
    wire wb_stall_rom;

    leiwand_rv32_core
        cpu_core (
            clk, 
            reset,

            wb_ack,
            wb_data_in,
            wb_stall,
            wb_we,
            wb_stb,
            wb_cyc,
            wb_addr,
            wb_data_out
    );

    leiwand_rv32_ram #(
        .MEM_WIDTH(32),
        .MEM_SIZE(MEMORY_SIZE)
    ) internal_sram (
        clk,
        reset,
        /* byte addressing so counting start at bit 2 */
        wb_addr[`HIGH_BIT_TO_FIT(MEMORY_SIZE-1)+2:2],
        wb_data_out,
        wb_data_in_sram,
        wb_we,
        wb_stb_internal_sram,
        wb_ack_sram,
        wb_cyc,
        wb_stall_sram
    );

    leiwand_rv32_ram #(
        .MEM_WIDTH(32),
        .MEM_SIZE(MEMORY_SIZE)
    ) internal_rom (
        clk,
        reset,
        /* byte addressing so counting start at bit 2 */
        wb_addr[`HIGH_BIT_TO_FIT(MEMORY_SIZE-1)+2:2],
        wb_data_out,
        wb_data_in_rom,
        wb_we,
        wb_stb_internal_rom,
        wb_ack_rom,
        wb_cyc,
        wb_stall_rom
    );

    assign wb_stb_internal_sram = wb_stb &&  ( (wb_addr >= `MEM_WIDTH'h10000000) && (wb_addr < `MEM_WIDTH'h10000000 + (4*MEMORY_SIZE)) );
    assign wb_stb_internal_rom = wb_stb && ( (wb_addr >= `MEM_WIDTH'h20000000) && (wb_addr < `MEM_WIDTH'h20000000 + (4*MEMORY_SIZE)) );
    assign wb_data_in = wb_data_in_rom | wb_data_in_sram;
    assign wb_stall = wb_stall_rom | wb_stall_sram;
    assign wb_ack = wb_ack_rom | wb_ack_sram;

    initial begin 
        clk=0;
        forever #2 clk=~clk;
    end

    initial begin
        $display ("clk: %d", clk);
        #5
        reset=0;
        #10
        reset=1;
        #5
        reset=0;
    end
    
    integer i;
    initial
    begin
        $dumpfile("leiwand_rv32_soc_tb.vcd");
        $dumpvars(0,leiwand_rv32_core_tb);
        for (i = 0; i < `NR_RV_REGS; i = i + 1) begin
            $dumpvars(0, cpu_core.x[i]);
        end

        for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
            $dumpvars(0, internal_rom.mem[i]);
        end

        # 15000 $finish;
    end

endmodule 
