`include "helper.v"
`include "leiwand_rv32_constants.v"

module top(						//top module
	CLK,
	RST,
    LED1,
    LED2
);

    parameter MEMORY_SIZE = 1024;

    input CLK;
    input RST;
    output	LED1;				//output signal to LED1
    output	LED2;				//output signal to LED2

    wire wb_ack;
    wire [(`MEM_WIDTH-1):0] wb_data_in;
    wire wb_stall;
    wire wb_we;
    wire wb_cyc;
    wire wb_stb;
    wire [(`MEM_WIDTH-1):0] wb_addr;
    wire [(`MEM_WIDTH-1):0] wb_data_out;
    wire [`HIGH_BIT_TO_FIT(4):0] data_write_size;

    wire [(`MEM_WIDTH-1):0] wb_data_in_rom;
    wire wb_stb_internal_rom;
    wire wb_ack_rom;
    wire wb_stall_rom;

    leiwand_rv32_core
        cpu_core (
            CLK, 
            RST,

            wb_ack,
            wb_data_in,
            wb_stall,
            wb_we,
            wb_stb,
            wb_cyc,
            wb_addr,
            wb_data_out,
            data_write_size
    );

    leiwand_rv32_ram #(
        .MEM_WIDTH(`MEM_WIDTH),
        .MEM_SIZE(MEMORY_SIZE)
    ) internal_rom (
        clk,
        reset,
        wb_addr,
        wb_data_out,
        wb_data_in_rom,
        wb_we,
        wb_stb_internal_rom,
        wb_ack_rom,
        wb_cyc,
        wb_stall_rom,
        data_write_size
    );

    assign wb_stb_internal_rom = wb_stb && ( (wb_addr >= `MEM_WIDTH'h20400000) && (wb_addr < `MEM_WIDTH'h20400000 + (4*MEMORY_SIZE)) );
    assign wb_data_in = wb_data_in_rom;
    assign wb_stall = wb_stall_rom;
    assign wb_ack = wb_ack_rom;

    assign LED1 = wb_cyc;
    assign LED2 = wb_stb_internal_rom;

endmodule
