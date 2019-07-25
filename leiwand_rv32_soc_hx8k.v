`timescale 1ns/1ps 

`include "helper.v"
`include "leiwand_rv32_constants.v"

//`define TESTBENCH_MODE


module clk_divn #(
    parameter WIDTH = 3,
    parameter N = 5)
    (clk, clk_out);

    input clk;
    output clk_out;

    reg [WIDTH-1:0] pos_count = 0;
    reg [WIDTH-1:0] neg_count = 0;

    always @(posedge clk)
    if (pos_count == N-1) pos_count <= 0;
    else pos_count <= pos_count +1;

    always @(negedge clk)
    if (neg_count ==N-1) neg_count <= 0;
    else neg_count<= neg_count +1;

    assign clk_out = ((pos_count > (N>>1)) | (neg_count > (N>>1)));
endmodule

`ifdef TESTBENCH_MODE
module leiwandrv32_soc_hx8k();
`else
module leiwandrv32_soc_hx8k(
	CLK,
	RST,
    LED1,
    LED2
);
`endif
    parameter MEMORY_SIZE = 8;

`ifdef TESTBENCH_MODE
    reg CLK = 0;
    reg RST = 0;
    wire LED1;
    wire LED2;

    initial begin 
        CLK=0;
        forever #2 CLK=~CLK;
    end

    initial begin
        #5
        RST=1;
        #50
        RST=0;
        #50
        RST=1;
    end

    initial begin
        $dumpfile("leiwand_rv32_soc_hx8k_tb.vcd");
        $dumpvars(0, leiwandrv32_soc_hx8k);

        // for (i = 0; i < `NR_RV_REGS; i = i + 1) begin
        //     $dumpvars(0, cpu_core.x[i]);
        // end

        // for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
        //     $dumpvars(0, internal_rom.mem[i]);
        // end

        # 150000 $finish;
    end

`else
    input CLK;
    input RST;
    output LED1;
    output LED2;
`endif

    wire system_clock;

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

    clk_divn #(.WIDTH(32), .N(8)) slow_clk(CLK, system_clock);

    leiwand_rv32_core
        cpu_core (
            system_clock, 
            !RST,

            wb_ack,
            wb_data_in,
            wb_stall,
            wb_we,
            wb_stb,
            wb_cyc,
            wb_addr,
            wb_data_out,
            data_write_size,
            LED1
    );

    leiwand_rv32_ram #(
        .MEM_WIDTH(`MEM_WIDTH),
        .MEM_SIZE(MEMORY_SIZE)
    ) internal_rom (
        system_clock,
        !RST,

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

    assign wb_stb_internal_rom = wb_stb & wb_addr[22];// wb_stb && ( (wb_addr >= `MEM_WIDTH'h20400000) && (wb_addr < `MEM_WIDTH'h20400000 + (4*MEMORY_SIZE)) );
    assign wb_data_in = wb_data_in_rom;
    assign wb_stall = wb_stall_rom;
    assign wb_ack = wb_ack_rom;

    //assign LED1 = wb_cyc; //cpu_core.x[10][0];
    assign LED2 = wb_stb; //cpu_core.x[11][0];

endmodule
