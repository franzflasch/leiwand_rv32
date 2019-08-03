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

    wire mem_valid;
    wire mem_ready;
    wire [(`MEM_WIDTH-1):0] mem_addr;
    wire [(`MEM_WIDTH-1):0] mem_data_cpu_in;
    wire [(`MEM_WIDTH-1):0] mem_data_cpu_out;
    wire [3:0] mem_wen;

    clk_divn #(.WIDTH(32), .N(8)) slow_clk(CLK, system_clock);

    leiwand_rv32_core
        cpu_core (
            system_clock, 
            !RST,

            mem_valid,
            mem_ready,
            mem_addr,
            mem_data_cpu_in,
            mem_data_cpu_out,
            mem_wen,

            LED1
    );

	simple_mem #(
		.WORDS(MEMORY_SIZE)
	) internal_rom (
		.clk(system_clock),
        .rst(!RST),
        .valid(mem_valid),
        .ready(mem_ready),
		.wen(mem_wen),
		.addr(mem_addr[31:0]),
		.wdata(mem_data_cpu_out),
		.rdata(mem_data_cpu_in)
	);

    //assign LED1 = wb_cyc; //cpu_core.x[10][0];
    //assign LED2 = wb_stb; //cpu_core.x[11][0];

endmodule
