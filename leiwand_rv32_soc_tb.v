`timescale 1ns/1ps 

`include "helper.v"
`include "leiwand_rv32_constants.v"

module leiwand_rv32_core_tb();

    parameter MEMORY_SIZE = 4096;

    reg clk = 0;
    reg reset = 0;

    reg [(`MEM_WIDTH-1):0] pc;

    wire debug_led;

    wire mem_valid;
    wire mem_ready;
    wire [(`MEM_WIDTH-1):0] mem_addr;
    wire [(`MEM_WIDTH-1):0] mem_data_cpu_in;
    wire [(`MEM_WIDTH-1):0] mem_data_cpu_out;
    wire mem_read_write;
    wire [3:0] mem_wen;

    leiwand_rv32_core
        cpu_core (
            clk, 
            reset,

            mem_valid,
            mem_ready,
            mem_addr,
            mem_data_cpu_in,
            mem_data_cpu_out,
            mem_wen,

            debug_led
    );

	picosoc_mem #(
		.WORDS(MEMORY_SIZE)
	) internal_rom (
		.clk(clk),
        .valid(mem_valid),
        .ready(mem_ready),
		.wen(mem_wen),
		.addr(mem_addr[31:0]),
		.wdata(mem_data_cpu_out),
		.rdata(mem_data_cpu_in)
	);

    initial begin 
        clk=0;
        forever #2 clk=~clk;
    end

    `define SEEK_SET 0
    `define SEEK_CUR 1
    `define SEEK_END 2
   
    integer i, j;
    integer file_size, file, start, count, tmp;

    initial begin

        file = $fopenr(`BINARY_TO_LOAD);

        file_size = $fseek(file, 0, `SEEK_END); /* End of file */
        file_size = $ftell(file);
        tmp = $fseek(file, 0, `SEEK_SET);
        tmp = $fread(internal_rom.mem, file, 0, file_size);

        $display("file size: %d", file_size);

        for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
            internal_rom.mem[i] = {{internal_rom.mem[i][07:00]}, {internal_rom.mem[i][15:08]}, {internal_rom.mem[i][23:16]}, {internal_rom.mem[i][31:24]}};
            $display ("internal ram %d: %x", i, internal_rom.mem[i]);
        end

        $display ("clk: %d", clk);
        #5
        reset=0;
        #10
        reset=1;
        #5
        reset=0;

        for (i = 0; i < 10000; i++) begin
            wait (cpu_core.cpu_stage == cpu_core.STAGE_INSTR_FETCH);
            wait ((cpu_core.cpu_stage == cpu_core.STAGE_INSTR_ALU_PREPARE));

            $display("\n");
            $display("cycle: %d", i);
            $display("stage: %d", cpu_core.cpu_stage);
            $display("pc: %x", cpu_core.pc);
            $display("instr: %x", cpu_core.instruction);

            for(j = 0; j < 32; j++ ) begin
                $display("x[%2d]: %x", j, cpu_core.x[j]);
            end

            if(cpu_core.pc == `SUCCESS_PC) begin 
                $display("SUCCESS!");
                $finish;
            end

        end

        $display("SOMETHING WENT WRONG!");
        $finish;
    end 

    initial begin
        $dumpfile("leiwand_rv32_soc_tb.vcd");

        $dumpvars(0,leiwand_rv32_core_tb);
        for (i = 0; i < `NR_RV_REGS; i = i + 1) begin
            $dumpvars(0, cpu_core.x[i]);
        end

        // for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
        //     $dumpvars(0, internal_rom.mem[i]);
        // end

        // # 15000 $finish;
    end

endmodule 

module picosoc_mem #(
	parameter integer WORDS = 256
) (
	input clk,
    input valid,
    output reg ready,
	input [3:0] wen,
	input [31:0] addr,
	input [31:0] wdata,
	output reg [31:0] rdata
);
	reg [31:0] mem [WORDS-1:0];

	always @(posedge clk) begin
		rdata <= mem[addr[`HIGH_BIT_TO_FIT(WORDS)+2:2]];
		if (wen[0]) mem[addr][ 7: 0] <= wdata[ 7: 0];
		if (wen[1]) mem[addr][15: 8] <= wdata[15: 8];
		if (wen[2]) mem[addr][23:16] <= wdata[23:16];
		if (wen[3]) mem[addr][31:24] <= wdata[31:24];
        ready <= valid && ( (addr >= `MEM_WIDTH'h20400000) && (addr < `MEM_WIDTH'h20400000 + (4*WORDS)) );;
	end
endmodule
