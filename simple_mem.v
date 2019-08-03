`timescale 1ns/1ps 

module simple_mem #(
	parameter integer WORDS = 256
) (
	input clk,
	input rst,
    input valid,
    output reg ready,
	input [3:0] wen,
	input [31:0] addr,
	input [31:0] wdata,
	output reg [31:0] rdata
);
	reg [31:0] mem [WORDS-1:0];

	always @(posedge clk) begin
        if(rst) begin
			ready <= 0;
			mem[0] <= 32'h00018637;
			mem[1] <= 32'h00158593;
			mem[2] <= 32'h00c58463;
			mem[3] <= 32'hff9ff06f;
			mem[4] <= 32'h00000593;
			mem[5] <= 32'h00154513;
			mem[6] <= 32'hfedff06f;
		end
		else begin
			rdata <= mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]];
			if (wen[0]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][ 7: 0] <= wdata[ 7: 0];
			if (wen[1]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][15: 8] <= wdata[15: 8];
			if (wen[2]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][23:16] <= wdata[23:16];
			if (wen[3]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][31:24] <= wdata[31:24];
			ready <= valid && ( (addr >= `MEM_WIDTH'h20400000) && (addr < `MEM_WIDTH'h20400000 + (4*WORDS)) );
		end
	end
endmodule
