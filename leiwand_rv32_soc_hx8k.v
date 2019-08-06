/*
 *   This file is part of leiwand_rv32.
 *   Copyright (c) 2019 Franz Flasch.
 *
 *   leiwand_rv32 is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   leiwand_rv32 is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with leiwand_rv32.  If not, see <https://www.gnu.org/licenses/>.
 */

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
	input CLK,
	input RST,
    output LED1,
    output LED2,
    output flash_csb,
	output flash_clk,
	inout  flash_io0,
	inout  flash_io1,
	inout  flash_io2,
	inout  flash_io3
);
`endif
    parameter RAM_SIZE = 128; /* MEM_WIDTH WORDS */
    parameter FLASH_SIZE = `MEM_WIDTH'h400000;

`ifdef TESTBENCH_MODE
    reg CLK = 0;
    reg RST = 0;
    wire flash_csb;
    wire flash_clk;
    wire flash_io0;
    wire flash_io1;
    wire flash_io2;
    wire flash_io3;
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

    integer i;
    initial begin
        $dumpfile("leiwand_rv32_soc_hx8k_tb.vcd");
        $dumpvars(0, leiwandrv32_soc_hx8k);

        for (i = 0; i < `NR_RV_REGS; i = i + 1) begin
            $dumpvars(0, cpu_core.x[i]);
        end

        // for (i = 0; i < RAM_SIZE; i = i + 1) begin
        //     $dumpvars(0, internal_rom.mem[i]);
        // end

        # 150000 $finish;
    end
`endif

    wire system_clock;

	reg [5:0] reset_cnt = 0;
	wire resetn = &reset_cnt;

	always @(posedge system_clock) begin
		reset_cnt <= reset_cnt + !resetn;
	end

    wire mem_valid;
    wire mem_ready;
    wire [(`MEM_WIDTH-1):0] mem_addr;
    wire [(`MEM_WIDTH-1):0] mem_data_cpu_in;
    wire [(`MEM_WIDTH-1):0] mem_data_cpu_out;
    wire [3:0] mem_wen;

    clk_divn #(.WIDTH(32), .N(8)) slow_clk(CLK, system_clock);

    leiwand_rv32_core # (.PC_START_VAL(`MEM_WIDTH'h100000))
        cpu_core (
            system_clock, 
            !resetn,

            mem_valid,
            mem_ready,
            mem_addr,
            mem_data_cpu_in,
            mem_data_cpu_out,
            mem_wen,

            LED1
    );


    wire ram_ready;
    wire [(`MEM_WIDTH-1):0] ram_rdata;

	simple_mem #(
		.WORDS(RAM_SIZE)
	) internal_ram (
		.clk(system_clock),
        .rst(!resetn),
        .valid(mem_valid && (mem_addr >= `MEM_WIDTH'h20400000) && (mem_addr < `MEM_WIDTH'h20400000 + (4*RAM_SIZE))),
        .ready(ram_ready),
		.wen(mem_wen),
		.addr(mem_addr[31:0]),
		.wdata(mem_data_cpu_out),
		.rdata(ram_rdata)
	);

    // assign mem_ready = ram_ready;
	// assign mem_data_cpu_in = ram_ready ? ram_rdata : 32'h 0000_0000;

	wire flash_io0_oe, flash_io0_do, flash_io0_di;
	wire flash_io1_oe, flash_io1_do, flash_io1_di;
	wire flash_io2_oe, flash_io2_do, flash_io2_di;
	wire flash_io3_oe, flash_io3_do, flash_io3_di;

`ifndef TESTBENCH_MODE
	SB_IO #(
		.PIN_TYPE(6'b 1010_01),
		.PULLUP(1'b 0)
	) flash_io_buf [3:0] (
		.PACKAGE_PIN({flash_io3, flash_io2, flash_io1, flash_io0}),
		.OUTPUT_ENABLE({flash_io3_oe, flash_io2_oe, flash_io1_oe, flash_io0_oe}),
		.D_OUT_0({flash_io3_do, flash_io2_do, flash_io1_do, flash_io0_do}),
		.D_IN_0({flash_io3_di, flash_io2_di, flash_io1_di, flash_io0_di})
	);
`endif

    wire spimem_ready;
    wire [(`MEM_WIDTH-1):0] spimem_rdata;
    wire [(`MEM_WIDTH-1):0] spimemio_cfgreg_do;

	spimemio spimemio (
		.clk    (system_clock),
		.resetn (resetn),
		.valid  (mem_valid && (mem_addr >= `MEM_WIDTH'h100000) && (mem_addr < `MEM_WIDTH'h100000 + (4*FLASH_SIZE))),
		.ready  (spimem_ready),
		.addr   (mem_addr[23:0]),
		.rdata  (spimem_rdata),

		.flash_csb    (flash_csb   ),
		.flash_clk    (flash_clk   ),

		.flash_io0_oe (flash_io0_oe),
		.flash_io1_oe (flash_io1_oe),
		.flash_io2_oe (flash_io2_oe),
		.flash_io3_oe (flash_io3_oe),

		.flash_io0_do (flash_io0_do),
		.flash_io1_do (flash_io1_do),
		.flash_io2_do (flash_io2_do),
		.flash_io3_do (flash_io3_do),

		.flash_io0_di (flash_io0_di),
		.flash_io1_di (flash_io1_di),
		.flash_io2_di (flash_io2_di),
		.flash_io3_di (flash_io3_di),

		.cfgreg_we(4'b 0000),
		.cfgreg_di(mem_data_cpu_out),
		.cfgreg_do(spimemio_cfgreg_do)
	);

    assign mem_ready = ram_ready | spimem_ready;
	assign mem_data_cpu_in = ram_ready ? ram_rdata : spimem_ready ? spimem_rdata : 32'h 0000_0000;

    //assign LED1 = wb_cyc; //cpu_core.x[10][0];
    assign LED2 = spimem_ready; //cpu_core.x[11][0];

endmodule
