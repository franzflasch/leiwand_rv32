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
    parameter FLASH_SIZE = `MEM_WIDTH'h1000 /* FLASH_SIZE WORDS */; 

`ifdef TESTBENCH_MODE
    reg CLK = 0;
    reg [(`MEM_WIDTH-1):0] clk_count = 0;
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

    `define SEEK_SET 0
    `define SEEK_CUR 1
    `define SEEK_END 2

    integer i;
    integer file_size, file, tmp;
    initial begin
        file = $fopenr(`BINARY_TO_LOAD);

        file_size = $fseek(file, 0, `SEEK_END); /* End of file */
        file_size = $ftell(file);
        tmp = $fseek(file, 0, `SEEK_SET);
        tmp = $fread(internal_rom.mem, file, 0, file_size);

        $display("file size: %d", file_size);

        for (i = 0; i < (file_size/4); i = i + 1) begin
            internal_rom.mem[i] = {{internal_rom.mem[i][07:00]}, {internal_rom.mem[i][15:08]}, {internal_rom.mem[i][23:16]}, {internal_rom.mem[i][31:24]}};
            $display ("internal rom %d: %x", i, internal_rom.mem[i]);
        end

        // $dumpfile("leiwand_rv32_soc_hx8k_tb.vcd");
        // $dumpvars(0, leiwandrv32_soc_hx8k);

        // for (i = 0; i < `NR_RV_REGS; i = i + 1) begin
        //     $dumpvars(0, cpu_core.x[i]);
        // end

        $monitor("gpio_reg=%x\n",gpio_reg);
        //$monitor("pc: %x %x", cpu_core.pc, cpu_core.irq_return_pc);

        // # 150000 $finish;
    end

    always @(posedge system_clock) begin
        if(!resetn) clk_count <= `MEM_WIDTH'h0;
        else begin
            clk_count <= clk_count + 1;

            if(clk_count == 100000) begin
                $display("!!!!!!!!!!TRIGGER INTERRUPT=%x!!!!!!!!!\n",clk_count);
                irq_status_reg <= 1;
                clk_count <= 0;
            end
        end
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
    wire [(`MEM_WIDTH-1):0] irq_status_wire;
    wire irq_reset_flag_wire;

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

            irq_status_wire
    );


    wire ram_ready;
    wire [(`MEM_WIDTH-1):0] ram_rdata;

    simple_mem #(
        .WORDS(RAM_SIZE)
    ) internal_ram (
        .clk(system_clock),
        .rst(!resetn),
        .valid(mem_valid && (mem_addr >= `MEM_WIDTH'h20400000) && (mem_addr < (`MEM_WIDTH'h20400000 + (4*RAM_SIZE)))),
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

    wire rom_ready;
    wire [(`MEM_WIDTH-1):0] rom_rdata;

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

    wire [(`MEM_WIDTH-1):0] spimemio_cfgreg_do;

    spimemio spimemio (
        .clk    (system_clock),
        .resetn (resetn),
        .valid  (mem_valid && (mem_addr >= `MEM_WIDTH'h100000) && (mem_addr < (`MEM_WIDTH'h100000 + (4*FLASH_SIZE)))),
        .ready  (rom_ready),
        .addr   (mem_addr[23:0]),
        .rdata  (rom_rdata),

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
`else
    simple_mem #(
        .WORDS(FLASH_SIZE)
    ) internal_rom (
        .clk(system_clock),
        .rst(!resetn),
        .valid(mem_valid && (mem_addr >= `MEM_WIDTH'h100000) && (mem_addr < (`MEM_WIDTH'h100000 + (4*FLASH_SIZE)))),
        .ready(rom_ready),
        .wen(mem_wen),
        .addr(mem_addr[31:0]),
        .wdata(mem_data_cpu_out),
        .rdata(rom_rdata)
    );
`endif

    reg [(`MEM_WIDTH-1):0] gpio_reg;
    reg gpio_ready;

    always @(posedge system_clock) begin
        if(!resetn) gpio_reg <= `MEM_WIDTH'h0;
        else begin
            if(mem_valid && (mem_addr == `MEM_WIDTH'h30000000)) begin
                if(mem_wen != 0) gpio_reg <= mem_data_cpu_out;
                gpio_ready <= 1;
            end
            else gpio_ready <= 0;
        end
    end

    reg [(`MEM_WIDTH-1):0] irq_status_reg;
    reg irq_status_ready;
    reg irq_reset_flag;

    always @(posedge system_clock) begin
        if(!resetn) irq_status_reg <= `MEM_WIDTH'h0;
        else begin
            if(mem_valid && (mem_addr == `MEM_WIDTH'h40000000)) begin
                if(mem_wen != 0) irq_status_reg <= mem_data_cpu_out;
                irq_status_ready <= 1;
            end
            else begin 
                irq_status_ready <= 0;
            end
        end
    end

    assign mem_ready = ram_ready | rom_ready | gpio_ready | irq_status_ready;
    assign mem_data_cpu_in = ram_ready ? ram_rdata : rom_ready ? rom_rdata : gpio_ready ? gpio_reg : irq_status_ready ? irq_status_reg : 32'h 0000_0000;
    assign irq_status_wire = irq_status_reg;

    assign LED1 = gpio_reg[0];
    assign LED2 = gpio_reg[1];

endmodule
