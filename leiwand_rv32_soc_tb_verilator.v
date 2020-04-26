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

module leiwand_rv32_soc_tb_verilator(
    input i_clk,
    input i_rst
);

    /* 32KiB */
    parameter RAM_SIZE_BYTES /*verilator public_flat_rw*/ = 'h1000;
    parameter RAM_BASE_ADDR /*verilator public_flat_rw*/ = `XLEN'h80000000;

    wire mem_valid;
    wire mem_ready;
    wire [(`XLEN-1):0] mem_addr;
    wire [(`XLEN-1):0] mem_data_cpu_in;
    wire [(`XLEN-1):0] mem_data_cpu_out;
    wire [((`XLEN/8)-1):0] mem_wen;

    wire [12:0] irq;

    leiwand_rv32_core #(
        .PC_START_VAL(RAM_BASE_ADDR)
    ) cpu_core (
            .i_clk(i_clk),
            .i_rst(i_rst),

            .o_mem_valid(mem_valid),
            .i_mem_ready(mem_ready),
            .o_mem_addr(mem_addr),
            .i_mem_data(mem_data_cpu_in),
            .o_mem_data(mem_data_cpu_out),
            .o_mem_wen(mem_wen),

            .i_irq(irq)
    );

    wire ram_ready;
    wire [(`XLEN-1):0] ram_rdata;

    leiwand_rv32_simple_mem #(
        .WORDS(RAM_SIZE_BYTES/`MEM_WIDTH_BYTES)
    ) internal_ram (
        .clk(i_clk),
        .rst(i_rst),

        .valid(mem_valid && (mem_addr >= RAM_BASE_ADDR) && (mem_addr < RAM_BASE_ADDR + (RAM_SIZE_BYTES))),
        .ready(ram_ready),
        .wen(mem_wen),
        .addr(mem_addr[(`XLEN-1):0]),
        .wdata(mem_data_cpu_out[(`XLEN-1):0]),
        .rdata(ram_rdata[(`XLEN-1):0])
    );

    wire clint_ready;
    wire [(`XLEN-1):0] clint_rdata;

    leiwand_rv32_clint clint (
        .clk(i_clk),
        .rst(i_rst),

        .valid(mem_valid && (mem_addr >= `XLEN'h2000000) && (mem_addr <= `XLEN'h200FFFF)),
        .ready(clint_ready),
        .wen(mem_wen),
        .addr(mem_addr[(`XLEN-1):0]),
        .wdata(mem_data_cpu_out[(`XLEN-1):0]),
        .rdata(clint_rdata[(`XLEN-1):0]),
        .irq_out(irq)
    );

    wire uart_ready;
    wire [(`XLEN-1):0] uart_rdata;

    leiwand_rv32_simple_uart uart (
        .clk(i_clk),
        .rst(i_rst),

        .valid(mem_valid && (mem_addr >= `XLEN'h10000000) && (mem_addr <= `XLEN'h10000008)),
        .ready(uart_ready),
        .wen(mem_wen),
        .addr(mem_addr[(`XLEN-1):0]),
        .wdata(mem_data_cpu_out[(`XLEN-1):0]),
        .rdata(uart_rdata[(`XLEN-1):0])
    );

    assign mem_ready = ram_ready | clint_ready | uart_ready;
    assign mem_data_cpu_in = ram_ready ? ram_rdata : clint_ready ? clint_rdata : uart_ready ? uart_rdata : `XLEN'h 0000_0000;

endmodule
