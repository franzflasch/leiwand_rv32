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

    parameter MEMORY_SIZE = 4096;

    wire mem_valid;
    wire mem_ready;
    wire [(`XLEN-1):0] mem_addr;
    wire [(`XLEN-1):0] mem_data_cpu_in;
    wire [(`XLEN-1):0] mem_data_cpu_out;
    wire [((`XLEN/8)-1):0] mem_wen;

    leiwand_rv32_core
        cpu_core (
            .i_clk(i_clk),
            .i_rst(i_rst),

            .o_mem_valid(mem_valid),
            .i_mem_ready(mem_ready),
            .o_mem_addr(mem_addr),
            .i_mem_data(mem_data_cpu_in),
            .o_mem_data(mem_data_cpu_out),
            .o_mem_wen(mem_wen)
    );

    simple_mem #(
        .WORDS(MEMORY_SIZE),
        .WIDTH(`XLEN)
    ) internal_rom (
        .clk(i_clk),
        .rst(i_rst),

        .valid(mem_valid && (mem_addr >= `XLEN'h80000000) && (mem_addr < `XLEN'h80000000 + ((`XLEN/8)*MEMORY_SIZE))),
        .ready(mem_ready),
        .wen(mem_wen),
        .addr(mem_addr[(`XLEN-1):0]),
        .wdata(mem_data_cpu_out[(`XLEN-1):0]),
        .rdata(mem_data_cpu_in[(`XLEN-1):0])
    );

endmodule
