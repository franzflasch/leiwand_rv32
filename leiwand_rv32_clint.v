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

`include "leiwand_rv32_constants.v"

`define MSIP_REG_OFFS 'h0000
`define MTIMECMP_REG_OFFS 'h4000
`define MTIME_REG_OFFS 'hBFF8

`define MSIP_ADDR_INTERNAL 0
`define MTIMECMP_ADDR_INTERNAL 1
`define MTIME_ADDR_INTERNAL 2

`define read_write_mem(reg_addr)\
    begin \
        /* Read Access */ \
        rdata <= mem[`MSIP_ADDR_INTERNAL]; \
        /* Write Access */ \
        if (wen[0]) mem[reg_addr][ 7: 0] <= wdata[ 7: 0]; \
        if (wen[1]) mem[reg_addr][15: 8] <= wdata[15: 8]; \
        if (wen[2]) mem[reg_addr][23:16] <= wdata[23:16]; \
        if (wen[3]) mem[reg_addr][31:24] <= wdata[31:24]; \
        `ifdef RV64 \
            if (wen[4]) mem[reg_addr][39:32] <= wdata[39:32]; \
            if (wen[5]) mem[reg_addr][47:40] <= wdata[47:40]; \
            if (wen[6]) mem[reg_addr][55:48] <= wdata[55:48]; \
            if (wen[7]) mem[reg_addr][63:56] <= wdata[63:56]; \
        `endif \
    end

module leiwand_rv32_clint (
    input clk,
    input rst,
    input valid,
    output reg ready,
    input [((`XLEN/8)-1):0] wen,
    /* verilator lint_off UNUSED */
    input [(`XLEN-1):0] addr,
    input [(`XLEN-1):0] wdata,
    output reg [(`XLEN-1):0] rdata
);
    parameter integer WORDS = 3;
    reg [(`XLEN-1):0] mem [WORDS-1:0];

    always @(posedge clk) begin
        if(!rst) begin
            if(valid) begin
                case (addr[15:0])
                    `MSIP_REG_OFFS: `read_write_mem(`MSIP_ADDR_INTERNAL)
                    `MTIMECMP_REG_OFFS: `read_write_mem(`MTIMECMP_ADDR_INTERNAL)
                    `MTIME_REG_OFFS: `read_write_mem(`MTIME_ADDR_INTERNAL)
                    default: rdata <= 0;
                endcase
                ready <= 1;
            end
            else begin
                ready <= 0;
            end
        end
    end
endmodule
