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

module leiwand_rv32_simple_mem #(
    parameter integer WORDS = 256
) (
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
    reg [(`MEM_WIDTH-1):0] mem [WORDS-1:0];

    always @(posedge clk) begin
        if(!rst) begin
            if(valid) begin

                /* Read Access */
                rdata[31:0] <= mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]][31:0];
                `ifdef RV64
                    rdata[63:32] <= mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]+1][31:0];
                `endif

                /* Write Access */
                if (wen[0]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]][ 7: 0] <= wdata[ 7: 0];
                if (wen[1]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]][15: 8] <= wdata[15: 8];
                if (wen[2]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]][23:16] <= wdata[23:16];
                if (wen[3]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]][31:24] <= wdata[31:24];
                `ifdef RV64
                    if (wen[4]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]+1][ 7: 0] <= wdata[39:32];
                    if (wen[5]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]+1][15: 8] <= wdata[47:40];
                    if (wen[6]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]+1][23:16] <= wdata[55:48];
                    if (wen[7]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+`MEM_OFFSET:`MEM_OFFSET]+1][31:24] <= wdata[63:56];
                `endif
                ready <= 1;
            end
            else begin
                ready <= 0;
            end
        end
    end
endmodule
