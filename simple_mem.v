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

module simple_mem #(
    parameter integer WORDS = 256,
    parameter integer WIDTH = 32
) (
    input clk,
    input rst,
    input valid,
    output reg ready,
    input [((`XLEN/8)-1):0] wen,
    /* verilator lint_off UNUSED */
    input [(WIDTH-1):0] addr,
    input [(WIDTH-1):0] wdata,
    output reg [(WIDTH-1):0] rdata
);
    reg [(WIDTH-1):0] mem [WORDS-1:0];

    always @(posedge clk) begin
        if(rst) begin
            // mem[0] <= 32'h00018eb7;
            // mem[1] <= 32'h00000f13;
            // mem[2] <= 32'h00000e13;
            // mem[3] <= 32'h001e0e13;
            // mem[4] <= 32'h01de0463;
            // mem[5] <= 32'hff9ff06f;
            // mem[6] <= 32'h001f0f13;
            // mem[7] <= 32'hfedff06f;
        end
        else begin
            if(valid) begin

                /* Check if we are in a odd access */
                `ifdef RV64
                    if(addr[2]) begin
                        rdata[31:0] <= mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]][63:32];
                        rdata[63:32] <= mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]+1][31:0];
                    end
                    else begin
                        rdata <= mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]];
                    end
                `else
                    rdata <= mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]];
                `endif

                `ifdef RV64
                    if (wen[0]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]][ 7: 0] <= wdata[ 7: 0];
                    if (wen[1]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]][15: 8] <= wdata[15: 8];
                    if (wen[2]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]][23:16] <= wdata[23:16];
                    if (wen[3]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]][31:24] <= wdata[31:24];
                    if (wen[4]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]][39:32] <= wdata[39:32];
                    if (wen[5]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]][47:40] <= wdata[47:40];
                    if (wen[6]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]][55:48] <= wdata[55:48];
                    if (wen[7]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+3:3]][63:56] <= wdata[63:56];
                `else
                    if (wen[0]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][ 7: 0] <= wdata[ 7: 0];
                    if (wen[1]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][15: 8] <= wdata[15: 8];
                    if (wen[2]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][23:16] <= wdata[23:16];
                    if (wen[3]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][31:24] <= wdata[31:24];
                `endif
                ready <= 1;
            end
            else begin
                ready <= 0;
            end
        end
    end
endmodule
