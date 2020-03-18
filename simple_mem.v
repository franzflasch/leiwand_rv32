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

module simple_mem #(
    parameter integer WORDS = 256
) (
    input clk,
    input rst,
    input valid,
    output reg ready,
    input [3:0] wen,
    /* verilator lint_off UNUSED */
    input [31:0] addr,
    input [31:0] wdata,
    output reg [31:0] rdata
);
    reg [31:0] mem [WORDS-1:0];

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
                rdata <= mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]];
                if (wen[0]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][ 7: 0] <= wdata[ 7: 0];
                if (wen[1]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][15: 8] <= wdata[15: 8];
                if (wen[2]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][23:16] <= wdata[23:16];
                if (wen[3]) mem[addr[(`HIGH_BIT_TO_FIT(WORDS)-1)+2:2]][31:24] <= wdata[31:24];
                ready <= 1;
            end
            else begin
                ready <= 0;
            end
        end
    end
endmodule
