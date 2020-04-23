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

`define TXD_REG_OFFS 'h0000
`define RXD_REG_OFFS 'h0008

module leiwand_rv32_simple_uart (
    input clk,
    input rst,
    input valid,
    output reg ready,
    /* verilator lint_off UNUSED */
    input [((`XLEN/8)-1):0] wen,
    input [(`XLEN-1):0] addr,
    input [(`XLEN-1):0] wdata,
    output reg [(`XLEN-1):0] rdata
);
    always @(posedge clk) begin
        if(!rst) begin
            if(valid) begin
                case (addr[15:0])
                    `TXD_REG_OFFS: begin
                        if(wen[0] && !ready) begin
                            $write("%c", wdata[7:0]);
                        end
                    end
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
