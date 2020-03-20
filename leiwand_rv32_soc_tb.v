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

module leiwand_rv32_core_tb();

    parameter MEMORY_SIZE = 4096;

    reg clk = 0;
    reg reset = 0;

    wire mem_valid;
    wire mem_ready;
    wire [(`XLEN-1):0] mem_addr;
    wire [(`XLEN-1):0] mem_data_cpu_in;
    wire [(`XLEN-1):0] mem_data_cpu_out;
    wire [3:0] mem_wen;

    wire [(`XLEN-1):0] dummy_irq_status;

    leiwand_rv32_core
        cpu_core (
            clk,
            reset,

            mem_valid,
            mem_ready,
            mem_addr,
            mem_data_cpu_in,
            mem_data_cpu_out,
            mem_wen
    );

    simple_mem #(
        .WORDS(MEMORY_SIZE),
        .WIDTH(`XLEN)
    ) internal_rom (
        .clk(clk),
        .rst(reset),

        .valid(mem_valid && (mem_addr >= `XLEN'h80000000) && (mem_addr < `XLEN'h80000000 + (4*MEMORY_SIZE))),
        .ready(mem_ready),
        .wen(mem_wen),
        .addr(mem_addr[(`XLEN-1):0]),
        .wdata(mem_data_cpu_out[(`XLEN-1):0]),
        .rdata(mem_data_cpu_in[(`XLEN-1):0])
    );

    initial begin
        clk=0;
        forever #2 clk=~clk;
    end

    `define SEEK_SET 0
    `define SEEK_CUR 1
    `define SEEK_END 2

    integer i, j;
    integer file_size, file, tmp;
    reg [(`XLEN-1):0] tmp_mem [MEMORY_SIZE-1:0];

    initial begin

        file = $fopenr(`BINARY_TO_LOAD);

        file_size = $fseek(file, 0, `SEEK_END); /* End of file */
        file_size = $ftell(file);
        tmp = $fseek(file, 0, `SEEK_SET);
        tmp = $fread(tmp_mem, file, 0, file_size);

        $display("file size: %d", file_size);

        for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
            //internal_rom.mem[i] = { {(`XLEN-32){1'b0}}, {tmp_mem[i][07:00]}, {tmp_mem[i][15:08]}, {tmp_mem[i][23:16]}, {tmp_mem[i][31:24]} };
            `ifdef RV64
                internal_rom.mem[i][63:32] = { {tmp_mem[i][07:00]}, {tmp_mem[i][15:08]}, {tmp_mem[i][23:16]}, {tmp_mem[i][31:24]} };
                internal_rom.mem[i][31:0] = {{tmp_mem[i][39:32]}, {tmp_mem[i][47:40]}, {tmp_mem[i][55:48]}, {tmp_mem[i][63:56]}};
            `else
                internal_rom.mem[i] = { {(`XLEN-32){1'b0}}, {tmp_mem[i][07:00]}, {tmp_mem[i][15:08]}, {tmp_mem[i][23:16]}, {tmp_mem[i][31:24]} };
            `endif
            $display ("internal ram %d: %x", i, internal_rom.mem[i]);
        end

        $display ("clk: %d", clk);
        #5
        reset=0;
        #10
        reset=1;
        #5
        reset=0;

        for (i = 0; i < 10000; i++) begin
            wait (cpu_core.cpu_stage == cpu_core.STAGE_INSTR_FETCH);
            wait (cpu_core.cpu_stage == cpu_core.STAGE_INSTR_ALU_PREPARE);

            $display("\n");
            $display("cycle: %d", i);
            $display("stage: %d", cpu_core.cpu_stage);
            $display("pc: %x", cpu_core.pc);
            $display("instr: %x", cpu_core.instruction);

            for(j = 0; j < `NR_RV_REGS; j++ ) begin
                $display("x[%2d]: %x", j, cpu_core.x[j]);
            end

            if(cpu_core.pc == `SUCCESS_PC) begin
                $display("SUCCESS!");
                $finish;
            end

        end

        $display("SOMETHING WENT WRONG!");
        $finish;
    end

    // initial begin
    //     $dumpfile("leiwand_rv32_soc_tb.vcd");

    //     $dumpvars(0,leiwand_rv32_core_tb);
    //     for (i = 0; i < `NR_RV_REGS; i = i + 1) begin
    //         $dumpvars(0, cpu_core.x[i]);
    //     end

    //     // for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
    //     //     $dumpvars(0, internal_rom.mem[i]);
    //     // end

    //     // # 15000 $finish;
    // end

endmodule
