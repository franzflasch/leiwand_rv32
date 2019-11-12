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

//`define DEBUG

`ifdef DEBUG
  `define debug(debug_command) debug_command
`else
  `define debug(debug_command)
`endif


`define ENABLE_CSR_REGS


module leiwand_rv32_core # (
        parameter PC_START_VAL = `MEM_WIDTH'h20400000 /* HiFive1 Value */
    )
    (
        input i_clk,
        input i_rst,

        /* MEMORY INTERFACE */
        output o_mem_valid,
        input i_mem_ready,
        output [(`MEM_WIDTH-1):0] o_mem_addr,
        input [(`MEM_WIDTH-1):0] i_mem_data,
        output [(`MEM_WIDTH-1):0] o_mem_data,
        output [3:0] o_mem_wen,

        input [(`MEM_WIDTH-1):0] irq_status
    );

    `ifdef ENABLE_CSR_REGS
        /* CSR instructions */
        localparam OP_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI = 7'b1110011;
        localparam FUNC3_CSRRW = 3'b001;
        localparam FUNC3_CSRRS = 3'b010;
        localparam FUNC3_CSRRC = 3'b011;
        localparam FUNC3_CSRRWI = 3'b101;
        localparam FUNC3_CSRRSI = 3'b110;
        localparam FUNC3_CSRRCI = 3'b111;

        reg is_CSRRW;
        reg is_CSRRS;
        reg is_CSRRC;        
        reg is_CSRRWI;
        reg is_CSRRSI;
        reg is_CSRRCI;

        /* CSR Registers */
        /* Trap Setup */
        localparam MSTATUS = 0;
        localparam MIE = 1;
        localparam MTVEC = 2;
        /* Trap Handling */
        localparam MSCRATCH = 3;
        localparam MEPC = 4;
        localparam MCAUSE = 5;
        localparam MTVAL = 6;
        localparam MIP = 7;

        reg [(`MEM_WIDTH-1):0] csr[(MIP-1):0];

        //function automatic [(`MEM_WIDTH-1):0] csr_reg_val;
        function [`HIGH_BIT_TO_FIT(MIP):0] csr_reg_to_internal_index;
            input [11:0] i_reg_nr;
            begin
                case (i_reg_nr)
                        12'h300: begin
                            csr_reg_to_internal_index = MSTATUS;
                        end
                        12'h304: begin
                            csr_reg_to_internal_index = MIE;
                        end
                        12'h305: begin
                            csr_reg_to_internal_index = MTVEC;
                        end
                        12'h340: begin
                            csr_reg_to_internal_index = MSCRATCH;
                        end
                        12'h341: begin
                            csr_reg_to_internal_index = MEPC;
                        end
                        12'h342: begin
                            csr_reg_to_internal_index = MCAUSE;
                        end
                        12'h343: begin
                            csr_reg_to_internal_index = MTVAL;
                        end
                        12'h344: begin
                            csr_reg_to_internal_index = MIP;
                        end
                        default: csr_reg_to_internal_index = 0;
                endcase
            end
        endfunction
    `endif

    /* Currently 4 Stages (not pipelined) */
    localparam STAGE_INSTR_FETCH = 0;
    localparam STAGE_INSTR_DECODE = 1;
    localparam STAGE_INSTR_ALU_PREPARE = 2;
    localparam STAGE_INSTR_ALU_EXECUTE = 3;
    localparam STAGE_INSTR_ACCESS = 4;
    localparam STAGE_INSTR_WRITEBACK = 5;
    reg [`HIGH_BIT_TO_FIT(STAGE_INSTR_WRITEBACK):0] cpu_stage;

    /* RISC-V Registers x0-x31 */
    reg [(`MEM_WIDTH-1):0] x[(`NR_RV_REGS-1):0];
    reg [(`MEM_WIDTH-1):0] pc;
    reg [(`MEM_WIDTH-1):0] instruction;

    /* opcode registers */
    reg [(`MEM_WIDTH-1):0] next_pc;
    reg [4:0] rs1;
    reg [4:0] rs2_shamt;
    reg [4:0] rd;
    reg [(`MEM_WIDTH-1):0] immediate;

    /* bus signals */
    reg mem_valid;
    reg [(`MEM_WIDTH-1):0] mem_addr_out;
    reg [(`MEM_WIDTH-1):0] mem_data_in;
    reg [(`MEM_WIDTH-1):0] mem_data_out;
    reg [3:0] mem_wen;
    reg mem_access;

    /* RV32I Base instructions */
    localparam OP_LUI = 7'b0110111;
    localparam OP_AUIPC = 7'b0010111;

    localparam OP_JAL = 7'b1101111;

    localparam OP_JALR = 7'b1100111;
    localparam FUNC3_JALR = 3'b000;

    localparam OP_BEQ_BNE_BLT_BGE_BLTU_BGEU = 7'b1100011;
    localparam FUNC3_BEQ = 3'b000;
    localparam FUNC3_BNE = 3'b001;
    localparam FUNC3_BLT = 3'b100;
    localparam FUNC3_BGE = 3'b101;
    localparam FUNC3_BLTU = 3'b110;
    localparam FUNC3_BGEU = 3'b111;

    localparam OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI = 7'b0010011;
    localparam FUNC3_ADDI = 3'b000;
    localparam FUNC3_SLTI = 3'b010;
    localparam FUNC3_SLTIU = 3'b011;
    localparam FUNC3_XORI = 3'b100;
    localparam FUNC3_ORI = 3'b110;
    localparam FUNC3_ANDI = 3'b111;
    localparam FUNC3_SLLI = 3'b001;
    localparam FUNC7_SLLI = 7'b0000000;
    localparam FUNC3_SRLI = 3'b101;
    localparam FUNC7_SRLI = 7'b0000000;
    localparam FUNC3_SRAI = 3'b101;
    localparam FUNC7_SRAI = 7'b0100000;

    localparam OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND = 7'b0110011;
    localparam FUNC3_ADD = 3'b000;
    localparam FUNC7_ADD = 7'b0000000;
    localparam FUNC3_SUB = 3'b000;
    localparam FUNC7_SUB = 7'b0100000;
    localparam FUNC3_SLL = 3'b001;
    localparam FUNC7_SLL = 7'b0000000;
    localparam FUNC3_SLT = 3'b010;
    localparam FUNC7_SLT = 7'b0000000;
    localparam FUNC3_SLTU = 3'b011;
    localparam FUNC7_SLTU = 7'b0000000;
    localparam FUNC3_XOR = 3'b100;
    localparam FUNC7_XOR = 7'b0000000;
    localparam FUNC3_SRL = 3'b101;
    localparam FUNC7_SRL = 7'b0000000;
    localparam FUNC3_SRA = 3'b101;
    localparam FUNC7_SRA = 7'b0100000;
    localparam FUNC3_OR = 3'b110;
    localparam FUNC7_OR = 7'b0000000;
    localparam FUNC3_AND = 3'b111;
    localparam FUNC7_AND = 7'b0000000;

    localparam OP_LB_LH_LW_LBU_LHU = 7'b0000011;
    localparam FUNC3_LB = 3'b000;
    localparam FUNC3_LH = 3'b001;
    localparam FUNC3_LW = 3'b010;
    localparam FUNC3_LBU = 3'b100;
    localparam FUNC3_LHU = 3'b101;

    localparam OP_SB_SH_SW = 7'b0100011;
    localparam FUNC3_SB = 3'b000;
    localparam FUNC3_SH = 3'b001;
    localparam FUNC3_SW = 3'b010;

    reg is_LUI;
    reg is_AUIPC;
    reg is_JAL;
    reg is_JALR;
    reg is_BEQ, is_BNE, is_BLT, is_BGE, is_BLTU, is_BGEU;
    reg is_ADDI, is_SLTI, is_SLTIU, is_XORI, is_ORI, is_ANDI, is_SLLI, is_SRLI, is_SRAI;
    reg is_ADD, is_SUB, is_SLL, is_SLT, is_SLTU, is_XOR, is_SRL, is_SRA, is_OR, is_AND;
    reg is_LB, is_LH, is_LW, is_LBU, is_LHU;
    reg is_SB, is_SH, is_SW;

    /* ALU */
    reg alu_result_slt;
    reg alu_result_sltu;
    reg [(`MEM_WIDTH-1):0] alu_result_xor;
    reg [(`MEM_WIDTH-1):0] alu_result_or;
    reg [(`MEM_WIDTH-1):0] alu_result_and;
    reg [(`MEM_WIDTH-1):0] alu_result_sl;
    reg [(`MEM_WIDTH):0] alu_result_sr;
    reg [(`MEM_WIDTH-1):0] alu_result_sub;
    reg [(`MEM_WIDTH-1):0] alu_result_add;
    reg [(`MEM_WIDTH-1):0] alu_op1;
    reg [(`MEM_WIDTH-1):0] alu_op2;

    reg [(`MEM_WIDTH-1):0] alu_branch_op1;
    reg [(`MEM_WIDTH-1):0] alu_branch_op2;
    reg alu_branch_eq;
    reg alu_branch_ge;
    reg alu_branch_geu;

    /* CPU Core */
    always @(posedge i_clk) begin
        if(i_rst) begin
            /* Initialize all general purpose regs */
            x[0]  <= 0; 
            x[1]  <= 0;
            x[2]  <= 0;
            x[3]  <= 0;
            x[4]  <= 0;
            x[5] <= PC_START_VAL;
            x[6]  <= 0;
            x[7]  <= 0;
            x[8]  <= 0;
            x[9]  <= 0;
            x[10] <= 0;
            x[11] <= 0;
            x[12] <= 0;
            x[13] <= 0;
            x[14] <= 0;
            x[15] <= 0;
            x[16] <= 0;
            x[17] <= 0;
            x[18] <= 0;
            x[19] <= 0;
            x[20] <= 0;
            x[21] <= 0;
            x[22] <= 0;
            x[23] <= 0;
            x[24] <= 0;
            x[25] <= 0;
            x[26] <= 0;
            x[27] <= 0;
            x[28] <= 0;
            x[29] <= 0;
            x[30] <= 0;
            x[31] <= 0;

            /* Initialize program counter */
            pc <= PC_START_VAL;
            next_pc <= PC_START_VAL;
            instruction <= 0;

            /* First stage is instruction */
            cpu_stage <= STAGE_INSTR_FETCH;

            mem_addr_out <= 0;
            mem_data_out <= 0;
            mem_valid <= 0;
            mem_wen <= 0;

            mem_access <= 0;
        end
        else begin

            /* New data ready */
            if(i_mem_ready && mem_valid && mem_access) begin
                mem_data_in <= i_mem_data;
                mem_access <= 0;
                mem_valid <= 0;
                mem_wen <= 0;
            end

            if (!mem_access) begin

                case (cpu_stage)

                        STAGE_INSTR_FETCH: begin
                            mem_addr_out <= next_pc;
                            mem_access <= 1;
                            mem_valid <= 1;

                            pc <= next_pc;
                            next_pc <= next_pc + 4;
                            cpu_stage <= STAGE_INSTR_DECODE;
                        end

                        /* Decode next instruction */
                        STAGE_INSTR_DECODE: begin

                            rs1[4:0] <= mem_data_in[19:15];
                            rs2_shamt[4:0] <= mem_data_in[24:20];
                            rd[4:0] <= mem_data_in[11:7];

                            is_LUI <= (mem_data_in[6:0] == OP_LUI) ? 1 : 0;
                            is_AUIPC <= (mem_data_in[6:0] == OP_AUIPC) ? 1 : 0;
                            is_JAL <= (mem_data_in[6:0] == OP_JAL) ? 1 : 0;
                            is_JALR <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_JALR, OP_JALR} ) ? 1 : 0;

                            is_BEQ <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_BEQ, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BNE <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_BNE, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BLT <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_BLT, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BGE <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_BGE, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BLTU <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_BLTU, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BGEU <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_BGEU, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;

                            is_ADDI <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_ADDI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SLTI <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_SLTI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SLTIU <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_SLTIU, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_XORI <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_XORI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_ORI <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_ORI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_ANDI <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_ANDI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SLLI <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_SLLI, FUNC3_SLLI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SRLI <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_SRLI, FUNC3_SRLI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SRAI <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_SRAI, FUNC3_SRAI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;

                            is_ADD <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_ADD, FUNC3_ADD, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SUB <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_SUB, FUNC3_SUB, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SLL <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_SLL, FUNC3_SLL, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SLT <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_SLT, FUNC3_SLT, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SLTU <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_SLTU, FUNC3_SLTU, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_XOR <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_XOR, FUNC3_XOR, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SRL <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_SRL, FUNC3_SRL, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SRA <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_SRA, FUNC3_SRA, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_OR <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_OR, FUNC3_OR, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_AND <= ({mem_data_in[31:25],mem_data_in[14:12],mem_data_in[6:0]} == {FUNC7_AND, FUNC3_AND, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;

                            is_LB <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_LB, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;
                            is_LH <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_LH, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;
                            is_LW <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_LW, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;
                            is_LBU <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_LBU, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;
                            is_LHU <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_LHU, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;

                            is_SB <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_SB, OP_SB_SH_SW} ) ? 1 : 0;
                            is_SH <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_SH, OP_SB_SH_SW} ) ? 1 : 0;
                            is_SW <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_SW, OP_SB_SH_SW} ) ? 1 : 0;

                            `ifdef ENABLE_CSR_REGS
                                is_CSRRW <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_CSRRW, OP_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI} ) ? 1 : 0;
                                is_CSRRS <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_CSRRS, OP_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI} ) ? 1 : 0;
                                is_CSRRC <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_CSRRC, OP_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI} ) ? 1 : 0;
                                is_CSRRWI <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_CSRRWI, OP_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI} ) ? 1 : 0;
                                is_CSRRSI <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_CSRRSI, OP_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI} ) ? 1 : 0;
                                is_CSRRCI <= ({mem_data_in[14:12],mem_data_in[6:0]} == {FUNC3_CSRRCI, OP_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI} ) ? 1 : 0;
                            `endif

                            case (mem_data_in[6:0])

                                /* S-type */
                                OP_SB_SH_SW: begin
                                    immediate <= { {20{mem_data_in[31]}}, mem_data_in[31:25], mem_data_in[11:7] };
                                end

                                /* B-type */
                                OP_BEQ_BNE_BLT_BGE_BLTU_BGEU: begin
                                    immediate <= { {20{mem_data_in[31]}}, mem_data_in[7], mem_data_in[30:25], mem_data_in[11:8], 1'b0 };
                                end

                                /* U-type */
                                OP_LUI,
                                OP_AUIPC: begin
                                    immediate <= { mem_data_in[31:12], 12'b0};
                                end

                                /* J-type */
                                OP_JAL: begin
                                    immediate <= { {11{mem_data_in[31]}}, mem_data_in[19:12], mem_data_in[20], mem_data_in[31:21], 1'b0 };
                                end

                                /* I-type */
                                default: immediate <= { {20{mem_data_in[31]}}, mem_data_in[31:20] };

                            endcase

                            instruction <= mem_data_in;
                            cpu_stage <= STAGE_INSTR_ALU_PREPARE;
                        end

                        STAGE_INSTR_ALU_PREPARE: begin

                            alu_branch_op1 <= x[rs1];
                            alu_branch_op2 <= x[rs2_shamt];

                            if(is_auipc_jal_op || is_branch_op) begin
                               alu_op1 <= pc;
                            end
                            else begin
                                alu_op1 <= x[rs1];
                            end

                            if(is_alu_shift_immediate) begin
                                alu_op2 <= { {27{1'b0}}, rs2_shamt };
                            end
                            else if(is_alu_logic || is_alu_shift) begin
                                alu_op2 <= x[rs2_shamt];
                            end
                            else alu_op2 <= immediate;

                            cpu_stage <= STAGE_INSTR_ALU_EXECUTE;
                        end

                        STAGE_INSTR_ALU_EXECUTE: begin

                            alu_result_slt <= $signed(alu_op1) < $signed(alu_op2);
                            alu_result_sltu <= alu_op1 < alu_op2;
                            alu_result_xor <= alu_op1 ^ alu_op2;
                            alu_result_or <= alu_op1 | alu_op2;
                            alu_result_and <= alu_op1 & alu_op2;
                            alu_result_sl <= alu_op1 << alu_op2[4:0];
                            alu_result_sr <= $signed({is_SRA || is_SRAI ? alu_op1[31] : 1'b0, alu_op1}) >>> alu_op2[4:0];
                            alu_result_sub <= alu_op1 - alu_op2;
                            alu_result_add <= alu_op1 + alu_op2;

                            alu_branch_eq <= (alu_branch_op1 == alu_branch_op2);
                            alu_branch_ge <= ($signed(alu_branch_op1) >= $signed(alu_branch_op2));
                            alu_branch_geu <= (alu_branch_op1 >= alu_branch_op2);

                            cpu_stage <= (is_load | is_store) ? STAGE_INSTR_ACCESS : STAGE_INSTR_WRITEBACK;
                        end

                        STAGE_INSTR_ACCESS: begin
                            mem_addr_out <= alu_result;
                            mem_data_out <= x[rs2_shamt];

                            if(is_SB) begin
                                case (alu_result[1:0])
                                    1: begin mem_data_out[15:8] <= x[rs2_shamt][7:0]; mem_wen <= 4'b0010; end
                                    2: begin mem_data_out[23:16] <= x[rs2_shamt][7:0]; mem_wen <= 4'b0100; end
                                    3: begin mem_data_out[31:24] <= x[rs2_shamt][7:0]; mem_wen <= 4'b1000; end
                                    default: begin mem_data_out[7:0] <= x[rs2_shamt][7:0]; mem_wen <= 4'b0001; end
                                endcase
                            end
                            if(is_SH) begin
                                case (alu_result[1])
                                    1: begin mem_data_out[31:16] <= x[rs2_shamt][15:0]; mem_wen <= 4'b1100; end
                                    default: begin mem_data_out[15:0] <= x[rs2_shamt][15:0]; mem_wen <= 4'b0011; end
                                endcase
                            end
                            if(is_SW) begin mem_data_out[31:0] <= x[rs2_shamt][31:0]; mem_wen <= 4'b1111; end

                            mem_access <= 1;
                            mem_valid <= 1;
                            
                            cpu_stage <= (is_store) ?  STAGE_INSTR_FETCH : STAGE_INSTR_WRITEBACK;
                        end

                        STAGE_INSTR_WRITEBACK: begin

                            if (is_LUI) x[rd] <= immediate;
                            else if(is_branch_op) begin
                                if(take_branch) begin 
                                    next_pc <= alu_result; 
                                end
                            end
                            else if (is_JAL || is_JALR) begin 
                                x[rd] <= next_pc; 
                                next_pc <= alu_result; 
                            end
                            else if (is_LB) begin
                                case (mem_addr_out[1:0])
                                    1: x[rd] <= { {24{mem_data_in[15]}}, mem_data_in[15:8] };
                                    2: x[rd] <= { {24{mem_data_in[23]}},mem_data_in[23:16] };
                                    3: x[rd] <= { {24{mem_data_in[31]}},mem_data_in[31:24] };
                                    default: x[rd] <= { {24{mem_data_in[7]}},mem_data_in[7:0] };
                                endcase
                            end
                            else if (is_LH) begin
                                x[rd] <= (mem_addr_out[1]) ? {{16{mem_data_in[31]}},mem_data_in[31:16]} : {{16{mem_data_in[15]}},mem_data_in[15:0]};
                            end
                            else if (is_LW) begin
                                x[rd] <= mem_data_in;
                            end
                            else if (is_LBU) begin
                                case (mem_addr_out[1:0])
                                    1: x[rd] <= { {24{1'b0}},mem_data_in[15:8] };
                                    2: x[rd] <= { {24{1'b0}},mem_data_in[23:16] };
                                    3: x[rd] <= { {24{1'b0}},mem_data_in[31:24] };
                                    default: x[rd] <= { {24{1'b0}},mem_data_in[7:0] };
                                endcase
                            end
                            else if (is_LHU) begin
                                x[rd] <= { {16{1'b0}}, (mem_addr_out[1]) ? mem_data_in[31:16] : mem_data_in[15:0] };
                            end
                            `ifdef ENABLE_CSR_REGS
                                else if (is_csr_op) begin
                                    x[rd] <= csr[csr_reg_to_internal_index(immediate[11:0])];
                                    if (is_CSRRW | is_CSRRWI) begin
                                        csr[csr_reg_to_internal_index(immediate[11:0])] <= is_CSRRWI ? { {(`MEM_WIDTH-5){1'b0}}, rs1[4:0] } : x[rs1[4:0]];
                                    end
                                    else if (is_CSRRS | is_CSRRSI) begin
                                        csr[csr_reg_to_internal_index(immediate[11:0])] <= is_CSRRSI ? 
                                                                                           csr[csr_reg_to_internal_index(immediate[11:0])] | { {(`MEM_WIDTH-5){1'b0}}, rs1[4:0] } : 
                                                                                           csr[csr_reg_to_internal_index(immediate[11:0])] | x[rs1[4:0]];
                                    end
                                    else if (is_CSRRC | is_CSRRCI) begin
                                        csr[csr_reg_to_internal_index(immediate[11:0])] <= is_CSRRCI ? 
                                                                                           csr[csr_reg_to_internal_index(immediate[11:0])] & (~{ {(`MEM_WIDTH-5){1'b0}}, rs1[4:0] }) : 
                                                                                           csr[csr_reg_to_internal_index(immediate[11:0])] & ~x[rs1[4:0]];
                                    end
                                end
                            `endif
                            else x[rd] <= alu_result;

                            cpu_stage <= STAGE_INSTR_FETCH;
                        end

                        default: begin
                            cpu_stage <= STAGE_INSTR_FETCH;
                            `debug($display("Invalid CPU stage!");)
                        end

                endcase

            end

            /* reset x0 to zero, as theoretically in this implementation it can be set to any value */
            x[0] <= 0; 
        end
    end

    wire is_auipc_jal_op;
    assign is_auipc_jal_op = (is_AUIPC | is_JAL);

    wire is_branch_op;
    assign is_branch_op = (is_BEQ | is_BNE | is_BLT | is_BGE | is_BLTU | is_BGEU);

    wire take_branch;
    assign take_branch = ( (is_BEQ && alu_branch_eq) || 
                           (is_BNE && !alu_branch_eq) || 
                           (is_BLT && !alu_branch_ge) ||
                           (is_BGE && alu_branch_ge) || 
                           (is_BLTU && !alu_branch_geu) ||
                           (is_BGEU && alu_branch_geu) );

    wire is_alu_shift_immediate;
    assign is_alu_shift_immediate = (is_SLLI | is_SRLI | is_SRAI);

    wire is_alu_logic;
    assign is_alu_logic = (is_ADD | is_SUB | is_SLT | is_SLTU | is_XOR | is_OR | is_AND);

    wire is_alu_shift;
    assign is_alu_shift = (is_SLL | is_SRL | is_SRA);

    wire is_load;
    assign is_load = (is_LB | is_LH | is_LW | is_LBU | is_LHU);

    wire is_store;
    assign is_store = (is_SB | is_SH | is_SW);

    wire alu_is_slt_op;
    wire alu_is_sltu_op;
    wire alu_is_xor_op;
    wire alu_is_or_op;
    wire alu_is_and_op;
    wire alu_is_sl_op;
    wire alu_is_sr_op;
    wire alu_is_sub_op;
    wire [(`MEM_WIDTH-1):0] alu_result;

    assign alu_result = ( alu_is_slt_op ? { {31{1'b0}},alu_result_slt } :
                          alu_is_sltu_op ? { {31{1'b0}},alu_result_sltu } :
                          alu_is_xor_op ? alu_result_xor :
                          alu_is_or_op ? alu_result_or :
                          alu_is_and_op ? alu_result_and :
                          alu_is_sl_op ? alu_result_sl :
                          alu_is_sr_op ? alu_result_sr[31:0] :
                          alu_is_sub_op ? alu_result_sub :
                          alu_result_add );

    assign alu_is_slt_op = (is_SLT | is_SLTI);
    assign alu_is_sltu_op = (is_SLTU | is_SLTIU);
    assign alu_is_xor_op = (is_XOR | is_XORI);
    assign alu_is_or_op = (is_OR | is_ORI);
    assign alu_is_and_op = (is_AND | is_ANDI);
    assign alu_is_sl_op = (is_SLL | is_SLLI);
    assign alu_is_sr_op = (is_SRA | is_SRAI | is_SRL | is_SRLI);
    assign alu_is_sub_op = (is_SUB);

    assign o_mem_valid = mem_valid;
    assign o_mem_addr = mem_addr_out;
    assign o_mem_data = mem_data_out;
    assign o_mem_wen = mem_wen;

    `ifdef ENABLE_CSR_REGS
    wire is_csr_op;
    assign is_csr_op = (is_CSRRC | is_CSRRCI | is_CSRRS | is_CSRRSI | is_CSRRW | is_CSRRWI);
    `endif

endmodule
