`timescale 1ns/1ps 

`include "helper.v"
`include "leiwand_rv32_constants.v"

`define DEBUG

`ifdef DEBUG
  `define debug(debug_command) debug_command
`else
  `define debug(debug_command)
`endif

module leiwand_rv32_core
    (
        input i_clk,
        input i_rst,

        /* wb signals */
        input i_ack,
        input [(`MEM_WIDTH-1):0] i_data,
        input i_stall,
        output o_we,
        output o_stb,
        output o_cyc,
        output [(`MEM_WIDTH-1):0] o_addr,
        output [(`MEM_WIDTH-1):0] o_data
    );

    /* Currently 4 Stages (not pipelined) */
    parameter STAGE_INSTR_FETCH = 0;
    parameter STAGE_INSTR_DECODE = 1;
    parameter STAGE_INSTR_ALU_PREPARE = 2;
    parameter STAGE_INSTR_ALU_EXECUTE = 3;
    parameter STAGE_INSTR_ACCESS = 4;
    parameter STAGE_INSTR_WRITEBACK = 5;
    reg [`HIGH_BIT_TO_FIT(STAGE_INSTR_WRITEBACK):0] cpu_stage;

    parameter PC_START_VAL = `MEM_WIDTH'h20400000;

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

    /* bus access variables */
    reg bus_read_write;
    reg bus_ready;
    reg bus_access;
    reg [(`MEM_WIDTH-1):0] bus_data_out;
    reg [(`MEM_WIDTH-1):0] bus_data_in;
    reg [(`MEM_WIDTH-1):0] bus_addr;


    /* RV32I Base instructions */
    parameter OP_LUI = 7'b0110111;
    parameter OP_AUIPC = 7'b0010111;

    parameter OP_JAL = 7'b1101111;

    parameter OP_JALR = 7'b1100111;
    parameter FUNC3_JALR = 3'b000;

    parameter OP_BEQ_BNE_BLT_BGE_BLTU_BGEU = 7'b1100011;
    parameter FUNC3_BEQ = 3'b000;
    parameter FUNC3_BNE = 3'b001;
    parameter FUNC3_BLT = 3'b100;
    parameter FUNC3_BGE = 3'b101;
    parameter FUNC3_BLTU = 3'b110;
    parameter FUNC3_BGEU = 3'b111;

    parameter OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI = 7'b0010011;
    parameter FUNC3_ADDI = 3'b000;
    parameter FUNC3_SLTI = 3'b010;
    parameter FUNC3_SLTIU = 3'b011;
    parameter FUNC3_XORI = 3'b100;
    parameter FUNC3_ORI = 3'b110;
    parameter FUNC3_ANDI = 3'b111;
    parameter FUNC3_SLLI = 3'b001;
    parameter FUNC7_SLLI = 7'b0000000;
    parameter FUNC3_SRLI = 3'b101;
    parameter FUNC7_SRLI = 7'b0000000;
    parameter FUNC3_SRAI = 3'b101;
    parameter FUNC7_SRAI = 7'b0100000;

    parameter OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND = 7'b0110011;
    parameter FUNC3_ADD = 3'b000;
    parameter FUNC7_ADD = 7'b0000000;
    parameter FUNC3_SUB = 3'b000;
    parameter FUNC7_SUB = 7'b0100000;
    parameter FUNC3_SLL = 3'b001;
    parameter FUNC7_SLL = 7'b0000000;
    parameter FUNC3_SLT = 3'b010;
    parameter FUNC7_SLT = 7'b0000000;
    parameter FUNC3_SLTU = 3'b011;
    parameter FUNC7_SLTU = 7'b0000000;
    parameter FUNC3_XOR = 3'b100;
    parameter FUNC7_XOR = 7'b0000000;
    parameter FUNC3_SRL = 3'b101;
    parameter FUNC7_SRL = 7'b0000000;
    parameter FUNC3_SRA = 3'b101;
    parameter FUNC7_SRA = 7'b0100000;
    parameter FUNC3_OR = 3'b110;
    parameter FUNC7_OR = 7'b0000000;
    parameter FUNC3_AND = 3'b111;
    parameter FUNC7_AND = 7'b0000000;

    parameter OP_FENCE_FENCEI = 7'b0001111;
    parameter FUNC3_FENCE =  3'b000;
    parameter FUNC3_FENCEI = 3'b001;

    parameter OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI = 7'b1110011;
    parameter FUNC3_ECALL = 3'b000;
    parameter IMM11_ECALL = 12'b000000000000;
    parameter FUNC3_EBREAK = 3'b000;
    parameter IMM11_EBREAK = 12'b000000000001;
    parameter FUNC3_CSRRW = 3'b001;
    parameter FUNC3_CSRRS = 3'b010;
    parameter FUNC3_CSRRC = 3'b011;
    parameter FUNC3_CSRRWI = 3'b101;
    parameter FUNC3_CSRRSI = 3'b110;
    parameter FUNC3_CSRRCI = 3'b111;

    parameter OP_LB_LH_LW_LBU_LHU = 7'b0000011;
    parameter FUNC3_LB = 3'b000;
    parameter FUNC3_LH = 3'b001;
    parameter FUNC3_LW = 3'b010;
    parameter FUNC3_LBU = 3'b100;
    parameter FUNC3_LHU = 3'b101;

    reg is_LUI;
    reg is_AUIPC;
    reg is_JAL;
    reg is_JALR;
    reg is_BEQ, is_BNE, is_BLT, is_BGE, is_BLTU, is_BGEU;
    reg is_ADDI, is_SLTI, is_SLTIU, is_XORI, is_ORI, is_ANDI, is_SLLI, is_SRLI, is_SRAI;
    reg is_ADD, is_SUB, is_SLL, is_SLT, is_SLTU, is_XOR, is_SRL, is_SRA, is_OR, is_AND;
    reg is_LB, is_LH, is_LW, is_LBU, is_LHU;

    /* ALU */
    reg [(`MEM_WIDTH-1):0] alu_result_addu;
    reg [(`MEM_WIDTH-1):0] alu_result_add;
    reg [(`MEM_WIDTH-1):0] alu_result_sub;
    reg alu_result_eq;
    reg alu_result_ge;
    reg alu_result_geu;
    reg [(`MEM_WIDTH-1):0] alu_result_xor;
    reg [(`MEM_WIDTH-1):0] alu_result_or;
    reg [(`MEM_WIDTH-1):0] alu_result_and;
    reg [(`MEM_WIDTH-1):0] alu_result_sll;
    reg [(`MEM_WIDTH-1):0] alu_result_srl;
    reg [(`MEM_WIDTH-1):0] alu_result_sra;
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
            x[5]  <= 32'h20400000; // This is only because of comparison with qemu
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

            /* zero out opcode registers */
            rs1 <= 0;
            rs2_shamt <= 0;
            rd <= 0;
            immediate <= 0;

            /* First stage is instruction */
            cpu_stage <= STAGE_INSTR_FETCH;

            {is_LUI, 
             is_AUIPC, 
             is_JAL, is_JALR, 
             is_BEQ, is_BNE, is_BLT, is_BGE, is_BLTU, is_BGEU, 
             is_ADDI, is_SLTI, is_SLTIU, is_XORI, is_ORI, is_ANDI, is_SLLI, is_SRLI, is_SRAI, 
             is_ADD, is_SUB, is_SLL, is_SLT, is_SLTU, is_XOR, is_SRL, is_SRA, is_OR, is_AND,
             is_LB, is_LH, is_LW, is_LBU, is_LHU } <= 0;

            {alu_result_addu, 
             alu_result_add, 
             alu_result_sub, 
             alu_result_eq, 
             alu_result_ge, 
             alu_branch_geu,
             alu_result_xor, 
             alu_result_or, 
             alu_result_and,
             alu_result_sll, 
             alu_result_srl, 
             alu_result_sra, 
             alu_op1, alu_op2} <= 0;
            {alu_branch_eq, alu_branch_ge, alu_branch_geu, alu_branch_op1, alu_branch_op2} <= 0;
        end
        else begin

            if (bus_ready && !bus_access) begin

                case (cpu_stage)

                        STAGE_INSTR_FETCH: begin
                            bus_addr <= next_pc;
                            bus_data_out <= 0;
                            bus_access <= 1;
                            bus_read_write <= 0;

                            pc <= next_pc;
                            next_pc <= next_pc + 4;
                            cpu_stage <= STAGE_INSTR_DECODE;
                        end

                        /* Decode next instruction */
                        STAGE_INSTR_DECODE: begin

                            rs1[4:0] <= bus_data_in[19:15];
                            rs2_shamt[4:0] <= bus_data_in[24:20];
                            rd[4:0] <= bus_data_in[11:7];

                            is_LUI <= (bus_data_in[6:0] == OP_LUI) ? 1 : 0;
                            is_AUIPC <= (bus_data_in[6:0] == OP_AUIPC) ? 1 : 0;
                            is_JAL <= (bus_data_in[6:0] == OP_JAL) ? 1 : 0;
                            is_JALR <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_JALR, OP_JALR} ) ? 1 : 0;

                            is_BEQ <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_BEQ, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BNE <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_BNE, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BLT <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_BLT, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BGE <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_BGE, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BLTU <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_BLTU, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;
                            is_BGEU <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_BGEU, OP_BEQ_BNE_BLT_BGE_BLTU_BGEU} ) ? 1 : 0;

                            is_ADDI <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_ADDI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SLTI <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_SLTI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SLTIU <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_SLTIU, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_XORI <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_XORI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_ORI <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_ORI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_ANDI <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_ANDI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SLLI <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_SLLI, FUNC3_SLLI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SRLI <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_SRLI, FUNC3_SRLI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;
                            is_SRAI <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_SRAI, FUNC3_SRAI, OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI} ) ? 1 : 0;

                            is_ADD <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_ADD, FUNC3_ADD, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SUB <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_SUB, FUNC3_SUB, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SLL <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_SLL, FUNC3_SLL, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SLT <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_SLT, FUNC3_SLT, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SLTU <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_SLTU, FUNC3_SLTU, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_XOR <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_XOR, FUNC3_XOR, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SRL <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_SRL, FUNC3_SRL, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_SRA <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_SRA, FUNC3_SRA, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_OR <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_OR, FUNC3_OR, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;
                            is_AND <= ({bus_data_in[31:25],bus_data_in[14:12],bus_data_in[6:0]} == {FUNC7_AND, FUNC3_AND, OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND} ) ? 1 : 0;

                            is_LB <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_LB, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;
                            is_LH <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_LH, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;
                            is_LW <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_LW, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;
                            is_LBU <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_LBU, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;
                            is_LHU <= ({bus_data_in[14:12],bus_data_in[6:0]} == {FUNC3_LHU, OP_LB_LH_LW_LBU_LHU} ) ? 1 : 0;


                            case (bus_data_in[6:0])

                                /* I-type */
                                OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI, 
                                OP_JALR,
                                OP_FENCE_FENCEI,
                                OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI,
                                OP_LB_LH_LW_LBU_LHU: begin
                                    immediate <= { {20{bus_data_in[31]}}, bus_data_in[31:20] };
                                end

                                /* B-type */
                                OP_BEQ_BNE_BLT_BGE_BLTU_BGEU: begin
                                    immediate <= { {20{bus_data_in[31]}}, bus_data_in[7], bus_data_in[30:25], bus_data_in[11:8], 1'b0 };
                                end

                                /* U-type */
                                OP_LUI,
                                OP_AUIPC: begin
                                    immediate <= { bus_data_in[31:12], 12'b000000000000};
                                end

                                /* J-type */
                                OP_JAL: begin
                                    immediate <= { {11{bus_data_in[31]}}, bus_data_in[19:12], bus_data_in[20], bus_data_in[31:21], 1'b0 };
                                end

                            endcase

                            instruction <= bus_data_in;
                            cpu_stage <= STAGE_INSTR_ALU_PREPARE;
                        end

                        STAGE_INSTR_ALU_PREPARE: begin

                            alu_branch_op1 <= x[rs1];
                            alu_branch_op2 <= x[rs2_shamt];

                            if(is_AUIPC | is_JAL |
                               is_BEQ | is_BNE | is_BLT | is_BGE | is_BLTU | is_BGEU) begin
                               alu_op1 <= pc;
                            end

                            if(is_LUI | is_AUIPC | is_JAL | 
                               is_BEQ | is_BNE | is_BLT | is_BGE | is_BLTU | is_BGEU | 
                               is_ADDI | is_SLTI | is_SLTIU | is_XORI | is_ORI | is_ANDI) begin
                               alu_op2 <= immediate;
                            end

                            if(is_JALR) begin
                                alu_op1 <= {x[rs1][31:1], 1'b0};
                                alu_op2 <= { immediate[31:1], 1'b0 };
                            end

                            if(is_ADDI | is_SLTI | is_SLTIU | is_XORI | is_ORI | is_ANDI | is_SLLI | is_SRLI | is_SRAI | 
                               is_ADD | is_SUB | is_SLT | is_SLTU | is_XOR | is_OR | is_AND | is_SLL | is_SRL | is_SRA) begin
                                alu_op1 <= x[rs1];
                            end

                            if(is_SLLI | is_SRLI | is_SRAI) begin
                                alu_op2 <= rs2_shamt;
                            end

                            if(is_ADD | is_SUB | is_SLT | is_SLTU | is_XOR | is_OR | is_AND) begin
                                alu_op2 <= x[rs2_shamt];
                            end

                            if(is_SLL | is_SRL | is_SRA) begin
                                alu_op2 <= x[rs2_shamt][4:0];
                            end

                            if(is_LB | is_LH | is_LW | is_LBU | is_LHU) begin
                                bus_addr <= ( $signed(x[rs1]) + $signed(immediate) );
                                bus_data_out <= 0;
                                bus_access <= 1;
                                bus_read_write <= 0;
                                cpu_stage <= STAGE_INSTR_ACCESS;
                            end
                            else cpu_stage <= STAGE_INSTR_ALU_EXECUTE;
                        end

                        STAGE_INSTR_ALU_EXECUTE: begin

                            alu_result_addu <= alu_op1 + alu_op2;
                            alu_result_add <= $signed(alu_op1) + $signed(alu_op2);
                            alu_result_sub <= alu_op1 - alu_op2;
                            alu_result_ge <= $signed(alu_op1) >= $signed(alu_op2);
                            alu_result_geu <= alu_op1 >= alu_op2;
                            alu_result_xor <= alu_op1 ^ alu_op2;
                            alu_result_or <= alu_op1 | alu_op2;
                            alu_result_and <= alu_op1 & alu_op2;
                            alu_result_sll <= alu_op1 << alu_op2;
                            alu_result_srl <= alu_op1 >> alu_op2;
                            alu_result_sra <= $signed(alu_op1) >>> alu_op2;

                            alu_branch_eq <= (alu_branch_op1 == alu_branch_op2);
                            alu_branch_ge <= ($signed(alu_branch_op1) >= $signed(alu_branch_op2));
                            alu_branch_geu <= (alu_branch_op1 >= alu_branch_op2);

                            cpu_stage <= STAGE_INSTR_WRITEBACK;
                        end

                        STAGE_INSTR_ACCESS: begin

                            if (is_LB) begin
                                case (bus_addr[1:0])
                                    0: x[rd] <= {{24{bus_data_in[7]}},bus_data_in[7:0]};
                                    1: x[rd] <= {{24{bus_data_in[15]}},bus_data_in[15:8]};
                                    2: x[rd] <= {{24{bus_data_in[23]}},bus_data_in[23:16]};
                                    3: x[rd] <= {{24{bus_data_in[31]}},bus_data_in[31:24]};
                                    default: x[rd] <= 0;
                                endcase
                            end
                            if (is_LH) begin
                                x[rd] <= (bus_addr[1]) ? {{16{bus_data_in[31]}},bus_data_in[31:16]} : {{16{bus_data_in[15]}},bus_data_in[15:0]};
                            end
                            if (is_LW) begin
                                x[rd] <= bus_data_in;
                            end
                            if (is_LBU) begin
                                case (bus_addr[1:0])
                                    0: x[rd] <= bus_data_in[7:0];
                                    1: x[rd] <= bus_data_in[15:8];
                                    2: x[rd] <= bus_data_in[23:16];
                                    3: x[rd] <= bus_data_in[31:24];
                                    default: x[rd] <= 0;
                                endcase
                            end
                            if (is_LHU) begin
                                x[rd] <= (bus_addr[1]) ? bus_data_in[31:16] : bus_data_in[15:0];
                            end

                            cpu_stage <= STAGE_INSTR_WRITEBACK;
                        end

                        STAGE_INSTR_WRITEBACK: begin

                            if (is_LUI) x[rd] <= immediate;
                            if (is_AUIPC) x[rd] <= alu_result_addu;
                            if (is_JAL | is_JALR) begin x[rd] <= next_pc; next_pc <= alu_result_addu; end
                            if ( (is_BEQ && alu_branch_eq) || 
                                 (is_BNE && !alu_branch_eq) || 
                                 (is_BLT && !alu_branch_ge) ||
                                 (is_BGE && alu_branch_ge) || 
                                 (is_BLTU && !alu_branch_geu) ||
                                 (is_BGEU && alu_branch_geu) ) begin
                                next_pc <= alu_result_addu; 
                            end
                            if(is_ADDI) begin x[rd] <= alu_result_add; end
                            if(is_SLTI) begin x[rd] <= !alu_result_ge; end
                            if(is_SLTIU) begin x[rd] <= !alu_result_geu; end
                            if(is_XORI) begin x[rd] <= alu_result_xor; end
                            if(is_ORI) begin x[rd] <= alu_result_or; end
                            if(is_ANDI) begin x[rd] <= alu_result_and; end
                            if(is_SLLI) begin x[rd] <= alu_result_sll; end
                            if(is_SRLI) begin x[rd] <= alu_result_srl; end
                            if(is_SRAI) begin x[rd] <= alu_result_sra; end
                            if(is_ADD) begin x[rd] <= alu_result_add; end
                            if(is_SUB) begin x[rd] <= alu_result_sub; end
                            if(is_SLL) begin x[rd] <= alu_result_sll; end
                            if(is_SLT) begin x[rd] <= !alu_result_ge; end
                            if(is_SLTU) begin x[rd] <= !alu_result_geu; end
                            if(is_XOR) begin x[rd] <= alu_result_xor; end
                            if(is_SRL) begin x[rd] <= alu_result_srl; end
                            if(is_SRA) begin x[rd] <= alu_result_sra; end
                            if(is_OR) begin x[rd] <= alu_result_or; end
                            if(is_AND) begin x[rd] <= alu_result_and; end

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


    /* WB Bus Handling */
    /* WB master signals */
    reg we_out_reg;
    reg stb_out_reg;
    reg cyc_out_reg;
    reg [(`MEM_WIDTH-1):0] address_out_reg;
    reg [(`MEM_WIDTH-1):0] data_out_reg;

    always @(posedge i_clk) begin
        if(i_rst) begin
            bus_access <= 0;
            bus_ready <= 0;
            bus_data_out <= 0;
            bus_data_in <= 0;
            bus_read_write <= 0;
            bus_addr <= 0;

            /* initialize wb master signals */
            we_out_reg <= 0;
            cyc_out_reg <= 0;
            address_out_reg <= 0; //`MEM_WIDTH'h20000004;
            data_out_reg <= 0;
            stb_out_reg <= 0;
        end
        else begin
            /* initialization */
            if(!i_stall && !cyc_out_reg && !bus_access) begin
                bus_ready <= 1;
            end

            /* Begin access */
            if(bus_ready && bus_access) begin
                address_out_reg <= bus_addr;
                we_out_reg <= bus_read_write;
                stb_out_reg <= 1;

                if(bus_read_write) begin
                    data_out_reg <= bus_data_out;
                end

                cyc_out_reg <= 1;
                bus_ready <= 0;
            end

            /* Wait for slave ack */
            if(bus_access && !bus_ready) begin
                stb_out_reg <= 0;
            end

            if(i_ack && !i_stall) begin
                bus_data_in <= i_data;
                address_out_reg <= 0;
                we_out_reg <= 0;
                data_out_reg <= 0;
                cyc_out_reg <= 0;
                bus_access <= 0;
            end
        end
    end

    assign o_we = we_out_reg;
    assign o_stb = stb_out_reg;
    assign o_cyc = cyc_out_reg;
    assign o_addr = address_out_reg;
    assign o_data = data_out_reg;

endmodule
