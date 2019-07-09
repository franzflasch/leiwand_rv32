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

    /* Classic 5 RISC Stages (currently not pipelined) */
    parameter STAGE_INSTR_FETCH = 0;
    parameter STAGE_INSTR_DECODE = 1;
    parameter STAGE_INSTR_EXECUTE = 2;
    parameter STAGE_INSTR_ACCESS = 3;
    parameter STAGE_INSTR_WRITEBACK = 4;
    reg [`HIGH_BIT_TO_FIT(STAGE_INSTR_WRITEBACK):0] cpu_stage;

    parameter PC_START_VAL = `MEM_WIDTH'h20400000;

    /* RISC-V Registers x0-x31 */
    reg [(`MEM_WIDTH-1):0] x[(`NR_RV_REGS-1):0];
    reg [(`MEM_WIDTH-1):0] pc;
    reg [(`MEM_WIDTH-1):0] instruction;

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

    /* opcode registers 1 */
    reg [4:0] rs1;
    reg [4:0] rs2_shamt;
    reg [4:0] rd;
    reg [(`MEM_WIDTH-1):0] immediate;

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
            instruction <= 0;

            /* zero out opcode registers */
            rs1 <= 0;
            rs2_shamt <= 0;
            rd <= 0;
            immediate <= 0;

            /* First stage is instruction */
            cpu_stage <= STAGE_INSTR_FETCH;

        end
        else begin

            if (bus_ready && !bus_access) begin

                case (cpu_stage)

                        STAGE_INSTR_FETCH: begin
                            bus_addr <= pc;
                            bus_data_out <= 0;
                            bus_access <= 1;
                            bus_read_write <= 0;

                            // instruction <= 0;
                            // rs1 <= 0;
                            // rs2_shamt <= 0;
                            // rd <= 0;
                            // immediate <= 0;

                            cpu_stage <= STAGE_INSTR_DECODE;
                        end

                        /* Decode next instruction */
                        STAGE_INSTR_DECODE: begin

                            pc <= pc + 4;

                            /* U-Type instruction */
                            if ( (bus_data_in[6:0] == OP_LUI) || (bus_data_in[6:0] == OP_AUIPC) ) begin
                                rd[4:0] <= bus_data_in[11:7];
                                immediate <= bus_data_in[31:12];
                            end

                            /* R-Type instruction */
                            else if ( (bus_data_in[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) ) begin
                                rs1[4:0] <= bus_data_in[19:15];
                                rs2_shamt[4:0] <= bus_data_in[24:20];
                                rd[4:0] <= bus_data_in[11:7];
                            end

                            /* I-Type instruction */
                            else if ( (bus_data_in[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) || 
                                      (bus_data_in[6:0] == OP_JALR) || 
                                      (bus_data_in[6:0] == OP_FENCE_FENCEI) ||
                                      (bus_data_in[6:0] == OP_FENCE_FENCEI) ||
                                      (bus_data_in[6:0] == OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI) ) begin
                                rs1[4:0] <= bus_data_in[19:15];
                                rs2_shamt[4:0] <= bus_data_in[24:20];
                                rd[4:0] <= bus_data_in[11:7];
                                immediate <= bus_data_in[31:20];
                            end

                            /* B-Type instruction */
                            else if (bus_data_in[6:0] == OP_BEQ_BNE_BLT_BGE_BLTU_BGEU) begin
                                rs1[4:0] <= bus_data_in[19:15];
                                rs2_shamt[4:0] <= bus_data_in[24:20];
                                immediate <= ( (bus_data_in[31] << 12) | 
                                               (bus_data_in[7] << 11) | 
                                               (bus_data_in[30:25] << 5) | 
                                               (bus_data_in[11:8] << 1) );
                            end

                            /* J-Type instruction */
                            else if (bus_data_in[6:0] == OP_JAL) begin
                                rd[4:0] <= bus_data_in[11:7];
                                immediate <= ( (bus_data_in[31] << 20) | 
                                               (bus_data_in[19:12] << 12) | 
                                               (bus_data_in[20] << 11) | 
                                               (bus_data_in[30:21] << 1) );
                            end

                            instruction <= bus_data_in;
                            cpu_stage <= STAGE_INSTR_EXECUTE;
                        end

                        STAGE_INSTR_EXECUTE: begin
                            /* LUI */
                            if ( (instruction[6:0] == OP_LUI) ) begin
                                x[rd] <= (immediate << 12);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR LUI");)
                            end
                            /* AUIPC */
                            else if ( (instruction[6:0] == OP_AUIPC) ) begin
                                x[rd] <= (pc - 4) + (immediate << 12);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR AUIPC");)
                            end
                            /* JAL */
                            else if (instruction[6:0] == OP_JAL) begin
                                x[rd] <= pc;
                                if(immediate[20]) pc <= ( (pc - 4) + $signed(immediate[31:0] | 32'hFFF00000) );
                                else pc <= (pc - 4) + $signed(immediate[31:0]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR JAL");)
                            end
                            /* JALR */
                            else if ( (instruction[6:0] == OP_JALR) && (instruction[14:12] == FUNC3_JALR) ) begin
                                x[rd] <= pc;
                                if(immediate[11]) pc <= ( ( x[rs1] + $signed(immediate[31:0] | 32'hFFFFF000) ) & 32'hFFFFFFFE);
                                else pc <= ( ( x[rs1] + $signed(immediate[31:0]) ) & 32'hFFFFFFFE);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR JALR");)
                            end
                            /* BEQ */
                            else if ( (instruction[6:0] == OP_BEQ_BNE_BLT_BGE_BLTU_BGEU) && (instruction[14:12] == FUNC3_BEQ) ) begin
                                if($signed(x[rs1]) == $signed(x[rs2_shamt])) begin
                                    if(immediate[12]) pc <= ( (pc - 4) + $signed(immediate[31:0] | 32'hFFFFF000) );
                                    else pc <= (pc - 4) + $signed(immediate[31:0]);
                                end
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR BEQ");)
                            end
                            /* BNE */
                            else if ( (instruction[6:0] == OP_BEQ_BNE_BLT_BGE_BLTU_BGEU) && (instruction[14:12] == FUNC3_BNE) ) begin
                                if($signed(x[rs1]) != $signed(x[rs2_shamt])) begin
                                    if(immediate[12]) pc <= ( (pc - 4) + $signed(immediate[31:0] | 32'hFFFFF000) );
                                    else pc <= (pc - 4) + $signed(immediate[31:0]);
                                end
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR BNE");)
                            end
                            /* BLT */
                            else if ( (instruction[6:0] == OP_BEQ_BNE_BLT_BGE_BLTU_BGEU) && (instruction[14:12] == FUNC3_BLT) ) begin
                                if($signed(x[rs1]) < $signed(x[rs2_shamt])) begin
                                    if(immediate[12]) pc <= ( (pc - 4) + $signed(immediate[31:0] | 32'hFFFFF000) );
                                    else pc <= (pc - 4) + $signed(immediate[31:0]);
                                end
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR BLT");)
                            end
                            /* BGE */
                            else if ( (instruction[6:0] == OP_BEQ_BNE_BLT_BGE_BLTU_BGEU) && (instruction[14:12] == FUNC3_BGE) ) begin
                                if($signed(x[rs1]) >= $signed(x[rs2_shamt])) begin
                                    if(immediate[12]) pc <= ( (pc - 4) + $signed(immediate[31:0] | 32'hFFFFF000) );
                                    else pc <= (pc - 4) + $signed(immediate[31:0]);
                                end
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR BGE");)
                            end
                            /* BLTU */
                            else if ( (instruction[6:0] == OP_BEQ_BNE_BLT_BGE_BLTU_BGEU) && (instruction[14:12] == FUNC3_BLTU) ) begin
                                if(x[rs1] < x[rs2_shamt]) begin
                                    if(immediate[12]) pc <= ( (pc - 4) + $signed(immediate[31:0] | 32'hFFFFF000) );
                                    else pc <= (pc - 4) + $signed(immediate[31:0]);
                                end
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR BLTU");)
                            end
                            /* BGEU */
                            else if ( (instruction[6:0] == OP_BEQ_BNE_BLT_BGE_BLTU_BGEU) && (instruction[14:12] == FUNC3_BGEU) ) begin
                                if(x[rs1] >= x[rs2_shamt]) begin
                                    if(immediate[12]) pc <= ( (pc - 4) + $signed(immediate[31:0] | 32'hFFFFF000) );
                                    else pc <= (pc - 4) + $signed(immediate[31:0]);
                                end
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR BGEU");)
                            end
                            /* LB */
                            /* LH */
                            /* LW */
                            /* LBU */
                            /* LHU */
                            /* SB */
                            /* SH */
                            /* SW */
                            /* ADDI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && 
                                      (instruction[14:12] == FUNC3_ADDI) ) begin
                                x[rd] <= ($signed(x[rs1]) + $signed(immediate[11:0]));
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR ADDI");)
                            end
                            /* SLTI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && 
                                      (instruction[14:12] == FUNC3_SLTI) ) begin
                                x[rd] <= ($signed(x[rs1]) < $signed(immediate[11:0]));
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SLTI");)
                            end
                            /* SLTIU */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && 
                                      (instruction[14:12] == FUNC3_SLTIU) ) begin
                                if(immediate[11]) x[rd] <= x[rs1] < ( immediate[31:0] | 32'hfffff000 );
                                else x[rd] <= x[rs1] < immediate[31:0];
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SLTIU");)
                            end
                            /* XORI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && 
                                      (instruction[14:12] == FUNC3_XORI) ) begin
                                if(immediate[11]) x[rd] <= x[rs1] ^ ( immediate[31:0] | 32'hfffff000 );
                                else x[rd] <= (x[rs1] ^ immediate[31:0]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR XORI");)
                            end
                            /* ORI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && 
                                      (instruction[14:12] == FUNC3_ORI) ) begin
                                if(immediate[11]) x[rd] <= x[rs1] | ( immediate[31:0] | 32'hfffff000 );
                                else x[rd] <= (x[rs1] | immediate[31:0]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR ORI");)
                            end
                            /* ANDI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && 
                                      (instruction[14:12] == FUNC3_ANDI) ) begin
                                if(immediate[11]) x[rd] <= x[rs1] & ( immediate[31:0] | 32'hfffff000 );
                                else x[rd] <= (x[rs1] & immediate[31:0]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR ANDI");)
                            end
                            /* SLLI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && 
                                      (instruction[14:12] == FUNC3_SLLI) && 
                                      (instruction[31:25] == FUNC7_SLLI) ) begin
                                x[rd] <= x[rs1] << rs2_shamt[4:0];
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SLLI");)
                            end
                            /* SRLI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && 
                                      (instruction[14:12] == FUNC3_SRLI) && 
                                      (instruction[31:25] == FUNC7_SRLI) ) begin
                                x[rd] <= x[rs1] >> rs2_shamt[4:0];
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SRLI");)
                            end
                            /* SRAI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && 
                                      (instruction[14:12] == FUNC3_SRAI) && 
                                      (instruction[31:25] == FUNC7_SRAI) ) begin
                                /* Arithmetic shift is >>> */
                                x[rd] <= ($signed(x[rs1]) >>> rs2_shamt[4:0]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SRAI");)
                            end
                            /* ADD */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_ADD) && 
                                      (instruction[31:25] == FUNC7_ADD) ) begin
                                x[rd] <= (x[rs1] + x[rs2_shamt]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR ADD");)
                            end
                            /* SUB */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_SUB) && 
                                      (instruction[31:25] == FUNC7_SUB) ) begin
                                x[rd] <= (x[rs1] - x[rs2_shamt]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SUB");)
                            end
                            /* SLL */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_SLL) && (instruction[31:25] == FUNC7_SLL) ) begin
                                x[rd] <= (x[rs1] << (x[rs2_shamt] & 'h1F));
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SLL");)
                            end
                            /* SLT */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_SLT) && 
                                      (instruction[31:25] == FUNC7_SLT) ) begin
                                x[rd] <= ($signed(x[rs1]) < $signed(x[rs2_shamt]));
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SLT");)
                            end
                            /* SLTU */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_SLTU) && 
                                      (instruction[31:25] == FUNC7_SLTU) ) begin
                                x[rd] <= (x[rs1] < x[rs2_shamt]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SLTU");)
                            end
                            /* XOR */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_XOR) && 
                                      (instruction[31:25] == FUNC7_XOR) ) begin
                                x[rd] <= (x[rs1] ^ x[rs2_shamt]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR XOR");)
                            end                            
                            /* SRL */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_SRL) && 
                                      (instruction[31:25] == FUNC7_SRL) ) begin
                                x[rd] <= (x[rs1] >> (x[rs2_shamt] & 'h1F));
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SRL");)
                            end
                            /* SRA */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_SRA) && 
                                      (instruction[31:25] == FUNC7_SRA) ) begin
                                x[rd] <= ($signed(x[rs1]) >>> (x[rs2_shamt] & 'h1F));
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR SRA");)
                            end
                            /* OR */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_OR) && 
                                      (instruction[31:25] == FUNC7_OR) ) begin
                                x[rd] <= (x[rs1] | x[rs2_shamt]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR OR");)
                            end
                            /* AND */
                            else if ( (instruction[6:0] == OP_ADD_SUB_SLL_SLT_SLTU_XOR_SRL_SRA_OR_AND) && 
                                      (instruction[14:12] == FUNC3_AND) && 
                                      (instruction[31:25] == FUNC7_AND) ) begin
                                x[rd] <= (x[rs1] & x[rs2_shamt]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR AND");)
                            end
                            /* FENCE */
                            else if ( (instruction[6:0] == OP_FENCE_FENCEI) && (instruction[14:12] == FUNC3_FENCE) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR FENCE");)
                            end
                            /* FENCE.I */
                            else if ( (instruction[6:0] == OP_FENCE_FENCEI) && (instruction[14:12] == FUNC3_FENCEI) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR FENCEI");)
                            end
                            /* ECALL */
                            else if ( (instruction[6:0] == OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI) && 
                                      (instruction[14:12] == FUNC3_ECALL) && 
                                      (instruction[31:20] == IMM11_ECALL) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR ECALL");)
                            end
                            /* EBREAK */
                            else if ( (instruction[6:0] == OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI) && 
                                      (instruction[14:12] == FUNC3_EBREAK) && 
                                      (instruction[31:20] == IMM11_EBREAK) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR EBREAK");)
                            end
                            /* CSRRW */
                            else if ( (instruction[6:0] == OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI) && 
                                      (instruction[14:12] == FUNC3_CSRRW) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR CSRRW");)
                            end
                            /* CSRRS */
                            else if ( (instruction[6:0] == OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI) && 
                                      (instruction[14:12] == FUNC3_CSRRS) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR CSRRS");)
                            end
                            /* CSRRC */
                            else if ( (instruction[6:0] == OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI) && 
                                      (instruction[14:12] == FUNC3_CSRRC) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR CSRRC");)
                            end
                            /* CSRRWI */
                            else if ( (instruction[6:0] == OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI) && 
                                      (instruction[14:12] == FUNC3_CSRRWI) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR CSRRWI");)
                            end
                            /* CSRRSI */
                            else if ( (instruction[6:0] == OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI) && 
                                      (instruction[14:12] == FUNC3_CSRRSI) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR CSRRSI");)
                            end
                            /* CSRRCI */
                            else if ( (instruction[6:0] == OP_ECALL_EBREAK_CSRRW_CSRRS_CSRRC_CSRRWI_CSRRSI_CSRRCI) && 
                                      (instruction[14:12] == FUNC3_CSRRCI) ) begin
                                /* NOP */
                                cpu_stage <= STAGE_INSTR_FETCH;
                                `debug($display("INSTR CSRRCI");)
                            end
                            else begin 
                                `debug($display("Unknown instruction! %x", instruction);)
                                /* Unknown instruction */
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                        end

                        STAGE_INSTR_ACCESS: begin
                            cpu_stage <= STAGE_INSTR_WRITEBACK;
                        end

                        STAGE_INSTR_WRITEBACK: begin
                            cpu_stage <= STAGE_INSTR_FETCH;
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



    //assign x[0][31:0] = 32'h00000000;
    assign o_we = we_out_reg;
    assign o_stb = stb_out_reg;
    assign o_cyc = cyc_out_reg;
    assign o_addr = address_out_reg;
    assign o_data = data_out_reg;


endmodule
