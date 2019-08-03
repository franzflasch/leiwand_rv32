`timescale 1ns/1ps 

`include "helper.v"
`include "leiwand_rv32_constants.v"

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

    parameter PC_START_VAL = `MEM_WIDTH'h10000000;

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
    parameter OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI = 7'b0010011;
    parameter FUNC3_ADDI = 3'b000;
    parameter FUNC3_SLTI = 3'b010;
    parameter FUNC3_SLTIU = 3'b011;
    parameter FUNC3_XORI = 3'b100;
    parameter FUNC3_ORI = 3'b110;
    parameter FUNC3_ANDI = 3'b111;
    parameter FUNC3_SLLI = 3'b001;
    parameter FUNC3_SRLI = 3'b101;
    parameter FUNC3_SRAI = 3'b101;
    parameter FUNC7_SLLI = 7'b0000000;
    parameter FUNC7_SRLI = 7'b0000000;
    parameter FUNC7_SRAI = 7'b0100000;

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
            x[5]  <= 0;
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
                            cpu_stage <= STAGE_INSTR_DECODE;
                        end

                        /* Decode next instruction */
                        STAGE_INSTR_DECODE: begin

                            /* ADDI */
                            if (bus_data_in[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) begin
                                rs1[4:0] <= bus_data_in[19:15];
                                rs2_shamt[4:0] <= bus_data_in[24:20];
                                rd[4:0] <= bus_data_in[11:7];
                                immediate <= bus_data_in[31:20];
                            end

                            instruction <= bus_data_in;
                            cpu_stage <= STAGE_INSTR_EXECUTE;
                        end

                        STAGE_INSTR_EXECUTE: begin

                            /* ADDI */
                            if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && (instruction[14:12] == FUNC3_ADDI) ) begin
                                x[rd] <= $signed(immediate[11:0]) + $signed(x[rs1]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                            /* SLTI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && (instruction[14:12] == FUNC3_SLTI) ) begin
                                x[rd] <= ($signed(x[rs1]) < $signed(immediate[11:0]));
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                            /* SLTIU */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && (instruction[14:12] == FUNC3_SLTIU) ) begin
                                x[rd] <= (x[rs1] < immediate[11:0]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                            /* XORI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && (instruction[14:12] == FUNC3_XORI) ) begin
                                if ($signed(immediate[11:0]) == -1) begin
                                    x[rd] <= (x[rs1] ^ 1);
                                end
                                else begin
                                    x[rd] <= (x[rs1] ^ immediate[11:0]);
                                end
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                            /* ORI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && (instruction[14:12] == FUNC3_ORI) ) begin
                                x[rd] <= x[rs1] | $signed(immediate[11:0]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                            /* ANDI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && (instruction[14:12] == FUNC3_ANDI) ) begin
                                x[rd] <= x[rs1] & $signed(immediate[11:0]);
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                            /* SLLI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && (instruction[14:12] == FUNC3_SLLI) && (instruction[31:25] == FUNC7_SLLI) ) begin
                                x[rd] <= x[rs1] << rs2_shamt[4:0];
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                            /* SRLI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && (instruction[14:12] == FUNC3_SRLI) && (instruction[31:25] == FUNC7_SRLI) ) begin
                                x[rd] <= x[rs1] >> rs2_shamt[4:0];
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                            /* SRAI */
                            else if ( (instruction[6:0] == OP_ADDI_SLTI_SLTIU_XORI_ORI_ANDI_SLLI_SRLI_SRAI) && (instruction[14:12] == FUNC3_SRAI) && (instruction[31:25] == FUNC7_SRAI) ) begin
                                x[rd] <= $signed(x[rs1]) >> rs2_shamt[4:0];
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end
                            else begin 
                                /* Unknown instruction */
                                cpu_stage <= STAGE_INSTR_FETCH;
                            end

                        end

                        STAGE_INSTR_ACCESS: begin
                            cpu_stage <= STAGE_INSTR_WRITEBACK;
                        end

                        STAGE_INSTR_WRITEBACK: begin
                            pc <= pc + 4;
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
