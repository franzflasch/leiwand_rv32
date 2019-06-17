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
    reg [(`MEM_WIDTH-1):0] next_instruction;

    reg bus_read_write;
    reg bus_ready;
    reg bus_access;
    reg [(`MEM_WIDTH-1):0] bus_data_out;
    reg [(`MEM_WIDTH-1):0] bus_data_in;
    reg [(`MEM_WIDTH-1):0] bus_addr;


    /* testing purposes */
    reg [`HIGH_BIT_TO_FIT(127):0] i;


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

            cpu_stage <= STAGE_INSTR_FETCH;

            /* Just for testing */
            //i <= 0;
        end
        else begin

            // /* Just for testing */
            // if(bus_ready && !bus_access) begin
            //     bus_addr <= `MEM_WIDTH'h20000000 + i;
            //     bus_data_out <= `MEM_WIDTH'h00000042 + i;
            //     bus_access <= 1;

            //     i <= i + 1;
            //     if(i>=127) begin
            //         i <= 0;
            //         bus_addr <= 0;
            //         bus_data_out <= 0;
            //         bus_access <= 0;
            //         bus_read_write <= !bus_read_write;
            //     end
            // end

            case (cpu_stage)

                STAGE_INSTR_FETCH: begin
                    if(bus_ready && !bus_access) begin
                        bus_addr <= pc;
                        bus_data_out <= 0;
                        bus_access <= 1;
                        bus_read_write <= 0;
                        cpu_stage <= STAGE_INSTR_DECODE;
                    end
                end

                STAGE_INSTR_DECODE: begin
                    if(bus_ready && !bus_access) begin
                        next_instruction <= bus_data_in;
                        cpu_stage <= STAGE_INSTR_EXECUTE;
                    end
                end

                STAGE_INSTR_EXECUTE: begin
                    if(bus_ready && !bus_access) begin
                        cpu_stage <= STAGE_INSTR_ACCESS;
                    end
                end

                STAGE_INSTR_ACCESS: begin
                    if(bus_ready && !bus_access) begin
                        cpu_stage <= STAGE_INSTR_WRITEBACK;
                    end
                end

                STAGE_INSTR_WRITEBACK: begin
                    if(bus_ready && !bus_access) begin
                        pc <= pc + 4;
                        cpu_stage <= STAGE_INSTR_FETCH;
                    end
                end

            endcase


            /* reset x0 to zero, as theoretically in this implementation it can be set to any value */
            x[0]  <= 0; 
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
