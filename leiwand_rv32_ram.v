`timescale 1ns/1ps 

`include "helper.v"

//`define INIT_RAM_TOZERO

module leiwand_rv32_ram # (
        parameter MEM_WIDTH = 8,
        parameter MEM_SIZE = 32,
        parameter MEM_HIGH_BIT = `HIGH_BIT_TO_FIT(MEM_SIZE-1)
    )
    (
        input i_clk,
        input i_rst,
        input [(MEM_WIDTH-1):0] i_addr,
        input [(MEM_WIDTH-1):0] i_dat,
        output [(MEM_WIDTH-1):0] o_dat,
        input i_we,
        input i_stb,
        output o_ack,
        input i_cyc,
        output o_stall,
        input [`HIGH_BIT_TO_FIT(4):0] i_dat_wr_size
    );

    parameter STATE_INIT = 0;
    parameter STATE_IDLE = 1;
    parameter STATE_FINISH = 2;
    
    reg [(MEM_WIDTH-1):0] data_out;
    reg ack_out;
    reg stall_out;
    reg tmp_stall;

    reg [(MEM_WIDTH-1):0] mem[0:MEM_SIZE];

    reg [`HIGH_BIT_TO_FIT(STATE_FINISH):0] internal_state;
    wire [MEM_HIGH_BIT:0] addr_index;

`ifdef INIT_RAM_TOZERO
    reg [`HIGH_BIT_TO_FIT(MEM_SIZE-1):0] mem_index;
`endif

    always @(posedge i_clk) begin
        if(i_rst) begin
            ack_out <= 0;
            stall_out <= 1;
            tmp_stall <= 1;
            data_out <= 0;

            internal_state <= STATE_INIT;

`ifdef INIT_RAM_TOZERO
            mem_index <= 0;
`endif
        end
        else begin
            case (internal_state)
                STATE_INIT: begin
                    ack_out <= 0;
                    stall_out <= 1;
                    tmp_stall <= 1;
                    data_out <= 0;
`ifdef INIT_RAM_TOZERO
                    mem[mem_index] = 0;
                    mem_index = mem_index + 1;
                    if(mem_index >= (MEM_SIZE-2)) begin
                        /* switch to next state after initialization */
                        internal_state <= STATE_IDLE;
                    end
`else
                    internal_state <= STATE_IDLE;
`endif

                end
                STATE_IDLE: begin
                    /* We will stall as soon as we are mentioned */
                    stall_out <= 0;
                    ack_out <= 0;
                    data_out <= 0;

                    /* Check for bus request condition */
                    if(i_cyc && i_stb) begin
                        stall_out <= 1;
                        tmp_stall <= 0;
                        internal_state <= STATE_FINISH;
                    end
                end
                STATE_FINISH: begin
                    if(i_we) begin
                        case (i_dat_wr_size)
                            1: case (i_addr[1:0])
                                0: mem[addr_index][7:0] <= i_dat[7:0];
                                1: mem[addr_index][15:8] <= i_dat[7:0];
                                2: mem[addr_index][23:16] <= i_dat[7:0];
                                3: mem[addr_index][31:24] <= i_dat[7:0];
                                default: mem[addr_index] <= 0;
                            endcase
                            2: case (i_addr[1])
                                0: mem[addr_index][15:0] <= i_dat[15:0];
                                1: mem[addr_index][31:16] <= i_dat[15:0];
                                default: mem[addr_index] <= 0;
                            endcase
                            4: mem[addr_index][31:0] <= i_dat[31:0];
                            default: mem[addr_index][31:0] <= i_dat[31:0];
                        endcase
                    end
                    else begin
                        data_out <= mem[addr_index];
                    end

                    if(i_cyc && !i_stb) begin
                        stall_out <= 0;
                        ack_out <= 1;
                        tmp_stall <= 1;
                        internal_state <= STATE_IDLE;
                    end
                end
            endcase
        end
    end

    assign o_dat = data_out;
    assign o_ack = ack_out;
    /* the combination with | (tmp_stall & i_stb) saves one clock cycle */
    assign o_stall = stall_out | (tmp_stall & i_stb);
    assign addr_index[MEM_HIGH_BIT:0] = i_addr[MEM_HIGH_BIT+2:2];

endmodule
