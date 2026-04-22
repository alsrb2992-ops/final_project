`timescale 1ns / 1ps

module uart_rx #( 
    parameter BPS = 9600
) (
    input clk,
    input reset_n,
    input rx,
    output reg [7:0] data_out,
    output reg rx_done
);

    localparam 
        IDLE        = 2'b00,
        START_BIT   = 2'b01,
        DATA_BITS   = 2'b10,
        STOP_BIT    = 2'b11;
    
    localparam DIVIDER_COUNT = 125_000_000 / (BPS * 16);

    reg [1:0] r_state;         
    reg [3:0] r_bit_cnt;       
    reg [7:0] r_data_reg;      
    reg [15:0] r_baud_cnt;     
    reg        r_baud_tick;    
    reg [4:0]  r_baud_tick_cnt;

   always @(posedge clk, negedge reset_n) begin
        if (!reset_n) begin
            r_baud_cnt <= 0;
            r_baud_tick <= 0;
        end else begin
            if (r_baud_cnt == DIVIDER_COUNT-1) begin
                r_baud_cnt <= 0;
                r_baud_tick <= 1; 
            end
            else begin
                r_baud_cnt <= r_baud_cnt + 1;
                r_baud_tick <= 0;
            end 
        end 
    end 

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            r_state <= IDLE;
            data_out <= 8'b0;
            rx_done <= 1'b0;
            r_bit_cnt <= 4'd0;
            r_data_reg <= 8'b0;
            r_baud_tick_cnt <= 5'd0;
        end
        else begin
            rx_done <= 1'b0;
            case (r_state)
                IDLE: begin
                    r_baud_tick_cnt <= 0;
                    if (!rx) begin
                        r_state <= START_BIT;
                        r_baud_tick_cnt <= 5'd0;
                    end
                end
                
                START_BIT: begin
                    if (r_baud_tick) begin
                        r_baud_tick_cnt <= r_baud_tick_cnt + 1;
                        if (r_baud_tick_cnt == 5'd7) begin
                            if(rx == 0) begin
                                r_state <= DATA_BITS;
                                r_bit_cnt <= 4'd0;
                                r_baud_tick_cnt <= 4'd0;
                            end
                            else
                                r_state <= IDLE;
                        end
                    end
                end

                DATA_BITS: begin
                    if (r_baud_tick) begin
                        r_baud_tick_cnt <= r_baud_tick_cnt + 1;
                        if (r_baud_tick_cnt == 5'd15) begin
                            r_data_reg[r_bit_cnt] <= rx;
                            r_baud_tick_cnt <= 5'd0;
                            if (r_bit_cnt == 4'd7) begin
                                r_state <= STOP_BIT;
                            end else begin
                                r_bit_cnt <= r_bit_cnt + 1;
                            end
                        end
                    end
                end
                
                STOP_BIT: begin
                    if (r_baud_tick) begin
                        r_baud_tick_cnt <= r_baud_tick_cnt + 1;
                        if (r_baud_tick_cnt == 5'd16) begin
                            if(rx == 1) begin
                                r_state <= IDLE;
                                data_out <= r_data_reg;
                                rx_done <= 1'b1;
                            end
                            else
                                r_state <= IDLE;
                        end
                    end
                end
                default: r_state <= IDLE;
            endcase
        end
    end
endmodule