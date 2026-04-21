`timescale 1ns / 1ps

module uart_tx_my (
    input            clk,
    input            reset,
    input      [7:0] tx_data,
    input            tx_start,
    output reg       tx,
    output reg       tx_busy,
    output reg       tx_done
);

    parameter S_IDLE = 2'b00;
    parameter S_START = 2'b01;
    parameter S_DATA = 2'b10;
    parameter S_STOP = 2'b11;

    parameter BPS = 128000;
    parameter DIVIDER_CNT = 100_000_000 / BPS;


    reg [1:0] r_state;
    reg [3:0] r_bit_cnt;
    reg [7:0] r_data;
    reg [15:0] r_baud_cnt;
    reg r_baud_tick;

    always @(posedge clk, negedge reset) begin
        if (!reset) begin
            r_baud_cnt  <= 0;
            r_baud_tick <= 0;
        end else begin
            if (r_baud_cnt == DIVIDER_CNT - 1) begin
                r_baud_cnt  <= 0;
                r_baud_tick <= 1;
            end else begin
                r_baud_cnt  <= r_baud_cnt + 1;
                r_baud_tick <= 0;
            end
        end
    end

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            r_state <= S_IDLE;
            r_bit_cnt <= 0;
            r_data <= 0;
            tx_done <= 0;
            tx_busy <= 0;
            tx <= 1;  // idle HIGH 
        end else begin
            case (r_state)
                S_IDLE: begin
                    tx_done <= 0;
                    if (tx_start) begin
                        r_data <= tx_data;
                        r_state <= S_START;
                        tx_busy <= 1'b1;
                        r_bit_cnt <= 0;
                    end
                end

                S_START: begin
                    if (r_baud_tick) begin
                        tx <= 1'b0;  // start bit 
                        r_state <= S_DATA;
                    end
                end

                S_DATA: begin
                    if (r_baud_tick) begin
                        tx <= r_data[r_bit_cnt];
                        if (r_bit_cnt == 4'd7) begin
                            r_state <= S_STOP;
                        end else begin
                            r_bit_cnt <= r_bit_cnt + 1;
                        end
                    end
                end

                S_STOP: begin
                    if (r_baud_tick) begin
                        tx_done <= 1;
                        tx <= 1'b1;  // STOP bit 
                        tx_busy <= 0;  // 현재 tx 작업중이 아니다.
                        r_state <= S_IDLE;
                    end
                end

                default: r_state <= S_IDLE;
            endcase
        end

    end

endmodule
