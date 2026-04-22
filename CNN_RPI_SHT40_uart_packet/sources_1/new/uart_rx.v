`timescale 1ns / 1ps
//=============================================================================
// Module  : uart_rx
// Purpose : Deserialize one byte from UART (8N1)
//           Samples at mid-bit using a 16x oversampled tick
// Signals : rx_data  - received byte (valid when rx_done pulses)
//           rx_done  - 1-cycle pulse when full byte received
//           rx_error - framing error (stop bit not '1')
//=============================================================================
module uart_rx #(
    parameter integer CLK_FREQ  = 100_000_000,
    parameter integer BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,
    output reg  [7:0] rx_data,
    output reg        rx_done,
    output reg        rx_error
);

    localparam integer OVS_DIV = CLK_FREQ / (BAUD_RATE * 16);

    reg [$clog2(OVS_DIV)-1:0] ovs_cnt;
    reg                        ovs_tick;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ovs_cnt  <= 0;
            ovs_tick <= 1'b0;
        end else if (ovs_cnt == OVS_DIV - 1) begin
            ovs_cnt  <= 0;
            ovs_tick <= 1'b1;
        end else begin
            ovs_cnt  <= ovs_cnt + 1'b1;
            ovs_tick <= 1'b0;
        end
    end

    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_START = 2'd1;
    localparam [1:0] ST_DATA  = 2'd2;
    localparam [1:0] ST_STOP  = 2'd3;

    reg [1:0] state;
    reg [3:0] sample_cnt;
    reg [2:0] bit_idx;
    reg [7:0] shift_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state      <= ST_IDLE;
            sample_cnt <= 4'd0;
            bit_idx    <= 3'd0;
            shift_reg  <= 8'd0;
            rx_data    <= 8'd0;
            rx_done    <= 1'b0;
            rx_error   <= 1'b0;
        end else begin
            rx_done  <= 1'b0;
            rx_error <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (rx == 1'b0) begin
                        sample_cnt <= 4'd0;
                        state      <= ST_START;
                    end
                end

                ST_START: begin
                    if (ovs_tick) begin
                        if (sample_cnt == 4'd7) begin
                            if (rx == 1'b0) begin
                                sample_cnt <= 4'd0;
                                bit_idx    <= 3'd0;
                                state      <= ST_DATA;
                            end else begin
                                state <= ST_IDLE;
                            end
                        end else begin
                            sample_cnt <= sample_cnt + 1'b1;
                        end
                    end
                end

                ST_DATA: begin
                    if (ovs_tick) begin
                        if (sample_cnt == 4'd15) begin
                            sample_cnt <= 4'd0;
                            shift_reg  <= {rx, shift_reg[7:1]};
                            if (bit_idx == 3'd7)
                                state <= ST_STOP;
                            else
                                bit_idx <= bit_idx + 1'b1;
                        end else begin
                            sample_cnt <= sample_cnt + 1'b1;
                        end
                    end
                end

                ST_STOP: begin
                    if (ovs_tick) begin
                        if (sample_cnt == 4'd15) begin
                            if (rx == 1'b1) begin
                                rx_data <= shift_reg;
                                rx_done <= 1'b1;
                            end else begin
                                rx_error <= 1'b1;
                            end
                            state <= ST_IDLE;
                        end else begin
                            sample_cnt <= sample_cnt + 1'b1;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule