`timescale 1ns / 1ps
//=============================================================================
// Module  : uart_tx
// Purpose : Serialize one byte over UART (8N1)
// Signals : start  - pulse high for 1 cycle to begin transmission
//           data   - byte to transmit (sampled on start)
//           busy   - high while transmitting
//           done   - 1-cycle pulse when last stop bit is sent
//           tx     - UART line (idle = 1)
//=============================================================================
module uart_tx_packet (
    input  wire       clk,
    input  wire       rst,
    input  wire       baud_tick,
    input  wire       start,
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy,
    output reg        done
);

    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_START = 2'd1;
    localparam [1:0] ST_DATA = 2'd2;
    localparam [1:0] ST_STOP = 2'd3;

    reg [1:0] state;
    reg [7:0] shift_reg;
    reg [2:0] bit_idx;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= ST_IDLE;
            shift_reg <= 8'd0;
            bit_idx   <= 3'd0;
            tx        <= 1'b1;
            busy      <= 1'b0;
            done      <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    if (start) begin
                        shift_reg <= data;
                        busy      <= 1'b1;
                        state     <= ST_START;
                    end
                end

                ST_START: begin
                    if (baud_tick) begin
                        tx      <= 1'b0;
                        bit_idx <= 3'd0;
                        state   <= ST_DATA;
                    end
                end

                ST_DATA: begin
                    if (baud_tick) begin
                        tx        <= shift_reg[0];
                        shift_reg <= {1'b0, shift_reg[7:1]};
                        if (bit_idx == 3'd7) state <= ST_STOP;
                        else bit_idx <= bit_idx + 1'b1;
                    end
                end

                ST_STOP: begin
                    if (baud_tick) begin
                        tx    <= 1'b1;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
