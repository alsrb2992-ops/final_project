`timescale 1ns / 1ps
//=============================================================================
// Module  : baud_gen
// Purpose : Generate baud-rate tick (1 pulse per bit period)
// Params  : CLK_FREQ  - system clock frequency in Hz  (default 100 MHz)
//           BAUD_RATE - desired baud rate in bps       (default 115200)
//=============================================================================
module baud_gen #(
    parameter integer CLK_FREQ  = 100_000_000,
    parameter integer BAUD_RATE = 115_200
)(
    input  wire clk,
    input  wire rst,
    output reg  tick        // 1-cycle pulse every bit period
);

    localparam integer DIVISOR = CLK_FREQ / BAUD_RATE;  // 868 @ 100MHz/115200

    reg [$clog2(DIVISOR)-1:0] cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt  <= 0;
            tick <= 1'b0;
        end else if (cnt == DIVISOR - 1) begin
            cnt  <= 0;
            tick <= 1'b1;
        end else begin
            cnt  <= cnt + 1'b1;
            tick <= 1'b0;
        end
    end

endmodule