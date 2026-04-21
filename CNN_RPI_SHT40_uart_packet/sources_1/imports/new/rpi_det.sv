`timescale 1ns / 1ps
//=============================================================================
// Module  : rpi_det
// Purpose : Detect RPi GPIO signal rising edge with 3-stage synchronizer.
//           Outputs 1-cycle rpi_event only when the signal goes HIGH.
//=============================================================================
module rpi_det (
    input  logic clk,
    input  logic rst,
    input  logic rpi_signal,    // async GPIO from RPi
    output logic rpi_event,     // 1-cycle pulse on posedge
    output logic rpi_active     // synchronized stable state
);

    logic d0, d1, d2, d_prev;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            d0         <= 1'b0;
            d1         <= 1'b0;
            d2         <= 1'b0;
            d_prev     <= 1'b0;
            rpi_active <= 1'b0;
            rpi_event  <= 1'b0;
        end else begin
            d0         <= rpi_signal;
            d1         <= d0;
            d2         <= d1;
            d_prev     <= d2;
            rpi_active <= d2;
            
            // 상승 에지(0 -> 1)만 감지하도록 XOR(^) 대신 AND(&!) 사용
            rpi_event  <= d2 && !d_prev;
        end
    end

endmodule