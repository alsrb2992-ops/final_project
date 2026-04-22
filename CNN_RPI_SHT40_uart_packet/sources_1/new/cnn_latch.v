`timescale 1ns / 1ps
//=============================================================================
// Module  : cnn_latch
// Purpose : Latch CNN inference result on rising edge of cnn_valid.
//           Simplified to detect only 'person' status.
//=============================================================================
module cnn_latch (
    input  wire       clk,
    input  wire       rst,
    input  wire       cnn_signal,     // 이 신호가 HIGH가 되면 사람 인식으로 판단
    output reg  [7:0] cnn_data,       // 항상 8'h01 (사람)로 출력
    output reg        cnn_event
);

    reg signal_prev;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            signal_prev <= 1'b0;
            cnn_data    <= 8'h00;
            cnn_event   <= 1'b0;
        end else begin
            cnn_event   <= 1'b0;
            signal_prev <= cnn_signal;

            // 신호의 상승 에지 감지 시 데이터를 '1'로 고정
            if (cnn_signal && !signal_prev) begin
                cnn_data  <= 8'h01; // 신호 발생 = 사람 인식 확정
                cnn_event <= 1'b1;
            end
        end
    end

endmodule