`timescale 1ns / 1ps
`include "lidar_define.vh"

module left_right_comparator #(
    parameter TURN_THRESHOLD_MM = 14'd800,
    parameter BIG_TURN_DIFF_MM  = 14'd100
) (
    input  wire [13:0] left_min_distance,
    input  wire [13:0] right_min_distance,
    input  wire        warning_signal,
    output reg  [ 2:0] direction_degree
);

    // 왼쪽과 오른쪽의 차이나는 정도에 따라 방향을 다르게 꺾음
    always @(*) begin
        if (warning_signal) begin
            if (left_min_distance < right_min_distance) begin
                direction_degree = `TURN_RIGHT_BIG;  // 오른쪽으로 크게 꺾음
            end else begin
                direction_degree = `TURN_LEFT_BIG;  // 왼쪽으로 크게 꺾음
            end
        end else begin
            if(left_min_distance > TURN_THRESHOLD_MM && right_min_distance > TURN_THRESHOLD_MM) begin
                direction_degree = `CENTER;  // 직진
            end else if (left_min_distance < right_min_distance) begin
                if (right_min_distance - left_min_distance >BIG_TURN_DIFF_MM ) begin
                    direction_degree = `TURN_RIGHT_BIG;  // 오른쪽으로 크게 꺾음
                end else begin
                    direction_degree = `TURN_RIGHT_SMALL;  // 오른쪽으로 조금 꺾음
                end
            end else if (right_min_distance < left_min_distance) begin
                if (left_min_distance - right_min_distance > BIG_TURN_DIFF_MM) begin
                    direction_degree = `TURN_LEFT_BIG;  // 왼쪽으로 크게 꺾음
                end else begin
                    direction_degree = `TURN_LEFT_SMALL ;  // 왼쪽으로 조금 꺾음
                end
            end else begin
                direction_degree = `CENTER;  // 직진
            end
        end
    end

endmodule
