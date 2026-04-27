`timescale 1ns / 1ps
`include "lidar_define.vh"

module left_right_comparator #(

    parameter CLK_FREQ             = 125_000_000,
    parameter DIR_CHANGE_FREQUENCY = 2_500_000,
    parameter TURN_THRESHOLD_MM    = 14'd800,
    parameter BIG_TURN_DIFF_MM     = 14'd100
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [13:0] left_min_distance,
    input  wire [13:0] right_min_distance,
    input  wire        warning_signal,
    output wire [ 2:0] direction_degree
);

    localparam DIR_CHANGE_COUNT = CLK_FREQ / DIR_CHANGE_FREQUENCY;

    reg [$clog2(DIR_CHANGE_COUNT)-1:0] change_clk_count;

    reg [2:0] c_direction_degree, n_direction_degree;
    reg tick;
    assign direction_degree = c_direction_degree;

    // tick 생성

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            tick <= 0;
            change_clk_count <= 0;
        end else begin
            if (change_clk_count == DIR_CHANGE_COUNT - 1) begin
                change_clk_count <= 0;
                tick <= 1;
            end else begin
                change_clk_count <= change_clk_count + 1;
                tick <= 0;
            end
        end
    end

    // DIR_CHANGE_FREQUENCY 마다 방향 바꾸기

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            c_direction_degree <= `CENTER;
        end else begin
            if (tick) begin
                c_direction_degree <= n_direction_degree;
            end
        end

    end

    // 왼쪽과 오른쪽의 차이나는 정도에 따라 방향을 다르게 꺾음
    always @(*) begin
        n_direction_degree = c_direction_degree;

        if (warning_signal) begin
            if (left_min_distance < right_min_distance) begin
                n_direction_degree = `TURN_RIGHT_BIG;  // 오른쪽으로 크게 꺾음
            end else begin
                n_direction_degree = `TURN_LEFT_BIG;  // 왼쪽으로 크게 꺾음
            end
        end else begin
            if(left_min_distance > TURN_THRESHOLD_MM && right_min_distance > TURN_THRESHOLD_MM) begin
                n_direction_degree = `CENTER;  // 직진
            end else if (left_min_distance < right_min_distance) begin
                if (right_min_distance - left_min_distance >BIG_TURN_DIFF_MM ) begin
                    n_direction_degree = `TURN_RIGHT_BIG;  // 오른쪽으로 크게 꺾음
                end else begin
                    n_direction_degree = `TURN_RIGHT_SMALL;  // 오른쪽으로 조금 꺾음
                end
            end else if (right_min_distance < left_min_distance) begin
                if (left_min_distance - right_min_distance > BIG_TURN_DIFF_MM) begin
                    n_direction_degree = `TURN_LEFT_BIG;  // 왼쪽으로 크게 꺾음
                end else begin
                    n_direction_degree = `TURN_LEFT_SMALL ;  // 왼쪽으로 조금 꺾음
                end
            end else begin
                n_direction_degree = `CENTER;  // 직진
            end
        end
    end

endmodule
