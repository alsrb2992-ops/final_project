`timescale 1ns / 1ps
`include "lidar_define.vh"

module left_right_comparator #(
    parameter CLK_FREQ             = 125_000_000,
    parameter DIR_CHANGE_FREQUENCY = 2_500_000,
    parameter TURN_THRESHOLD_MM    = 14'd800,
    parameter BIG_TURN_DIFF_MM     = 14'd500,
    parameter SMALL_TURN_DIFF_MM   = 14'd200
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [13:0] left_min_distance,
    input  wire [13:0] right_min_distance,
    input  wire        is_left_over,
    input  wire        is_right_over,
    input  wire        warning_signal,
    output wire [ 2:0] direction_degree
);

    // ===== 우선순위 정리 =====
    //
    // 1순위: only_right_over  → TURN_RIGHT_BIG (warning 켜져도 열린 쪽으로)
    // 2순위: only_left_over   → TURN_LEFT_BIG  (warning 켜져도 열린 쪽으로)
    // 3순위: warning=1        → 긴급 BIG (min_distance 기준)
    // 4순위: 양쪽 다 threshold → CENTER
    // 5순위: diff < SMALL     → CENTER
    // 6순위: min_distance 비교 → BIG/SMALL (히스테리시스)

    localparam DIR_CHANGE_COUNT = CLK_FREQ / DIR_CHANGE_FREQUENCY;

    reg [$clog2(DIR_CHANGE_COUNT)-1:0] change_clk_count;

    reg [2:0] c_direction_degree, n_direction_degree;
    reg tick;

    assign direction_degree = c_direction_degree;

    // ===== tick 생성 =====
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick             <= 1'b0;
            change_clk_count <= 0;
        end else begin
            if (change_clk_count == DIR_CHANGE_COUNT - 1) begin
                change_clk_count <= 0;
                tick             <= 1'b1;
            end else begin
                change_clk_count <= change_clk_count + 1;
                tick             <= 1'b0;
            end
        end
    end

    // ===== tick 마다 방향 래치 =====
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_direction_degree <= `CENTER;
        end else begin
            if (tick) begin
                c_direction_degree <= n_direction_degree;
            end
        end
    end

    // ===== 좌우 거리 차이 계산 =====
    wire        left_closer;
    wire [13:0] diff;

    assign left_closer = (left_min_distance < right_min_distance);
    assign diff        = left_closer ? (right_min_distance - left_min_distance)
                                     : (left_min_distance  - right_min_distance);

    // ===== over 신호 조합 =====
    wire only_left_over = is_left_over & ~is_right_over;
    wire only_right_over = is_right_over & ~is_left_over;

    // ===== 현재 방향 상태 (히스테리시스용) =====
    wire is_right_big = (c_direction_degree == `TURN_RIGHT_BIG);
    wire is_left_big = (c_direction_degree == `TURN_LEFT_BIG);

    // ===== 방향 판단 =====
    always @(*) begin
        n_direction_degree = c_direction_degree;

        // ===== 1순위: 오른쪽만 열림 → TURN_RIGHT_BIG =====
        // warning 켜져있어도 열린 쪽으로 가야 탈출 가능
        if (only_right_over) begin
            n_direction_degree = `TURN_RIGHT_BIG;

            // ===== 2순위: 왼쪽만 열림 → TURN_LEFT_BIG =====
        end else if (only_left_over) begin
            n_direction_degree = `TURN_LEFT_BIG;

            // ===== 3순위: warning → min_distance 기준 긴급 BIG 회피 =====
            // 양쪽 다 열렸거나 양쪽 다 막혔을 때 warning 처리
        end else if (warning_signal) begin
            if (left_closer) begin
                n_direction_degree = `TURN_RIGHT_BIG;
            end else begin
                n_direction_degree = `TURN_LEFT_BIG;
            end

            // ===== 4순위: 양쪽 다 충분히 멀리 → CENTER =====
        end else if (left_min_distance  > TURN_THRESHOLD_MM &&
                     right_min_distance > TURN_THRESHOLD_MM) begin
            n_direction_degree = `CENTER;

            // ===== 5순위: 차이가 작으면 → CENTER =====
        end else if (diff < SMALL_TURN_DIFF_MM) begin
            n_direction_degree = `CENTER;

            // ===== 6순위: 오른쪽이 더 가까움 → 왼쪽으로 회피 =====
        end else if (!left_closer) begin
            if (is_left_big) begin
                n_direction_degree = `TURN_LEFT_BIG;
            end else begin
                if (diff > BIG_TURN_DIFF_MM) begin
                    n_direction_degree = `TURN_LEFT_BIG;
                end else begin
                    n_direction_degree = `TURN_LEFT_SMALL;
                end
            end

            // ===== 7순위: 왼쪽이 더 가까움 → 오른쪽으로 회피 =====
        end else begin
            if (is_right_big) begin
                n_direction_degree = `TURN_RIGHT_BIG;
            end else begin
                if (diff > BIG_TURN_DIFF_MM) begin
                    n_direction_degree = `TURN_RIGHT_BIG;
                end else begin
                    n_direction_degree = `TURN_RIGHT_SMALL;
                end
            end
        end
    end

endmodule
