`timescale 1ns / 1ps
`include "lidar_define.vh"

module left_right_comparator #(
    parameter CLK_FREQ = 125_000_000,
    parameter DIR_CHANGE_FREQUENCY = 2_500_000,
    parameter TURN_THRESHOLD_MM    = 14'd800,   // 양쪽 다 이 이상이면 직진
    parameter BIG_TURN_DIFF_MM = 14'd500,  // 이 이상 차이나면 BIG
    parameter SMALL_TURN_DIFF_MM   = 14'd200    // 이 이상 차이나면 SMALL, 미만이면 CENTER
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

    // ===== 동작 정리 =====
    //
    //        0        200              500
    //        |         |                |
    //      CENTER  | SMALL    |   BIG
    //
    //  diff < SMALL_TURN_DIFF_MM(200) → CENTER  (차이 작으면 직진)
    //  diff 200 ~ 500                 → SMALL
    //  diff > BIG_TURN_DIFF_MM(500)   → BIG
    //
    //  히스테리시스:
    //  BIG   → SMALL : diff < SMALL_TURN_DIFF_MM(200) 이 되어야 내려감
    //                  (200~500 구간에서는 BIG 유지)

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

    wire only_left_over = is_left_over & ~is_right_over;
    wire only_right_over = ~is_left_over & is_right_over;


    // 현재 방향 상태
    wire is_right_big = (c_direction_degree == `TURN_RIGHT_BIG);
    wire is_left_big = (c_direction_degree == `TURN_LEFT_BIG);

    // ===== 방향 판단 =====
    always @(*) begin
        n_direction_degree = c_direction_degree;

        // ===== 1순위: warning → 무조건 BIG으로 긴급 회피 =====
        if (warning_signal) begin
            if (left_closer) begin
                n_direction_degree = `TURN_RIGHT_BIG;
            end else begin
                n_direction_degree = `TURN_LEFT_BIG;
            end

            // ===== 2순위: 양쪽 다 충분히 멀리 → CENTER =====
        end else if (only_left_over) begin

            n_direction_degree = `TURN_LEFT_BIG;
        end else if (only_right_over) begin

            n_direction_degree = `TURN_RIGHT_BIG;
        end else if (left_min_distance  > TURN_THRESHOLD_MM &&
                     right_min_distance > TURN_THRESHOLD_MM) begin
            n_direction_degree = `CENTER;

            // ===== 3순위: 차이가 작으면 → CENTER =====
        end else if (diff < SMALL_TURN_DIFF_MM) begin
            n_direction_degree = `CENTER;

            // ===== 4순위: 오른쪽이 더 가까움 → 왼쪽으로 회피 =====
        end else if (!left_closer) begin
            if (is_left_big) begin
                // 현재 LEFT_BIG → SMALL_DIFF 미만이 되어야 내려감 (히스테리시스)
                // diff >= SMALL_TURN_DIFF_MM 이므로 여기선 BIG 유지
                n_direction_degree = `TURN_LEFT_BIG;
            end else begin
                if (diff > BIG_TURN_DIFF_MM) begin
                    n_direction_degree = `TURN_LEFT_BIG;
                end else begin
                    n_direction_degree = `TURN_LEFT_SMALL;
                end
            end

            // ===== 5순위: 왼쪽이 더 가까움 → 오른쪽으로 회피 =====
        end else begin
            if (is_right_big) begin
                // 현재 RIGHT_BIG → SMALL_DIFF 미만이 되어야 내려감 (히스테리시스)
                // diff >= SMALL_TURN_DIFF_MM 이므로 여기선 BIG 유지
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
