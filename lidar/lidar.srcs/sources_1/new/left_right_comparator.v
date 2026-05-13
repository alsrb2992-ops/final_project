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

    // ===== 로직 구조 =====
    //
    // [Step 1] 방향 결정 (어느 쪽으로 꺾을지)
    //   1순위: only_right_over → 오른쪽
    //   2순위: only_left_over  → 왼쪽
    //   3순위: 거리 계산       → 더 먼 쪽
    //   없음:                  → CENTER
    //
    // [Step 2] 각도 크기 결정 (얼마나 꺾을지)
    //   warning=1 → 무조건 BIG
    //   warning=0 → diff 계산으로 BIG/SMALL/CENTER (히스테리시스 적용)

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

    // ===== Step 1: 방향 결정 (1=오른쪽, 0=왼쪽) =====
    // 방향만 결정하고, 각도 크기는 Step 2에서 결정
    reg  turn_right;  // 1: 오른쪽, 0: 왼쪽
    reg  do_turn;  // 1: 회전 필요, 0: CENTER

    always @(*) begin
        turn_right = 1'b0;
        do_turn    = 1'b1;

        // 1순위: 오른쪽만 열림 → 오른쪽
        if (only_right_over) begin
            turn_right = 1'b1;
            do_turn    = 1'b1;

            // 2순위: 왼쪽만 열림 → 왼쪽
        end else if (only_left_over) begin
            turn_right = 1'b0;
            do_turn    = 1'b1;

            // 3순위: 양쪽 다 충분히 멀리 → CENTER
        end else if (left_min_distance  > TURN_THRESHOLD_MM &&
                     right_min_distance > TURN_THRESHOLD_MM) begin
            do_turn = 1'b0;

            // 4순위: 차이 작으면 → CENTER (warning 없을 때만 적용, Step2에서 처리)
            // 5순위: 거리 계산 → 더 먼 쪽
        end else begin
            turn_right = left_closer;   // 왼쪽이 가까우면 오른쪽으로
            do_turn = 1'b1;
        end
    end

    // ===== Step 2: 각도 크기 결정 =====
    always @(*) begin
        n_direction_degree = c_direction_degree;

        if (!do_turn) begin
            // 회전 불필요 → CENTER
            n_direction_degree = `CENTER;

        end else if (warning_signal) begin
            // warning=1 → 무조건 BIG
            if (turn_right) begin
                n_direction_degree = `TURN_RIGHT_BIG;
            end else begin
                n_direction_degree = `TURN_LEFT_BIG;
            end

        end else begin
            // warning=0 → diff 계산으로 BIG/SMALL/CENTER
            if (diff < SMALL_TURN_DIFF_MM) begin
                // 차이 작으면 CENTER (only_over 아닐 때만 적용됨)
                // only_over 이면 do_turn=1이고 diff는 무의미하므로 BIG으로 감
                if (only_right_over || only_left_over) begin
                    // over 신호 있으면 diff 무시하고 BIG
                    if (turn_right) begin
                        n_direction_degree = `TURN_RIGHT_BIG;
                    end else begin
                        n_direction_degree = `TURN_LEFT_BIG;
                    end
                end else begin
                    n_direction_degree = `CENTER;
                end

            end else if (turn_right) begin
                // 오른쪽으로 꺾기
                if (is_right_big) begin
                    // 히스테리시스: BIG 유지
                    n_direction_degree = `TURN_RIGHT_BIG;
                end else begin
                    if (diff > BIG_TURN_DIFF_MM) begin
                        n_direction_degree = `TURN_RIGHT_BIG;
                    end else begin
                        n_direction_degree = `TURN_RIGHT_SMALL;
                    end
                end

            end else begin
                // 왼쪽으로 꺾기
                if (is_left_big) begin
                    // 히스테리시스: BIG 유지
                    n_direction_degree = `TURN_LEFT_BIG;
                end else begin
                    if (diff > BIG_TURN_DIFF_MM) begin
                        n_direction_degree = `TURN_LEFT_BIG;
                    end else begin
                        n_direction_degree = `TURN_LEFT_SMALL;
                    end
                end
            end
        end
    end

endmodule
