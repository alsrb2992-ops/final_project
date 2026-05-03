// ============================================================
// collision_detector.sv
// 각도 범위 + 거리 임계값으로 충돌 위험 판단
//
// 개선사항:
// 1. 히스테리시스 추가 (떨림 방지)
// 2. round_done 시 min 래치 + 리셋 (매 회전 갱신)
// 3. 좌우 독립 판단
// ============================================================

`include "lidar_define.vh"

module collision_detector #(
    parameter FRONT_ANGLE_DEG       = 9'd45,
    parameter BEHIND_ANGLE_DEG      = 9'd40,
    parameter RIGHT_START_ANGLE_DEG = 9'd45,
    parameter RIGHT_END_ANGLE_DEG   = 9'd90,
    parameter LEFT_START_ANGLE_DEG  = 9'd270,
    parameter LEFT_END_ANGLE_DEG    = 9'd315,
    parameter BRAKE_DIST_MM         = 14'd300,
    parameter WARN_DIST_MM          = 14'd600,
    parameter SIDE_DIST_MM          = 14'd300,
    parameter HYSTERESIS_MM         = 14'd100
) (
    input wire clk,
    input wire rst_n,

    input wire [13:0] distance,
    input wire [ 8:0] angle,
    input wire        data_valid,
    input wire        round_done,

    output reg brake_signal,
    output reg warning_signal,
    output wire side_warning_signal,
    output wire [13:0] left_min_distance,
    output wire [13:0] right_min_distance
);

    // ===== 히스테리시스 임계값 =====
    localparam BRAKE_ON_THRESHOLD = BRAKE_DIST_MM;
    localparam BRAKE_OFF_THRESHOLD = BRAKE_DIST_MM + HYSTERESIS_MM;

    localparam WARN_ON_THRESHOLD = WARN_DIST_MM;
    localparam WARN_OFF_THRESHOLD = WARN_DIST_MM + HYSTERESIS_MM;

    // ===== 각도 영역 판단 =====
    wire in_front_zone = (angle <= FRONT_ANGLE_DEG) ||
                         (angle >= (9'd360 - FRONT_ANGLE_DEG));

    wire be_hind_zone = (angle <= 9'd180 + BEHIND_ANGLE_DEG) &&
                        (angle >= (9'd180 - BEHIND_ANGLE_DEG));

    wire right_zone = (angle <= RIGHT_END_ANGLE_DEG) &&
                      (angle >= RIGHT_START_ANGLE_DEG);

    wire left_zone = (angle <= LEFT_END_ANGLE_DEG) &&
                     (angle >= LEFT_START_ANGLE_DEG);

    // ===== 전방 거리 수집 및 위험 판단 (히스테리시스 적용) =====
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brake_signal   <= 1'b0;
            warning_signal <= 1'b0;
        end else begin
            if (data_valid && in_front_zone && distance != 14'd0) begin
                // 브레이크 신호
                if (distance <= BRAKE_ON_THRESHOLD) begin
                    brake_signal   <= 1'b1;
                    warning_signal <= 1'b1;
                end else if (distance >= BRAKE_OFF_THRESHOLD && brake_signal) begin
                    brake_signal <= 1'b0;
                end

                // 경고 신호
                if (distance <= WARN_ON_THRESHOLD) begin
                    warning_signal <= 1'b1;
                end else if (distance >= WARN_OFF_THRESHOLD && warning_signal) begin
                    warning_signal <= 1'b0;
                end
            end
        end
    end

    // ===== 측면 거리 수집 및 위험 판단 =====
    // 스캔 중 누적되는 min 값
    reg [13:0] c_left_min_distance, c_right_min_distance;
    reg [13:0] n_left_min_distance, n_right_min_distance;

    // round_done 시 래치되는 확정 값 (판단에 사용)
    reg [13:0] c_left_min_reg, c_right_min_reg;
    reg [13:0] n_left_min_reg, n_right_min_reg;

    // 경고 신호
    reg c_left_warning_signal, n_left_warning_signal;
    reg c_right_warning_signal, n_right_warning_signal;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_left_min_distance    <= 14'h3FFF;
            c_right_min_distance   <= 14'h3FFF;
            c_left_min_reg         <= 14'h3FFF;  // FIX: 0 → 3FFF
            c_right_min_reg        <= 14'h3FFF;  // FIX: 0 → 3FFF
            c_left_warning_signal  <= 1'b0;
            c_right_warning_signal <= 1'b0;
        end else begin
            c_left_min_distance    <= n_left_min_distance;
            c_right_min_distance   <= n_right_min_distance;
            c_left_min_reg         <= n_left_min_reg;
            c_right_min_reg        <= n_right_min_reg;
            c_left_warning_signal  <= n_left_warning_signal;
            c_right_warning_signal <= n_right_warning_signal;
        end
    end

    // 측면 거리 측정 + 히스테리시스 판단
    localparam SIDE_ON_THRESHOLD = SIDE_DIST_MM;
    localparam SIDE_OFF_THRESHOLD = SIDE_DIST_MM + HYSTERESIS_MM;

    always @(*) begin
        // 기본값: 이전 상태 유지
        n_left_min_distance    = c_left_min_distance;
        n_right_min_distance   = c_right_min_distance;
        n_left_min_reg         = c_left_min_reg;
        n_right_min_reg        = c_right_min_reg;
        n_left_warning_signal  = c_left_warning_signal;
        n_right_warning_signal = c_right_warning_signal;

        // ===== 스캔 중: min 값 누적 갱신 =====
        if (data_valid) begin
            if (left_zone && distance != 14'd0) begin
                if (distance < c_left_min_distance) begin
                    n_left_min_distance = distance;
                end
            end else if (right_zone && distance != 14'd0) begin
                if (distance < c_right_min_distance) begin
                    n_right_min_distance = distance;
                end
            end
        end

        // ===== 좌우 독립 판단 (래치된 확정값 기준) =====
        // 왼쪽 벽이 가까우면 → left_warning ON
        if (c_left_min_reg < SIDE_ON_THRESHOLD) begin
            n_left_warning_signal = 1'b1;
        end else if (c_left_min_reg >= SIDE_OFF_THRESHOLD && c_left_warning_signal) begin
            n_left_warning_signal = 1'b0;
        end

        // 오른쪽 벽이 가까우면 → right_warning ON
        if (c_right_min_reg < SIDE_ON_THRESHOLD) begin  // FIX: n_ → c_
            n_right_warning_signal = 1'b1;
        end else if (c_right_min_reg >= SIDE_OFF_THRESHOLD && c_right_warning_signal) begin
            n_right_warning_signal = 1'b0;
        end

        // ===== 1회전 완료: 래치 + 리셋 =====
        if (round_done) begin
            n_left_min_reg       = c_left_min_distance;
            n_right_min_reg      = c_right_min_distance;
            n_left_min_distance  = 14'h3FFF;
            n_right_min_distance = 14'h3FFF;
        end
    end

    assign side_warning_signal = c_left_warning_signal || c_right_warning_signal;
    assign left_min_distance = c_left_min_reg;
    assign right_min_distance = c_right_min_reg;

endmodule
