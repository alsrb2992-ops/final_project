// ============================================================
// collision_detector.sv
// 각도 범위 + 거리 임계값으로 충돌 위험 판단
//
// 개선사항:
// 1. 히스테리시스 추가 (떨림 방지)
//    - 켜는 문턱: BRAKE_DIST_MM
//    - 끄는 문턱: BRAKE_DIST_MM + HYSTERESIS_MM
// 2. 기존 기능 유지 (round_done 해제 로직)
// ============================================================

`include "lidar_define.vh"

module collision_detector #(
    parameter FRONT_ANGLE_DEG       = 9'd45,
    parameter BEHIND_ANGLE_DEG      = 9'd40,
    parameter RIGHT_START_ANGLE_DEG = 9'd45,
    parameter RIGHT_END_ANGLE_DEG   = 9'd90,
    parameter LEFT_START_ANGLE_DEG  = 9'd270,
    parameter LEFT_END_ANGLE_DEG    = 9'd315,
    parameter BRAKE_DIST_MM         = 14'd300,  // 브레이크 켜는 거리
    parameter WARN_DIST_MM          = 14'd600,  // 경고 켜는 거리
    parameter SIDE_DIST_MM          = 14'd300,
    parameter HYSTERESIS_MM         = 14'd100   // 히스테리시스 (100mm)
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
    localparam BRAKE_OFF_THRESHOLD = BRAKE_DIST_MM + HYSTERESIS_MM;  // 400mm

    localparam WARN_ON_THRESHOLD = WARN_DIST_MM;
    localparam WARN_OFF_THRESHOLD = WARN_DIST_MM + HYSTERESIS_MM;  // 700mm

    // ===== 각도 영역 판단 =====
    wire in_front_zone = (angle <= FRONT_ANGLE_DEG) ||
                         (angle >= (9'd360 - FRONT_ANGLE_DEG));

    wire be_hind_zone = (angle <= 9'd180 + BEHIND_ANGLE_DEG) &&
                         (angle >= (9'd180 - BEHIND_ANGLE_DEG));

    wire right_zone = (angle <= RIGHT_END_ANGLE_DEG) &&
                         (angle >=  RIGHT_START_ANGLE_DEG);

    wire left_zone = (angle <= LEFT_END_ANGLE_DEG) &&
                       (angle >=  LEFT_START_ANGLE_DEG);

    // ===== 전방 거리 수집 및 위험 판단 (히스테리시스 적용) =====
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brake_signal   <= 1'b0;
            warning_signal <= 1'b0;
        end else begin

            // 1. 위험 감지 - 히스테리시스 적용
            if (data_valid && in_front_zone && distance != 14'd0) begin
                // 브레이크 신호
                if (distance <= BRAKE_ON_THRESHOLD) begin
                    brake_signal   <= 1'b1;  // 켜기 (300mm 이하)
                    warning_signal <= 1'b1;
                end else if (distance >= BRAKE_OFF_THRESHOLD && brake_signal) begin
                    brake_signal <= 1'b0;  // 끄기 (400mm 이상)
                end
                // brake_signal은 300~400mm 사이에서 이전 상태 유지

                // 경고 신호
                if (distance <= WARN_ON_THRESHOLD) begin
                    warning_signal <= 1'b1;  // 켜기 (600mm 이하)
                end else if (distance >= WARN_OFF_THRESHOLD && warning_signal) begin
                    warning_signal <= 1'b0;  // 끄기 (700mm 이상)
                end
                // warning_signal은 600~700mm 사이에서 이전 상태 유지
            end

            // 2. 1회전 완료 시 강제 해제 (안전 보장)
            if (round_done) begin
                brake_signal   <= 1'b0;
                warning_signal <= 1'b0;
            end
        end
    end

    // ===== 측면 거리 수집 및 위험 판단 =====
    reg [13:0] c_left_min_distance, c_right_min_distance;
    reg [13:0] n_left_min_distance, n_right_min_distance;

    reg c_left_warning_signal, n_left_warning_signal;
    reg c_right_warning_signal, n_right_warning_signal;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_left_min_distance    <= 14'h3FFF;
            c_right_min_distance   <= 14'h3FFF;
            c_left_warning_signal  <= 0;
            c_right_warning_signal <= 0;
        end else begin
            c_left_min_distance    <= n_left_min_distance;
            c_right_min_distance   <= n_right_min_distance;
            c_left_warning_signal  <= n_left_warning_signal;
            c_right_warning_signal <= n_right_warning_signal;
        end
    end

    // 측면 거리 측정 (히스테리시스 추가 버전)
    localparam SIDE_ON_THRESHOLD = SIDE_DIST_MM;
    localparam SIDE_OFF_THRESHOLD = SIDE_DIST_MM + HYSTERESIS_MM;

    always @(*) begin
        n_left_min_distance    = c_left_min_distance;
        n_right_min_distance   = c_right_min_distance;
        n_left_warning_signal  = c_left_warning_signal;
        n_right_warning_signal = c_right_warning_signal;

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

        // 측면 경고 - 히스테리시스 적용
        if (c_left_min_distance < SIDE_ON_THRESHOLD && 
            c_right_min_distance < c_left_min_distance) begin
            n_right_warning_signal = 1'b1;
        end else if (c_left_min_distance >= SIDE_OFF_THRESHOLD && c_right_warning_signal) begin
            n_right_warning_signal = 1'b0;
        end

        if (c_right_min_distance < SIDE_ON_THRESHOLD && 
            c_left_min_distance < c_right_min_distance) begin
            n_left_warning_signal = 1'b1;
        end else if (c_right_min_distance >= SIDE_OFF_THRESHOLD && c_left_warning_signal) begin
            n_left_warning_signal = 1'b0;
        end

        // 1회전 완료 시 리셋
        if (round_done) begin
            n_left_min_distance    = 14'h3FFF;
            n_right_min_distance   = 14'h3FFF;
            n_right_warning_signal = 1'b0;
            n_left_warning_signal  = 1'b0;
        end
    end

    assign side_warning_signal = c_left_warning_signal || c_right_warning_signal;
    assign left_min_distance = c_left_min_distance;
    assign right_min_distance = c_right_min_distance;

endmodule
