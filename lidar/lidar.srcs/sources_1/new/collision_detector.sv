// ============================================================
// collision_detector.sv
// 각도 범위 + 거리 임계값으로 충돌 위험 판단
//
// 전방 각도 범위: 0° ± FRONT_ANGLE_DEG (기본 ±20°)
//   → 0°~20° 와 340°~359° 구간
// 거리 임계값: BRAKE_DIST_MM 이하면 제동
// ============================================================
// ============================================================
// collision_detector.sv
// 각도 범위 + 거리 임계값으로 충돌 위험 판단
//
// 감지 즉시 brake/warning 출력
// 해제는 1회전 완료(round_done) 후 위험 없을 때만
// ============================================================
module collision_detector #(
    parameter FRONT_ANGLE_DEG       = 9'd45,
    parameter BEHIND_ANGLE_DEG      = 9'd40,
    parameter RIGHT_START_ANGLE_DEG = 9'd45,
    parameter RIGHT_END_ANGLE_DEG   = 9'd90,
    parameter LEFT_START_ANGLE_DEG  = 9'd270,
    parameter LEFT_END_ANGLE_DEG    = 9'd315,
    parameter BRAKE_DIST_MM         = 14'd300,
    parameter WARN_DIST_MM          = 14'd400,
    parameter SIDE_DIST_MM          = 14'd300
) (
    input logic clk,
    input logic rst_n,

    input logic [13:0] distance,
    input logic [ 8:0] angle,
    input logic        data_valid,
    input logic        round_done,

    output logic brake_signal,
    output logic warning_signal,
    output logic left_warning_signal,
    output logic right_warning_signal,
    output logic [13:0] left_min_distance,
    output logic [13:0] right_min_distance
);



    wire in_front_zone = (angle <= FRONT_ANGLE_DEG) ||
                         (angle >= (9'd360 - FRONT_ANGLE_DEG));

    wire be_hind_zone = (angle <= 9'd180 + BEHIND_ANGLE_DEG) &&
                         (angle >= (9'd180 - BEHIND_ANGLE_DEG));

    wire right_zone = (angle <= RIGHT_END_ANGLE_DEG) &&
                         (angle >=  RIGHT_START_ANGLE_DEG);

    wire left_zone = (angle <= LEFT_END_ANGLE_DEG) &&
                       (angle >=  LEFT_START_ANGLE_DEG);

    // 1회전 동안 위험 누적 (해제 판단용)

    // ------------------------------------------------
    // 전방 거리 수집 및 위험 판단
    // ------------------------------------------------


    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brake_signal   <= 1'b0;
            warning_signal <= 1'b0;
        end else begin

            // 1. 위험 감지 즉시 출력 + 누적
            if (data_valid && in_front_zone) begin
                if (distance <= BRAKE_DIST_MM && distance != 14'd0) begin
                    brake_signal   <= 1'b1;  // 즉시 출력
                    warning_signal <= 1'b1;
                end else if (distance <= WARN_DIST_MM && distance != 14'd0) begin
                    warning_signal <= 1'b1;  // 즉시 출력
                end
            end


            // 2. 1회전 완료 시 위험 없었으면 해제
            if (round_done) begin
                brake_signal   <= 1'b0;
                warning_signal <= 1'b0;
            end
        end
    end

    //---------------------------------------------------------------
    // 측면 거리 수집 및 위험 판단
    //---------------------------------------------------------------

    logic [13:0] c_left_min_distance, c_right_min_distance;
    logic [13:0] n_left_min_distance, n_right_min_distance;

    logic c_left_warning_signal, n_left_warning_signal;
    logic c_right_warning_signal, n_right_warning_signal;
    logic c_left_zone_reg, n_left_zone_reg;
    logic c_right_zone_reg, n_right_zone_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_left_min_distance    <= 14'h3FFF;
            c_right_min_distance   <= 14'h3FFF;
            c_left_warning_signal  <= 0;
            c_right_warning_signal <= 0;
            c_left_zone_reg <= 0;
            c_right_zone_reg <= 0;
        end else begin
            c_left_min_distance    <= n_left_min_distance;
            c_right_min_distance   <= n_right_min_distance;
            c_left_warning_signal  <= n_left_warning_signal;
            c_right_warning_signal <= n_right_warning_signal;
            c_left_zone_reg <= n_left_zone_reg;
            c_right_zone_reg <= n_right_zone_reg;
        end
    end

    always_comb begin  // 거리 측정 및 위험 감지
        n_left_min_distance    = c_left_min_distance;
        n_right_min_distance   = c_right_min_distance;
        n_left_warning_signal  = c_left_warning_signal;
        n_right_warning_signal = c_right_warning_signal;
        n_left_zone_reg = c_left_zone_reg;
        n_right_zone_reg = c_right_zone_reg;

        if (data_valid) begin
            n_left_zone_reg  = left_zone;
            n_right_zone_reg = right_zone;
            if (left_zone) begin  // 왼쪽 범위
                if (distance < c_left_min_distance && distance != 14'd0) begin // 왼쪽 최솟값 수집
                    n_left_min_distance = distance;
                end


            end else if (right_zone) begin  // 오른쪽 범위
                if (distance < c_right_min_distance && distance != 14'd0) begin // 오른쪽 최솟값 수집
                    n_right_min_distance = distance;
                end

            end
            if (c_left_min_distance < SIDE_DIST_MM && c_left_zone_reg ) begin // 왼쪽 위험 판단
                n_left_warning_signal = 1'b1;
            end

            if (c_right_min_distance < SIDE_DIST_MM && c_right_zone_reg) begin // 오른쪽 위험 판단
                n_right_warning_signal = 1'b1;
            end
        end



        if (round_done) begin
            n_left_min_distance    = 14'h3FFF;
            n_right_min_distance   = 14'h3FFF;
            n_right_warning_signal = 1'b0;
            n_left_warning_signal  = 1'b0;
        end

    end


    assign left_warning_signal = c_left_warning_signal;
    assign right_warning_signal = c_right_warning_signal;
    assign left_min_distance = c_left_min_distance;
    assign right_min_distance = c_right_min_distance;

endmodule
