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
    parameter FRONT_ANGLE_DEG = 9'd20,
    parameter BRAKE_DIST_MM   = 14'd300,
    parameter WARN_DIST_MM    = 14'd400
) (
    input logic clk,
    input logic rst_n,

    input logic [13:0] distance,
    input logic [ 8:0] angle,
    input logic        data_valid,
    input logic        round_done,

    output logic brake_signal,
    output logic warning_signal
);


    wire in_front_zone = (angle <= FRONT_ANGLE_DEG) ||
                         (angle >= (9'd360 - FRONT_ANGLE_DEG));

    // 1회전 동안 위험 누적 (해제 판단용)
    logic danger_in_round;
    logic warn_in_round;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brake_signal    <= 1'b0;
            warning_signal  <= 1'b0;
            danger_in_round <= 1'b0;
            warn_in_round   <= 1'b0;
        end else begin

            // 1. 위험 감지 즉시 출력 + 누적
            if (data_valid && in_front_zone) begin
                if (distance <= BRAKE_DIST_MM && distance != 14'd0) begin
                    brake_signal    <= 1'b1;   // 즉시 출력
                    warning_signal  <= 1'b1;
                    danger_in_round <= 1'b1;
                    warn_in_round   <= 1'b1;
                end else if (distance <= WARN_DIST_MM && distance != 14'd0) begin
                    warning_signal <= 1'b1;  // 즉시 출력
                    warn_in_round  <= 1'b1;
                end
            end

            // 2. 1회전 완료 시 위험 없었으면 해제
            if (round_done) begin
                if (!danger_in_round) brake_signal <= 1'b0;
                if (!warn_in_round) warning_signal <= 1'b0;
                danger_in_round <= 1'b0;  // 다음 회전 누적 리셋
                warn_in_round   <= 1'b0;
            end

        end
    end

endmodule
