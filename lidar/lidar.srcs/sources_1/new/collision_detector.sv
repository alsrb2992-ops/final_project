// ============================================================
// collision_detector.sv
// 각도 범위 + 거리 임계값으로 충돌 위험 판단
//
// 전방 각도 범위: 0° ± FRONT_ANGLE_DEG (기본 ±20°)
//   → 0°~20° 와 340°~359° 구간
// 거리 임계값: BRAKE_DIST_MM 이하면 제동
// ============================================================
module collision_detector #(
    parameter FRONT_ANGLE_DEG = 9'd20,
    parameter BRAKE_DIST_MM   = 14'd500
) (
    input logic clk,
    input logic rst_n,

    input logic [13:0] distance,
    input logic [ 8:0] angle,
    input logic        data_valid,
    input logic        pkt_done,    // 패킷 1개 완료 펄스 추가

    output logic brake_signal,
    output logic warning_signal
);

    localparam WARN_DIST_MM = BRAKE_DIST_MM * 2;

    wire in_front_zone = (angle <= FRONT_ANGLE_DEG) ||
                     (angle >= (9'd360 - FRONT_ANGLE_DEG));

    // 패킷 내 위험 감지 플래그 (pkt_done 까지 유지)
    logic danger_in_pkt;
    logic warn_in_pkt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brake_signal   <= '0;
            warning_signal <= '0;
            danger_in_pkt  <= '0;
            warn_in_pkt    <= '0;
        end else begin

            // Si 포인트마다 위험 감지
            if (data_valid && in_front_zone) begin
                if (distance <= BRAKE_DIST_MM && distance != 0) begin
                    danger_in_pkt <= 1'b1;
                    warn_in_pkt   <= 1'b1;
                end else if (distance <= WARN_DIST_MM && distance != 0) begin
                    warn_in_pkt <= 1'b1;
                end
            end

            // 패킷 완료 시점에 결과 출력 + 플래그 리셋
            if (pkt_done) begin
                brake_signal   <= danger_in_pkt;
                warning_signal <= warn_in_pkt;
                danger_in_pkt  <= '0;   // 다음 패킷을 위해 리셋
                warn_in_pkt    <= '0;
            end
        end
    end

endmodule
