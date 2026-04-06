// ============================================================
// angle_calc.sv
// FSA, LSA, LSN 으로 각 Si 포인트의 각도 계산 (정수부만)
//
// 1단계:
//   Angle_FSA = (FSA >> 1) >> 6  → 정수 각도
//   Angle_LSA = (LSA >> 1) >> 6
//   중간 각도 = FSA + diff/(LSN-1) * i  (선형 보간)
//
// 충돌방지 목적이므로 정수 각도만 사용
// ============================================================
module angle_calc (
    input  logic        clk,
    input  logic        rst_n,

    // packet_parser 로부터
    input  logic [15:0] fsa_raw,
    input  logic [15:0] lsa_raw,
    input  logic [7:0]  lsn,
    input  logic        si_valid,      // Si 1개마다 펄스
    input  logic        pkt_start,     // 새 패킷 시작

    // 현재 Si 의 각도 (정수, 도 단위)
    output logic [8:0]  angle_deg,     // 0~359°
    output logic        angle_valid
);

// FSA, LSA 정수 각도
wire [8:0] angle_fsa = (fsa_raw >> 1) >> 6;
wire [8:0] angle_lsa = (lsa_raw >> 1) >> 6;

// diff: FSA → LSA 각도 차이 (시계방향)
// wrap-around 처리
wire [8:0] raw_diff  = angle_lsa - angle_fsa;
wire [8:0] angle_diff = (angle_lsa >= angle_fsa) ?
                         raw_diff :
                         (9'd360 - angle_fsa + angle_lsa);

// Si 인덱스 카운터 (0부터 시작)
logic [7:0] si_idx;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        si_idx      <= '0;
        angle_deg   <= '0;
        angle_valid <= '0;
    end else begin
        angle_valid <= '0;

        if (pkt_start) begin
            si_idx <= '0;
        end

        if (si_valid) begin
            if (lsn == 8'd1) begin
                // 시작 패킷: 포인트 1개, FSA = LSA
                angle_deg <= angle_fsa;
            end else begin
                // 선형 보간: angle = FSA + diff * i / (LSN-1)
                // 나눗셈 대신 근사: diff * i / (LSN-1)
                // LSN 최대 40 이므로 오버플로 주의
                angle_deg <= angle_fsa +
                             (angle_diff * {1'b0, si_idx}) /
                             ({1'b0, lsn} - 9'd1);
            end

            angle_valid <= 1'b1;
            si_idx      <= si_idx + 1;
        end
    end
end

endmodule
