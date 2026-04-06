// ============================================================
// distance_calc.sv
// Si raw 데이터에서 Distance(mm) 와 IS(간섭플래그) 추출
//
// Si 구조:
//   byte1 (LSB): [7:2]=Di[5:0], [1:0]=IS
//   byte2 (MSB): [7:0]=Di[13:6]
//
// Distance = Di[5:0] + Di[13:6] * 64  (단위: mm)
// ============================================================
module distance_calc (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [15:0] si_raw,        // {byte2, byte1}
    input  logic        si_valid,      // Si 1개 입력 펄스

    output logic [13:0] distance,      // mm 단위 거리
    output logic [1:0]  is_flag,       // 간섭 플래그
    output logic        calc_valid     // 계산 완료 펄스
);

// Di 분리
wire [5:0]  di_low  = si_raw[7:2];     // byte1 의 bit[7:2]
wire [7:0]  di_high = si_raw[15:8];    // byte2 전체

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        distance   <= '0;
        is_flag    <= '0;
        calc_valid <= '0;
    end else begin
        calc_valid <= '0;

        if (si_valid) begin
            is_flag    <= si_raw[1:0];
            // Distance = Di[5:0] + Di[13:6] * 64
            // *64 = <<6 이므로 곱셈 없이 쉬프트
            distance   <= {8'b0, di_low} + ({6'b0, di_high} << 6);
            calc_valid <= 1'b1;
        end
    end
end

endmodule
