// ============================================================
// distance_calc.sv
// Si raw 데이터에서 Distance(mm) 와 IS(간섭플래그) 추출
//
// Si 구조 (매뉴얼 5.2 ~ 5.3):
//   byte1 (LSB): bit[7:2] = Di[5:0]
//                bit[1:0] = IS (간섭 플래그)
//   byte2 (MSB): bit[7:0] = Di[13:6]
//
// Distance = Di[5:0] + Di[13:6] * 64  (단위: mm)
//
// IS = 0 → 정상
// IS = 2 → 경면반사 간섭
// IS = 3 → 주변광 간섭
// ============================================================
module distance_calc (
    input logic clk,
    input logic rst_n,

    input logic [15:0] si_raw,   // {byte2, byte1}
    input logic        si_valid, // Si 1개 입력 펄스

    output logic [13:0] distance,   // mm 단위 거리 (최대 16383mm)
    output logic [ 1:0] is_flag,    // 간섭 플래그
    output logic        calc_valid  // 계산 완료 펄스
);

    // ----------------------------------------------------------
    // Di 비트 분리
    // byte1 = si_raw[7:0], byte2 = si_raw[15:8]
    // ----------------------------------------------------------
    wire [ 5:0] di_low = si_raw[7:2];  // Di[5:0]  : byte1 의 bit[7:2]
    wire [ 7:0] di_high = si_raw[15:8];  // Di[13:6] : byte2 전체

    // ----------------------------------------------------------
    // Distance = Di[5:0] + Di[13:6] * 64
    //          = di_low  + di_high << 6
    //
    // 비트폭:
    //   di_high << 6 최대 = 255 << 6 = 16320 → 14비트
    //   di_low   최대     = 63        →  6비트
    //   합산 최대          = 16383     → 14비트 이내
    // ----------------------------------------------------------
    wire [13:0] dist_calc = {di_high, di_low};  // di_high di_low

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            distance   <= '0;
            is_flag    <= '0;
            calc_valid <= '0;
        end else begin
            calc_valid <= '0;

            if (si_valid) begin
                is_flag    <= si_raw[1:0];  // IS = byte1[1:0]
                distance   <= dist_calc;
                calc_valid <= 1'b1;
            end
        end
    end
endmodule
