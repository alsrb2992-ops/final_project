// ========================================
// test_rgbPatGen.sv: RGB 테스트 패턴 생성기
// ----------------------------------------
// 기능:
//     - HDMI 출력 테스트용 단색 패턴 생성
//     - de 신호에 따라 RGB 출력 제어
// ========================================

module test_rgbPatGen(
    input clk, rstn,

    input de,

    output reg [7:0] red, grn, blue
    );

    always_comb begin
        if (de) begin
            red = 255; grn = 0; blue = 0;        // 빨강
//            red = 0; grn = 255; blue = 0;        // 초록
//            red = 0; grn = 0; blue = 255;        // 파랑
//            red = 255; grn = 255; blue = 255;    // 흰색
        end
        else begin
            red = 0; grn = 0; blue = 0;
        end
    end

endmodule
