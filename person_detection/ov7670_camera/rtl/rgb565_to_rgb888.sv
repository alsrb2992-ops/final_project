// ==========================================
// rgb565_to_rgb888.sv: RGB565 -> RGB888 변환
// ==========================================

module rgb565_to_rgb888(
    input  [15:0] rgb565,
    output [23:0] rgb888
    );

    // RGB565 비트 필드 추출
    wire [4:0] r5 = rgb565[15:11];    // Red (5bit)
    wire [5:0] g6 = rgb565[10:5];     // Green (6bit)
    wire [4:0] b5 = rgb565[4:0];      // Blue (5bit)

    // RGB888 비트 확장
    wire [7:0] r8 = {r5, r5[4:2]};    // 5bit -> 8bit
    wire [7:0] g8 = {g6, g6[5:4]};    // 6bit -> 8bit
    wire [7:0] b8 = {b5, b5[4:2]};    // 5bit -> 8bit

    // RGB888 출력
    assign rgb888 = {r8, g8, b8};

endmodule
