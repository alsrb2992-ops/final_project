// =============================================================================
// top.v  —  최상위 래퍼 모듈
// =============================================================================
// 구조:
//   OV7670 카메라 인터페이스 (ov7670_cam.v)
//     → CNN 가속기 (cnn_top.v)
//       → AXI4-Lite PS 연결 (향후 추가)
//
// OV7670 핀:
//   pclk     — 픽셀 클럭 (카메라 출력)
//   href     — 수평 동기
//   vsync    — 수직 동기
//   d[7:0]   — 8bit 병렬 데이터
//
// 내부 연결:
//   ov7670_cam → pixel_r/g/b [15:0] Q4.12, pixel_valid
//   cnn_top    → grid_prob[7:0], grid_valid, inference_done
// =============================================================================

module top (
    input  wire        clk,        // 시스템 클럭 (50MHz, FPGA)
    input  wire        rst_n,      // 전역 리셋 (active low)

    // ── OV7670 카메라 인터페이스 ─────────────────────────────────────
    input  wire        cam_pclk,   // 카메라 픽셀 클럭
    input  wire        cam_href,   // 수평 동기
    input  wire        cam_vsync,  // 수직 동기
    input  wire [7:0]  cam_d,      // 8bit 병렬 데이터

    // ── CNN 출력 ─────────────────────────────────────────────────────
    output wire        grid_valid,      // 격자 확률맵 유효 신호
    output wire [7:0]  grid_prob,       // 격자 확률값 (0~255)
    output wire        inference_done   // 추론 완료 신호
);

    // ── 카메라 → CNN 연결 신호 ────────────────────────────────────────
    wire        pixel_valid;
    wire [15:0] pixel_r;
    wire [15:0] pixel_g;
    wire [15:0] pixel_b;

    // =====================================================================
    // OV7670 카메라 인터페이스
    // RGB444 수신 → Q4.12 16bit 변환
    // =====================================================================
    ov7670_cam u_cam (
        .clk        (clk),
        .rst_n      (rst_n),
        .pclk       (cam_pclk),
        .href       (cam_href),
        .vsync      (cam_vsync),
        .d          (cam_d),
        .pixel_valid(pixel_valid),
        .pixel_r    (pixel_r),
        .pixel_g    (pixel_g),
        .pixel_b    (pixel_b)
    );

    // =====================================================================
    // CNN 가속기
    // =====================================================================
    cnn_top u_cnn (
        .clk           (clk),
        .rst_n         (rst_n),
        .pixel_valid   (pixel_valid),
        .pixel_r       (pixel_r),
        .pixel_g       (pixel_g),
        .pixel_b       (pixel_b),
        .grid_valid    (grid_valid),
        .grid_prob     (grid_prob),
        .inference_done(inference_done)
    );

endmodule