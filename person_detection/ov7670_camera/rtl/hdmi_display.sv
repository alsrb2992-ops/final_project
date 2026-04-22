// ========================================================
// hdmi_display.sv: HDMI 디스플레이 제어 모듈
// --------------------------------------------------------
// 기능:
//     - BRAM에서 카메라 프레임 데이터 읽기
//     - 320x240 -> 640x480 (2배 업스케일, Nearest Neighbor)
//     - RGB565 -> RGB888 변환 및 출력
// 화면 배치:
//     - 카메라: 320x240
//     - 출력: 640x480 (각 픽셀을 2x2로 확대)
// ========================================================

module hdmi_display(
    input clk, rstn,

    // VGA 타이밍 입력
    input       de,
    input [9:0] pxl_x, pxl_y,

    // BRAM Port B (Read)
    output reg [16:0] rAddr,
    input      [15:0] rData,

    // HDMI RGB 출력
    output reg [7:0] red, grn, blue
    );

    // =================== 파라미터 정의 ===================
    // 카메라 영상 크기
    localparam CAM_WIDTH  = 320;
    localparam CAM_HEIGHT = 240;

    // ====================== 내부 신호 ====================
    // 2배 축소 좌표
    logic [8:0] cam_x;    // 0-319 (640/2)
    logic [7:0] cam_y;    // 0-239 (480/2)

    // RGB565 -> RGB888 변환
    logic [23:0] rgb888;

    // 1 클럭 앞선 좌표 (다음 픽셀)
    logic [9:0] pxl_x_next, pxl_y_next;
    logic       de_delayed;

    // =================== 좌표 파이프라인 =================
    always_ff @(posedge clk) begin
        pxl_x_next <= pxl_x + 1;
        pxl_y_next <= pxl_y;
        de_delayed <= de;
    end

    assign cam_x = pxl_x_next[9:1];    // pxl_x_next / 2
    assign cam_y = pxl_y_next[8:1];    // pxl_y_next / 2

    // =============== BRAM Read Address 생성 =============
    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) rAddr <= 0;
        else if (de) rAddr <= (cam_y * CAM_WIDTH) + cam_x;
        else rAddr <= 0;
    end

    // ================ RGB565 -> RGB888 변환 =============
    rgb565_to_rgb888 u_rgbConv (.rgb565(rData), .rgb888(rgb888));

    // ====================== RGB 출력 ====================
    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            red <= 0; grn <= 0; blue <= 0;
        end
        else begin
            if (de_delayed) begin
                red <= rgb888[23:16];
                grn <= rgb888[7:0];
                blue <= rgb888[15:8];
            end
            else {red, grn, blue} <= 24'h000000;
        end
    end

endmodule
