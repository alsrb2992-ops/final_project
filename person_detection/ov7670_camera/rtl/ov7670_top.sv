// ===============================================
// ov7670_top.sv: OV7670 to HDMI 최상위 모듈
// -----------------------------------------------
// 클럭:
//     - 시스템 클럭: 125MHz (입력)
//     - 픽셀 클럭: 25MHz (Clocking Wizard 출력)
//     - TMDS 직렬 클럭: 125MHz (시스템 클럭 재사용)
// ===============================================

module ov7670_top(
    input clk, rstn,

    output       hdmi_tx_clk_p, hdmi_tx_clk_n,
    output [2:0] hdmi_tx_p, hdmi_tx_n,

    output [3:0] led
    );

    // ============= 내부 신호 정의 ===============
    wire clk_pxl_25m;
    wire locked_pxl;

    wire       hsync, vsync;
    wire       de;
    wire [9:0] pxl_x, pxl_y;

    wire [7:0] red, grn, blue;

    // ================ 클럭 생성 =================
    hdmi_clk_gen u_hdmi_clk_gen (.clk_in1(clk), .clk_out1(clk_pxl_25m), .reset(!rstn), .locked(locked_pxl));

    // =============== 타이밍 생성 ================
    vga_timingGen u_vga_timingGen (
        .clk(clk_pxl_25m), .rstn(rstn & locked_pxl),
        .hsync(hsync), .vsync(vsync), .de(de), .pxl_x(pxl_x), .pxl_y(pxl_y));

    // ============== RGB 패턴 생성 ===============
    test_rgbPatGen u_rgbPatGen (
        .clk(clk_pxl_25m), .rstn(rstn & locked_pxl),
        .de(de),
        .red(red), .grn(grn), .blue(blue));

    // =============== HDMI 출력 =================
    hdmi_rgb2dvi u_rgb2dvi (
        .TMDS_Clk_p(hdmi_tx_clk_p), .TMDS_Clk_n(hdmi_tx_clk_n), .TMDS_Data_p(hdmi_tx_p), .TMDS_Data_n(hdmi_tx_n),
        .aRst(!rstn), .PixelClk(clk_pxl_25m), .SerialClk(clk),
        .vid_pData({red, grn, blue}), .vid_pVDE(de), .vid_pHSync(hsync), .vid_pVSync(vsync));

    assign led[0] = locked_pxl;
    assign led[1] = de;
    assign led[2] = hsync;
    assign led[3] = vsync;

endmodule
