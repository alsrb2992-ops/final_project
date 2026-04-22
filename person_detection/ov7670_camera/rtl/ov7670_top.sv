// ===================================================
// ov7670_top.sv: OV7670 to HDMI 최상위 모듈
// ---------------------------------------------------
// 데이터 흐름:
//     OV7670 카메라 -> ov7670_capture -> BRAM -> HDMI
// 클럭:
//     - 시스템 클럭: 125MHz (입력)
//     - 픽셀 클럭: 25MHz (HDMI용)
//     - 카메라 클럭: 24MHz (OV7670 XCLK)
//     - TMDS 직렬 클럭: 125MHz (시스템 클럭 재사용)
// ===================================================

module ov7670_top(
    input clk, rstn,

    // SCCB 통신
    output scl,
    inout  sda,

    // OV7670 카메라
    output       xclk,
    input        pclk,
    input        href, vsync,
    input  [7:0] data,

    // HDMI 출력
    output       hdmi_tx_clk_p, hdmi_tx_clk_n,
    output [2:0] hdmi_tx_p, hdmi_tx_n,

    output [3:0] led
    );

    // =============== 내부 신호 정의 =================
    wire clk_pxl_25m, clk_cam_24m;
    wire locked_pxl;

    wire scl_out, sda_out;
    wire sccb_cfgDone;

    wire        bram_wEn;
    wire [16:0] bram_wAddr, bram_rAddr;
    wire [15:0] bram_wData, bram_rData;

    wire       hdmi_hsync, hdmi_vsync;
    wire       de;
    wire [9:0] pxl_x, pxl_y;

    wire [7:0] red, grn, blue;

    // ================== 클럭 생성 ===================
    hdmi_clk_gen u_hdmi_clk_gen (.clk_in1(clk), .clk_out1(clk_pxl_25m), .clk_out2(clk_cam_24m), .reset(!rstn), .locked(locked_pxl));

    assign xclk = clk_cam_24m;

    // ================== SCCB 통신 ==================
    sccb_top u_sccb (
        .clk(clk), .rstn(rstn & locked_pxl),
        .scl(scl_out), .sda(sda_out),
        .cfgDone(sccb_cfgDone));

    // SCCB 핀 연결 (Open-drain)
    assign scl = scl_out;
    assign sda = sda_out;

    // ================= 카메라 캡처 ==================
    ov7670_capture u_capture (
        .pclk(pclk), .href(href), .vsync(vsync), .data(data),
        .wEn(bram_wEn), .wAddr(bram_wAddr), .wData(bram_wData));

    // ============== BRAM Frame Buffer ==============
    frameBuff_bram u_fb (
        .clka(pclk), .wea(bram_wEn), .addra(bram_wAddr), .dina(bram_wData), .douta(),                // Port A (Write) - OV7670
        .clkb(clk_pxl_25m), .web(1'b0), .addrb(bram_rAddr), .dinb(16'h0000), .doutb(bram_rData));    // Port B (Read) - HDMI

    // ================== 디버그 ILA ==================
//    ila_capture u_ila (
//        .clk(clk),
//        .probe0(pclk), .probe1(vsync), .probe2(href), .probe3(bram_wEn), .probe4(bram_wAddr), .probe5(bram_wData), .probe6(data));

    // ================= 타이밍 생성 ==================
    hdmi_timingGen u_timingGen (
        .clk(clk_pxl_25m), .rstn(rstn & locked_pxl),
        .hsync(hdmi_hsync), .vsync(hdmi_vsync), .de(de), .pxl_x(pxl_x), .pxl_y(pxl_y));

    // =============== HDMI 디스플레이 ================
    hdmi_display u_disp (
        .clk(clk_pxl_25m), .rstn(rstn & locked_pxl),
        .de(de), .pxl_x(pxl_x), .pxl_y(pxl_y),
        .rAddr(bram_rAddr), .rData(bram_rData),
        .red(red), .grn(grn), .blue(blue));

    // ================= HDMI 출력 ===================
    hdmi_rgb2dvi u_rgb2dvi (
        .TMDS_Clk_p(hdmi_tx_clk_p), .TMDS_Clk_n(hdmi_tx_clk_n), .TMDS_Data_p(hdmi_tx_p), .TMDS_Data_n(hdmi_tx_n),
        .aRst(!rstn), .PixelClk(clk_pxl_25m), .SerialClk(clk),
        .vid_pData({red, grn, blue}), .vid_pVDE(de), .vid_pHSync(hdmi_hsync), .vid_pVSync(hdmi_vsync));

    // ================== 디버그 LED ==================
    localparam STRETCH = 12_500_000;
    logic [$clog2(STRETCH)-1:0] cnt;

    logic sccb_cfgDone_stretch;

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            cnt <= 0;
            sccb_cfgDone_stretch <= 0;
        end
        else if (sccb_cfgDone) begin
            cnt <= STRETCH - 1;
            sccb_cfgDone_stretch <= 1'b1;
        end
        else if (cnt > 0) begin
            cnt <= cnt - 1;
            sccb_cfgDone_stretch <= 1'b1;
        end
        else begin
            sccb_cfgDone_stretch <= 1'b0;
        end
    end

    assign led[0] = locked_pxl;
    assign led[1] = de;
    assign led[2] = sccb_cfgDone_stretch;
    assign led[3] = vsync;

endmodule
