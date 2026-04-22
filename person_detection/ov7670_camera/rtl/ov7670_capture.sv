// ====================================================
// ov7670_capture.sv: OV7670 카메라 픽셀 데이터 캡처
// ----------------------------------------------------
// 기능:
//     - OV7670에서 RGB565 데이터 수신 (2바이트 -> 1픽셀)
//     - BRAM에 순차 저장 (0-76799)
//     - VSYNC 하강 엣지에서 주소 리셋 (새 프레임 시작)
// 타이밍:
//     - 클럭: pclk (OV7670이 생성, ~6MHz)
//     - 데이터: data[7:0] (2바이트 = 1픽셀 RGB565)
// ====================================================

module ov7670_capture(
    // OV7670 신호
    input       pclk,
    input       href, vsync,
    input [7:0] data,

    // BRAM Port A (Write)
    output reg        wEn,
    output reg [16:0] wAddr,    // 0-76799
    output reg [15:0] wData     // RGB565
    );

    // =================== 내부 신호 ===================
    logic vsync_prev, vsync_falling;

    logic byte_toggle;    // 0: 상위 바이트, 1: 하위 바이트

    // ============= VSYNC 하강 엣지 검출 ==============
    always_ff @(posedge pclk) begin
        vsync_prev <= vsync;
    end

    assign vsync_falling = ~vsync & vsync_prev;

    // ======= RGB565 데이터 수신 (2바이트 조합) ========
    always_ff @(posedge pclk) begin
        if (vsync_falling) begin
            byte_toggle <= 1'b0;
            wEn <= 1'b0;
        end
        else if (href) begin
            // 첫 번째 바이트 (상위 8bit)
            if (!byte_toggle) begin
                wData[15:8] <= data;
                byte_toggle <= 1'b1;
                wEn <= 1'b0;
            end
            // 두 번째 바이트 (하위 8bit)
            else begin
                wData[7:0] <= data;
                byte_toggle <= 1'b0;
                wEn <= 1'b1;
            end
        end
        else begin
            wEn <= 1'b0;
        end
    end

    // =========== BRAM Write Address 생성 ============
    always_ff @(posedge pclk) begin
        if (vsync_falling) begin
            wAddr <= 0;
        end
        else if (wEn) begin
            if (wAddr < 76799) begin
                wAddr <= wAddr + 1;
            end
        end
    end

endmodule
