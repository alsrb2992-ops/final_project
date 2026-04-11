// ===============================================================
// ov7670_capture: OV7670 Pixel Capture Module
// ---------------------------------------------------------------
// OV7670 카메라에서 픽셀 데이터 캡처 (PCLK 도메인)
//
// 동작:
//     1. VSYNC: 프레임 시작 감지 -> 주소 리셋
//     2. HREF: 유효 픽셀 라인 감지
//     3. PCLK: 8비트 데이터 2바이트씩 수신 -> RGB565 조립
//     4. 프레임 버퍼에 쓰기
// 타이밍:
//     - PCLK: 25MHz (카메라 출력 클럭)
//     - RGB565: 2바이트/픽셀 (MSB first)
//     - 해상도: QVGA (320x240 = 76,800 픽셀)
// 인터페이스:
//     - 입력: OV7670 신호 (PCLK, HREF, VSYNC, DATA[7:0])
//     - 출력: 프레임 버퍼 쓰기 (wEn, wAddr, wData)
//     - 제어: 프레임 완료 신호 (frame_done)
// ===============================================================

module ov7670_capture #(
    parameter FRAME_WIDTH = 320,
    parameter FRAME_HEIGHT = 240
)(
    input rstn,

    // OV7670 Camera Interface (PCLK 도메인)
    input       ov_pclk,                        // 카메라 픽셀 클럭 (~25MHz)
    input       ov_href, ov_vsync,              // 수평 참조 (유효 픽셀 라인) / 수직 동기 (프레임 시작, active HIGH)
    input [7:0] ov_data,                        // 픽셀 데이터 (8비트)

    // Frame Buffer Write Interface
    output reg        frame_wEn,       // 쓰기 활성화
    output reg [16:0] frame_wAddr,     // 쓰기 주소 (0-76799)
    output reg [15:0] frame_wData,     // RGB565 데이터

    // Status
    output reg frame_done, capturing    // 프레임 캡처 완료 (1 펄스) / 캡처 진행 중
    );

    // ====================== 로컬 파라미터 =======================
    localparam FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT;    // 76,800

    // ===================== 내부 신호 정의 =======================
    logic byte_toggle;                 // 바이트 카운터 (RGB565는 2바이트/픽셀). 0: 첫 번째 바이트(MSB), 1: 두 번째 바이트(LSB)
    logic [16:0] pxlAddr;              // 픽셀 주소 카운터 (0-76,799)
    logic [7:0] pxlData_msb;           // RGB565 데이터 조립용 레지스터. 첫 번째 바이트 저장

    logic vsync_prev, vsync_rising;    // VSYNC 엣지 감지용

    // ====================== VSYNC 엣지 감지 =====================
    always_ff @(posedge ov_pclk or negedge rstn) begin
        if (!rstn) vsync_prev <= 0;
        else vsync_prev <= ov_vsync;
    end

    assign vsync_rising = ov_vsync & ~vsync_prev;    // VSYNC 상승 엣지

    // =============== 메인 캡처 로직 (PCLK 도메인) ================
    always_ff @(posedge ov_pclk or negedge rstn) begin
        if (!rstn) begin
            byte_toggle <= 0;
            pxlAddr <= 0; pxlData_msb <= 0;
            frame_wEn <= 0; frame_wAddr <= 0; frame_wData <= 0;
            frame_done <= 0; capturing <= 0;
        end
        else begin
            // 1 클럭 펄스 신호
            frame_wEn <= 1'b0;
            frame_done <= 1'b0;

            // ---------- VSYNC 상승 엣지 -> 프레임 시작 -----------
            if (vsync_rising) begin
                byte_toggle <= 1'b0;
                pxlAddr <= '0; pxlData_msb <= '0;
                capturing <= 1'b1;
            end

            // ------- VSYNC가 LOW로 떨어지면 -> 프레임 종료 -------
            else if (vsync_prev && !ov_vsync) begin
                frame_done <= 1'b1; capturing <= 1'b0;    // 프레임 캡처 완료
            end

            // ------- HRER = HIGH -> 유효 픽셀 데이터 수신 --------
            else if (ov_href && capturing) begin
                if (!byte_toggle) begin
                    // 첫 번째 바이트 (MSB) 수신
                    pxlData_msb <= ov_data;
                    byte_toggle <= 1'b1;
                    frame_wEn <= 1'b0;         // 아직 쓰지 않음
                end
                else begin
                    // 두 번째 바이트 (LSB) 수신 -> RGB565 완성
                    frame_wAddr <= pxlAddr[16:0]; frame_wEn <= 1'b1;    // 프레임 버퍼에 쓰기
                    byte_toggle <= 1'b0;
                    pxlAddr <= pxlAddr + 1'b1;
                end
            end

            // ----- HREF = LOW -> 바이트 토글 리셋 (라인 끝) ------
            else if (!ov_href && byte_toggle) begin
                byte_toggle <= 1'b0;
            end
        end
    end

endmodule
