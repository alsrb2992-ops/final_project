// =======================================
// sccb_regRom.sv: OV7670 레지스터 설정 ROM
// ---------------------------------------
// 기능:
//     - OV7670 초기화 레지스터 값 저장
//     - 컬러바 테스트용 최소 설정
// 포맷:
//     - [15:8]: 레지스터 주소
//     - [7:0]:  레지스터 데이터
// =======================================

module sccb_regRom(
    input [6:0] romAddr,

    output [7:0] regAddr, regData
    );

    logic [15:0] rom [0:17];

    initial begin
        // Software Reset
        rom[0]  = 16'h1280;    // COM7: Reset all registers

        // 클럭 설정
        rom[1]  = 16'h1101;    // CLKRC: Use external clock directly

        // RGB565 출력 포맷
        rom[2]  = 16'h1204;    // COM7: Output format RGB
        rom[3]  = 16'h40D0;    // COM5: RGB565 output
        rom[4]  = 16'h8C00;    // RGB444: Disable (use RGB565)

        // QVGA 해상도 (320x240)
        rom[5]  = 16'h0C04;    // COM3: Enable scaling
        rom[6]  = 16'h3E19;    // COM14: Scaling parmeters
        rom[7]  = 16'h703A;    // SCALING_XSC
        rom[8]  = 16'h7135;    // SCALING_YSC
        rom[9]  = 16'h7211;    // SCALING_DCWTR
        rom[10] = 16'h73F1;    // SCALING_PCLK_DIV
        rom[11] = 16'hA202;    // SCALING_PCLK_DELAY

        // 프레임 타이밍
        rom[12] = 16'h1715;    // HSTART: Horizontal start
        rom[13] = 16'h1803;    // HSTOP: Horizontal stop
        rom[14] = 16'h3200;    // HREF: HREF control
        rom[15] = 16'h1903;    // VSTART: Vertical start
        rom[16] = 16'h1A7B;    // VSTOP: Vertical stop

        // 컬러바 활성화
        rom[17] = 16'h1242;    // COM7: Enalbe color bar test pattern
    end

    assign regAddr = rom[romAddr][15:8];
    assign regData = rom[romAddr][7:0];

endmodule
