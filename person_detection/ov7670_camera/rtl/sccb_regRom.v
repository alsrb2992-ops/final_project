// ===========================================================
// sccb_regRom.v: OV7670 레지스터 설정 ROM
// -----------------------------------------------------------
// 기능:
//     - OV7670 초기화 레지스터 값 저장
// 포맷:
//     - [15:8]: 레지스터 주소
//     - [7:0]:  레지스터 데이터
// ============================================================

module sccb_regRom #(
    parameter NUM_REGS = 90
)(
    input [6:0] romAddr,

    output [7:0] regAddr, regData
    );

    reg [15:0] rom [0:NUM_REGS-1];

    assign regAddr = rom[romAddr][15:8];
    assign regData = rom[romAddr][7:0];

    initial begin
        // Software Reset
        rom[0]  = 16'h1280;    // COM7: 모든 레지스터 리셋

        // Basic Settings
        rom[1]  = 16'h3A04;    // TSLB: UV 순서 및 출력 시퀀스 설정
        rom[2]  = 16'h1200;    // COM7: 기본 설정
        rom[3]  = 16'h13E7;    // COM8: AGC, AEC, AWB 자동 제어 활성화
        rom[4]  = 16'h6F9F;    // AWBCTR0: 화이트 밸런스 제어
        rom[5]  = 16'hB084;    // Reserved: 색상 품질 향상
        rom[6]  = 16'h703A;    // SCALING_XSC
        rom[7]  = 16'h7135;    // SCALING_YSC
        rom[8]  = 16'h7211;    // SCALING_DCWTR
        rom[9]  = 16'h73F0;    // SCALING_PCLK_DIV

        // Gamma Curve (명암 대비 곡선)
        rom[10] = 16'h7A20;    // SLOP
        rom[11] = 16'h7B10;
        rom[12] = 16'h7C1E;
        rom[13] = 16'h7D35;
        rom[14] = 16'h7E5A;
        rom[15] = 16'h7F69;
        rom[16] = 16'h8076;
        rom[17] = 16'h8180;
        rom[18] = 16'h8288;
        rom[19] = 16'h838F;
        rom[20] = 16'h8496;
        rom[21] = 16'h85A3;
        rom[22] = 16'h86AF;
        rom[23] = 16'h87C4;
        rom[24] = 16'h88D7;
        rom[25] = 16'h89E8;

        // AGC / AEC / Exposure Control
        rom[26] = 16'h0000;    // GAIN
        rom[27] = 16'h1000;    // AECH
        rom[28] = 16'h0D40;    // COM4
        rom[29] = 16'h1418;    // COM9
        rom[30] = 16'hA505;    // BD50MAX
        rom[31] = 16'hAB07;    // BD50MAX
        rom[32] = 16'h2495;    // AEW
        rom[33] = 16'h2533;    // AEB
        rom[34] = 16'h26E3;    // VPT
        rom[35] = 16'h9F78;
        rom[36] = 16'hA068;
        rom[37] = 16'hA103;    // Reserved: Magic value
        rom[38] = 16'hA6D8;
        rom[39] = 16'hA7D8;
        rom[40] = 16'hA8F0;
        rom[41] = 16'hA990;
        rom[42] = 16'hAA94;

        // QVGA Resolution (320x240)
        rom[43] = 16'h1211;    // COM7
        rom[44] = 16'h0C04;    // COM3
        rom[45] = 16'h3E19;    // COM14
        rom[46] = 16'h703A;
        rom[47] = 16'h7135;
        rom[48] = 16'h7211;
        rom[49] = 16'h73F1;
        rom[50] = 16'hA202;

        // Frame Control
        rom[51] = 16'h1715;    // HSTART
        rom[52] = 16'h1803;    // HSTOP
        rom[53] = 16'h3200;    // HREF
        rom[54] = 16'h1903;    // VSTART
        rom[55] = 16'h1A7B;    // VSTOP
        rom[56] = 16'h0300;    // VREF

        // RGB565 Output Format
        rom[57] = 16'h1214;    // COM7: RGB 출력 모드
        rom[58] = 16'h40D0;    // COM15: RGB565 + Full Range
        rom[59] = 16'h8C00;    // RGB444 비활성화 (RGB565 사용)

        // 밝기/대비/선명도 조정
        rom[60] = 16'h4200;    // COM17: DSP 색상 바 비활성화
        rom[61] = 16'h13E7;    // COM8: AGC/AEC/AWB 활성화
        rom[62] = 16'hAA14;    // HAECC7
        rom[63] = 16'h5520;    // Brightness
        rom[64] = 16'h1418;    // COM9
        rom[65] = 16'h3F08;    // Edge Enhancement
        rom[66] = 16'h5660;    // Contrast

        // Color Matrix (RGB 색상 보정)
        rom[67] = 16'h4FB3;
        rom[68] = 16'h50B3;
        rom[69] = 16'h5100;
        rom[70] = 16'h523D;
        rom[71] = 16'h53B0;
        rom[72] = 16'h54E4;
        rom[73] = 16'h589E;

        // Clock / Misc
        rom[74] = 16'h1101;    // CLKRC
        rom[75] = 16'h6B4A;    // DBLV
        rom[76] = 16'h1E07;    // MVFP

        // 노이즈 제거 및 색상 보정
        rom[77] = 16'h4108;    // COM16: Edge Enhancement 활성화
        rom[78] = 16'h3D00;    // COM13: UV 채도 비활성화
        rom[79] = 16'h1500;    // COM10: VSYNC/HREF 안정화
        rom[80] = 16'hB100;    // ABLC1
        rom[81] = 16'hB200;    // ABLC 오프셋
        rom[82] = 16'hB300;    // ABLC 타겟
        rom[83] = 16'h13E7;    // COM8: 자동 제어 유지
        rom[84] = 16'h0230;    // Red Gain
        rom[85] = 16'h0150;    // Blue Gain
        rom[86] = 16'h76E1;    // 노이즈 필터 활성화
        rom[87] = 16'h4F40;
        rom[88] = 16'h5034;
        rom[89] = 16'h5100;
    end

endmodule
