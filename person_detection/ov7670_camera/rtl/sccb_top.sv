// =========================================
// sccb_top.sv: SCCB 통합 모듈
// -----------------------------------------
// 기능:
//     - OV7670 카메라 초기화
//     - SCCB 프로토콜 자동 전송
// 구성:
//     - 초기화 타이머 (100ms 대기)
//     - ROM 주소 카운터
//     - 레지스터 ROM
//     - SCCB 컨트롤러
// =========================================

module sccb_top(
    input clk, rstn,

    output scl, sda,

    output cfgDone
    );

    // ============= 내부 신호 ==============
    wire ready, txDone;

    wire [6:0] romAddr;
    wire [7:0] regAddr, regData;

    // =========== 초기화 타이머 ============
    sccb_initTiming u_initTim (.clk(clk), .rstn(rstn), .ready(ready));

    // ========== ROM 주소 카운터 ===========
    sccb_addrCnt u_addrCnt (.clk(clk), .rstn(rstn), .txDone(txDone), .regAddr(romAddr), .cfgDone(cfgDone));

    // ============ 레지스터 ROM ============
    sccb_regRom u_regRom (.romAddr(romAddr), .regAddr(regAddr), .regData(regData));

    // =========== SCCB 컨트롤러 ============
    sccb_ctrl u_ctrl (
        .clk(clk), .rstn(rstn),
        .start(ready & !cfgDone),
        .regAddr(regAddr), .regData(regData),
        .scl(scl), .sda(sda),
        .txDone(txDone));

endmodule
