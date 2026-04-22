// ==========================================
// sccb_addrCnt.sv: SCCB ROM 주소 관리
// ------------------------------------------
// 기능:
//     - SCCB 전송 완료 시 다음 레지스터로 이동
// ==========================================

module sccb_addrCnt #(
    parameter NUM_REGS = 90
)(
    input clk, rstn,

    input txDone,

    output reg [6:0] regAddr,

    output reg cfgDone
    );

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            regAddr <= 0;
            cfgDone <= 0;
        end
        else if (txDone) begin
            if (regAddr < NUM_REGS - 1) regAddr <= regAddr + 1;
            else cfgDone <= 1'b1;
        end
        else if (cfgDone) cfgDone <= 1'b0;
    end

endmodule
