// ============================================
// sccb_initTiming.sv: SCCB 초기화 타이밍 생성기
// --------------------------------------------
// 기능:
//     - 전원 인가 후 100ms 대기
//     - OV7670 안정화 시간 확보
// ============================================

module sccb_initTiming(
    input clk, rstn,

    output reg ready
    );

    localparam WAIT = 12_500_000;    // 125MHz x 100ms

    logic [$clog2(WAIT)-1:0] cnt;

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            cnt <= 0;
            ready <= 0;
        end
        else begin
            if (cnt == WAIT - 1) begin
                ready <= 1'b1;
            end
            else begin
                cnt <= cnt + 1;
                ready <= 1'b0;
            end
        end
    end

endmodule
