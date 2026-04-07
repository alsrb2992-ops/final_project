// ============================================================
// brake_output.sv
// 제동 신호 GPIO 출력 모듈
// brake_signal 이 1이 되면 HOLD_CYCLES 동안 유지
// (모터 드라이버 반응 시간 고려)
// ============================================================
module brake_output #(
    parameter CLK_FREQ = 125_000_000,
    parameter HOLD_MS  = 32'd200       // 제동 유지 시간 (ms)
) (
    input logic clk,
    input logic rst_n,

    input logic brake_signal,
    input logic warning_signal,

    output logic brake_gpio,  // 모터 드라이버로
    output logic warning_led  // 경고 LED
);

    localparam HOLD_CYCLES = CLK_FREQ / 1000 * HOLD_MS;  // 10,000,000

    logic [31:0] hold_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brake_gpio  <= '0;
            warning_led <= '0;
            hold_cnt    <= '0;
        end else begin
            warning_led <= warning_signal;

            if (brake_signal) begin
                // 제동 신호 들어오면 카운터 리셋하고 유지
                brake_gpio <= 1'b1;
                hold_cnt   <= HOLD_CYCLES;
            end else if (hold_cnt > 0) begin
                // 홀드 타임 동안 유지
                hold_cnt   <= hold_cnt - 1;
                brake_gpio <= 1'b1;
            end else begin
                brake_gpio <= 1'b0;
            end
        end
    end

endmodule
