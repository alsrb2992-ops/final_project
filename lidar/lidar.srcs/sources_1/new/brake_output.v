// ============================================================
// brake_output.sv
// 제동 신호 GPIO 출력 모듈
// brake_signal 이 1이 되면 HOLD_CYCLES 동안 유지
// (모터 드라이버 반응 시간 고려)
// ============================================================

`include "lidar_define.vh"

module brake_output #(
    parameter CLK_FREQ     = 125_000_000,
    parameter HOLD_MS      = 32'd200,      // 제동 유지 시간 (ms)
    parameter SIDE_HOLD_MS = 32'd100       // 측면 경고 유지 시간 (ms)
) (
    input wire clk,
    input wire rst_n,

    input wire       round_done,
    input wire       brake_signal,
    input wire       warning_signal,
    input wire       side_warning_signal,
    input wire [3:0] direction_degree,

    output reg       brake_gpio,                // 모터 드라이버로
    output reg       warning_led,               // 경고 LED
    output reg       side_warning_signal_gpio,  // 측면 경고 GPIO
    output reg [2:0] direction_degree_gpio      // 방향 정보 GPIO
);

    localparam HOLD_CYCLES = CLK_FREQ / 1000 * HOLD_MS;  // 10,000,000
    localparam SIDE_HOLD_CYCLES = CLK_FREQ / 1000 * SIDE_HOLD_MS;  // 5,000,000


    reg [$clog2(HOLD_CYCLES) -1:0] hold_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brake_gpio <= 0;
            hold_cnt   <= 0;
        end else begin

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

    reg [$clog2(HOLD_CYCLES) -1 : 0] hold_cnt_1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            warning_led <= 0;
            hold_cnt_1  <= 0;
        end else begin

            if (warning_signal) begin
                warning_led <= 1'b1;
                hold_cnt_1  <= HOLD_CYCLES;
            end else if (hold_cnt_1 > 0) begin
                hold_cnt_1  <= hold_cnt_1 - 1;
                warning_led <= 1'b1;
            end else begin
                warning_led <= 1'b0;
            end
        end
    end

    reg [$clog2(SIDE_HOLD_CYCLES) -1 : 0] hold_cnt_2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hold_cnt_2 <= 0;
        end else begin

            if (side_warning_signal || warning_signal) begin
                side_warning_signal_gpio <= 1'b1;
                hold_cnt_2 <= SIDE_HOLD_CYCLES;
            end else if (hold_cnt_2 > 0) begin
                hold_cnt_2               <= hold_cnt_2 - 1;
                side_warning_signal_gpio <= 1'b1;
            end else begin
                side_warning_signal_gpio <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            direction_degree_gpio <= `CENTER;
        end else begin
            if (round_done) begin
                direction_degree_gpio <= direction_degree;
            end
        end
    end

endmodule
