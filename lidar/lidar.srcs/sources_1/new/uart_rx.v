// ============================================================
// uart_rx.sv
// UART 수신 모듈
// 115200 baud, 8N1
// ============================================================
module uart_rx_lidar #(
    parameter CLK_FREQ  = 125_000_000,
    parameter BAUD_RATE = 128_000
) (
    input wire clk,
    input wire rst_n,
    input wire rx,

    output reg [7:0] data,
    output reg       valid  // 1클럭 펄스: 수신 완료
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;  // 434

    // 상태 정의
    localparam IDLE = 2'b00;
    localparam START = 2'b01;
    localparam DATA = 2'b10;
    localparam STOP = 2'b11;

    reg [ 1:0] state;
    reg [15:0] clk_cnt;
    reg [ 2:0] bit_idx;
    reg [ 7:0] rx_shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            clk_cnt <= 0;
            bit_idx <= 0;
            rx_shift<= 0;
            data    <= 0;
            valid   <= 0;
        end else begin
            valid <= 0;  // 기본 0, 완료시 1클럭만 1

            case (state)
                // --------------------------------------------------
                IDLE: begin
                    if (rx == 1'b0) begin  // start bit 감지
                        state   <= START;
                        clk_cnt <= 0;
                    end
                end

                // --------------------------------------------------
                // start bit 중간까지 대기 (샘플링 오차 최소화)
                START: begin
                    if (clk_cnt == (CLKS_PER_BIT / 2) - 1) begin
                        if (rx == 1'b0) begin  // 여전히 0이면 유효
                            state   <= DATA;
                            clk_cnt <= 0;
                            bit_idx <= 0;
                        end else begin
                            state <= IDLE;  // 글리치 → 무시
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // --------------------------------------------------
                // 데이터 비트 8개 수신 (LSB first)
                DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt           <= 0;
                        rx_shift[bit_idx] <= rx;
                        if (bit_idx == 3'd7) begin
                            state   <= STOP;
                            bit_idx <= 0;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // --------------------------------------------------
                // stop bit 확인
                STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        state   <= IDLE;
                        if (rx == 1'b1) begin  // stop bit = 1 이면 정상
                            data  <= rx_shift;
                            valid <= 1'b1;
                        end
                        // stop bit 오류시 데이터 버림
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
            endcase
        end
    end

endmodule
