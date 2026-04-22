// ============================================================
// uart_tx.sv
// UART 송신 모듈
// 8N1, CLK_FREQ / BAUD_RATE
// ============================================================
module uart_tx_lidar #(
    parameter CLK_FREQ  = 125_000_000,
    parameter BAUD_RATE = 128_000
) (
    input wire clk,
    input wire rst_n,

    input  wire [7:0] data,
    input  wire       valid,  // 1클럭 펄스: 송신 요청
    output reg        ready,  // 1 = 송신 가능 상태
    output reg        tx
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    localparam IDLE = 2'b00;
    localparam START = 2'b01;
    localparam DATA = 2'b10;
    localparam STOP = 2'b11;

    reg [ 1:0] state;
    reg [15:0] clk_cnt;
    reg [ 2:0] bit_idx;
    reg [ 7:0] tx_shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            clk_cnt  <= 0;
            bit_idx  <= 0;
            tx_shift <= 0;
            tx       <= 1'b1;  // idle high
            ready    <= 1'b1;
        end else begin
            case (state)
                IDLE: begin
                    tx    <= 1'b1;
                    ready <= 1'b1;
                    if (valid) begin
                        tx_shift <= data;
                        state    <= START;
                        clk_cnt  <= 0;
                        ready    <= 1'b0;
                    end
                end

                START: begin
                    tx <= 1'b0;  // start bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                DATA: begin
                    tx <= tx_shift[bit_idx];
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        if (bit_idx == 3'd7) begin
                            state <= STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                STOP: begin
                    tx <= 1'b1;  // stop bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        state   <= IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
            endcase
        end
    end

endmodule
