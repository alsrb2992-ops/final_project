// =============================================================================
// maxpool2x2.v  —  2×2 MaxPooling stride=2, floor모드 (Q4.12, 16bit)
// =============================================================================

module maxpool2x2 #(
    parameter DATA_WIDTH = 16,
    parameter IMG_WIDTH  = 126
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         in_valid,
    input  wire signed [DATA_WIDTH-1:0] pixel_in,

    output reg                          out_valid,
    output reg  signed [DATA_WIDTH-1:0] max_out
);

    // ── BRAM 추론용: 읽기/쓰기 분리 ────────────────────────────────
    (* ram_style = "block" *) reg signed [DATA_WIDTH-1:0] line_buf [0:IMG_WIDTH-1];

    reg [$clog2(IMG_WIDTH)-1:0] col_cnt;
    reg [$clog2(IMG_WIDTH)-1:0] row_cnt;

    reg signed [DATA_WIDTH-1:0] p00, p10;
    reg signed [DATA_WIDTH-1:0] rd_line_buf;  // BRAM 읽기 출력 레지스터

    function signed [DATA_WIDTH-1:0] max2;
        input signed [DATA_WIDTH-1:0] a, b;
        begin
            max2 = (a > b) ? a : b;
        end
    endfunction

    // ── BRAM 쓰기 포트 (for 루프 초기화 제거) ───────────────────────
    always @(posedge clk) begin
        if (in_valid)
            line_buf[col_cnt] <= pixel_in;
    end

    // ── BRAM 읽기 포트 ───────────────────────────────────────────────
    always @(posedge clk) begin
        if (in_valid)
            rd_line_buf <= line_buf[col_cnt];
    end

    // ── 제어 로직 ────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt   <= 0;
            row_cnt   <= 0;
            out_valid <= 1'b0;
            max_out   <= 0;
            p00       <= 0;
            p10       <= 0;
        end
        else if (in_valid) begin
            out_valid <= 1'b0;

            if (row_cnt[0]) begin
                if (!col_cnt[0]) begin
                    p10 <= pixel_in;
                    p00 <= rd_line_buf;  // BRAM 읽기 결과 사용
                end
                else begin
                    max_out   <= max2(max2(p00, rd_line_buf),
                                      max2(p10, pixel_in));
                    out_valid <= 1'b1;
                end
            end

            if (col_cnt == IMG_WIDTH - 1) begin
                col_cnt <= 0;
                row_cnt <= row_cnt + 1;
            end
            else begin
                col_cnt <= col_cnt + 1;
            end
        end
        else begin
            out_valid <= 1'b0;
        end
    end

endmodule