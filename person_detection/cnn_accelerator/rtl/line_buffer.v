// =============================================================================
// line_buffer.v  —  3×3 Conv용 라인 버퍼 (Q4.12, 16bit)
// =============================================================================
// 라인 버퍼 2개로 이전 2행을 저장, 현재 행과 함께 row0/1/2 출력
// out_valid: 첫 2줄이 채워진 후부터 유효
// 기존 읽기/쓰기 동일 always문에서 동작 -> 분리
// =============================================================================
module line_buffer #(
    parameter DATA_WIDTH = 16,
    parameter IMG_WIDTH  = 128
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  in_valid,
    input  wire [DATA_WIDTH-1:0] pixel_in,

    output wire [DATA_WIDTH-1:0] row0,
    output wire [DATA_WIDTH-1:0] row1,
    output wire [DATA_WIDTH-1:0] row2,
    output reg                   out_valid
);

    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] buf0 [0:IMG_WIDTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] buf1 [0:IMG_WIDTH-1];

    reg [$clog2(IMG_WIDTH)-1:0] col_cnt;
    reg [1:0] row_cnt;

    // 읽기 결과 레지스터 (BRAM 출력 레지스터)
    reg [DATA_WIDTH-1:0] rd_buf0, rd_buf1;

    // ── 쓰기 포트 (buf0 ← buf1, buf1 ← pixel_in) ──────────────────
    always @(posedge clk) begin
        if (in_valid) begin
            buf0[col_cnt] <= buf1[col_cnt];  // 쓰기만
            buf1[col_cnt] <= pixel_in;
        end
    end

    // ── 읽기 포트 (다음 클럭에 출력) ───────────────────────────────
    always @(posedge clk) begin
        if (in_valid) begin
            rd_buf0 <= buf0[col_cnt];  // 읽기만
            rd_buf1 <= buf1[col_cnt];
        end
    end

    // ── 제어 로직 ───────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt   <= 0;
            row_cnt   <= 0;
            out_valid <= 1'b0;
        end
        else if (in_valid) begin
            if (col_cnt == IMG_WIDTH - 1) begin
                col_cnt <= 0;
                if (row_cnt < 2'd2)
                    row_cnt <= row_cnt + 1;
            end
            else begin
                col_cnt <= col_cnt + 1;
            end

            out_valid <= (row_cnt == 2'd2) ? 1'b1 :
                         (row_cnt == 2'd1 && col_cnt == IMG_WIDTH - 1) ? 1'b1 :
                         1'b0;
        end
        else begin
            out_valid <= 1'b0;
        end
    end

    assign row0 = rd_buf0;
    assign row1 = rd_buf1;
    assign row2 = pixel_in;

endmodule