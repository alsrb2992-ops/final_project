// =============================================================================
// conv2x2.v  —  2×2 Convolution (Conv_out 전용, Q4.12)
// =============================================================================
// Conv_out: C_IN=32, C_OUT=1, kernel=2×2, pad=0, stride=1
// 입력 14×14×32 → 출력 13×13×1 (logit)
//
// 2×2 윈도우: 라인버퍼 1줄 + 현재행으로 2행 구성
// 각 위치에서 C_IN×4 = 128 곱셈을 시분할 처리
//
// ReLU 없이 raw 누산값(40bit signed) 출력 → sigmoid_lut에 연결
// =============================================================================

module conv2x2 #(
    parameter C_IN        = 32,
    parameter IMG_W_IN    = 14,       // DSC2 Pool 출력 가로
    parameter WEIGHT_FILE = "conv_out_w.hex"
) (
    input  wire clk,
    input  wire rst_n,

    input  wire                   in_valid,
    input  wire [C_IN*16-1:0]    pixel_in,

    output reg                    out_valid,
    output reg  signed [39:0]    acc_out_raw    // sigmoid 입력용 raw logit
);

    // ── 가중치 ROM: C_IN × 4 (2×2) ──────────────────────────────────
    // localparam WEIGHT_DEPTH = C_IN * 4;
    // reg signed [15:0] weight_rom [0:WEIGHT_DEPTH-1];

    // initial begin
    //     if (WEIGHT_FILE != "")
    //         $readmemh(WEIGHT_FILE, weight_rom);
    // end
    localparam WEIGHT_DEPTH = C_IN * 4;
    (* rom_style = "block" *) reg signed [15:0] weight_rom [0:WEIGHT_DEPTH-1];

    initial begin
        if (WEIGHT_FILE != "")
            $readmemh(WEIGHT_FILE, weight_rom);
    end

    // ── 편향 ROM ──────────────────────────────────────────────────────
    reg signed [15:0] bias;
    initial bias = 16'sd0;

    // ── 라인버퍼 (C_IN 각각, 깊이 IMG_W_IN) ─────────────────────────
    // 이전 행의 C_IN 채널 픽셀을 저장
    // reg signed [15:0] line_buf [0:C_IN-1][0:IMG_W_IN-1];
    // 이전 행의 C_IN 채널 픽셀을 저장. BRAM 사용 강제 속성 추가
    (* ram_style = "block" *) reg signed [15:0] line_buf [0:C_IN-1][0:IMG_W_IN-1];

    // ── 카운터 ────────────────────────────────────────────────────────
    reg [$clog2(IMG_W_IN)-1:0] col_cnt;
    reg [$clog2(IMG_W_IN)-1:0] row_cnt;

    // ── 2×2 윈도우 래치 ──────────────────────────────────────────────
    // 4개 위치 × C_IN 채널
    // p[0] = prev_row, prev_col  (좌상)
    // p[1] = prev_row, cur_col   (우상)
    // p[2] = cur_row,  prev_col  (좌하)
    // p[3] = cur_row,  cur_col   (우하)
    reg signed [15:0] prev_col_prev_row [0:C_IN-1]; // 좌상
    reg signed [15:0] prev_col_cur_row  [0:C_IN-1]; // 좌하

    // 현재 입력 래치
    reg [C_IN*16-1:0] pixel_latch;

    // prev_row의 현재 열 값 (라인버퍼에서 읽음)
    reg signed [15:0] lb_read [0:C_IN-1];
    

    // ── FSM ──────────────────────────────────────────────────────────
    localparam ST_IDLE    = 3'd0;
    localparam ST_LOAD    = 3'd1;  // 라인버퍼 읽기 + 래치
    localparam ST_COMPUTE = 3'd2;
    localparam ST_OUTPUT  = 3'd3;
    localparam ST_STORE   = 3'd4;  // 라인버퍼 쓰기

    reg [2:0] state;
    reg [$clog2(C_IN)-1:0]  ch_idx;
    reg [1:0]               k_idx;    // 커널 위치 0~3
    reg signed [39:0]       acc;
    reg                     win_valid; // 2×2 윈도우가 유효한지

    integer i, j;

    // 현재 커널 위치의 픽셀/가중치
    wire signed [15:0] cur_pixel =
        (k_idx == 2'd0) ? prev_col_prev_row[ch_idx] :  // 좌상
        (k_idx == 2'd1) ? lb_read[ch_idx] :             // 우상
        (k_idx == 2'd2) ? prev_col_cur_row[ch_idx] :    // 좌하
                          $signed(pixel_latch[ch_idx*16 +: 16]); // 우하

    wire signed [15:0] cur_weight = weight_rom[ch_idx * 4 + k_idx];

    wire signed [31:0] product     = cur_pixel * cur_weight;
    wire signed [39:0] prod_scaled = {{20{product[31]}}, product[31:12]};
    wire signed [39:0] acc_next    = acc + prod_scaled;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            col_cnt   <= 0;
            row_cnt   <= 0;
            ch_idx    <= 0;
            k_idx     <= 0;
            acc       <= 0;
            out_valid <= 1'b0;
            // acc_out_raw <= 0; // 비동기 리셋으로 인한 DRC 문제 발생 제거
            pixel_latch <= 0;
            win_valid <= 1'b0;
            for (i = 0; i < C_IN; i = i + 1) begin
                prev_col_prev_row[i] <= 0;
                prev_col_cur_row[i]  <= 0;
                lb_read[i]           <= 0;
                
                // XXXXX [삭제된 부분] BRAM 합성을 방해하는 배열 초기화 로직 제거
                // for (j = 0; j < IMG_W_IN; j = j + 1)
                //     line_buf[i][j] <= 0;
            end
        end
        else begin
            out_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (in_valid) begin
                        pixel_latch <= pixel_in;
                        state <= ST_LOAD;
                    end
                end

                ST_LOAD: begin
                    // 라인버퍼에서 이전 행의 현재 열 읽기
                    for (i = 0; i < C_IN; i = i + 1)
                        lb_read[i] <= line_buf[i][col_cnt];

                    // 윈도우 유효 판단: row >= 1 && col >= 1
                    win_valid <= (row_cnt > 0 && col_cnt > 0) ? 1'b1 : 1'b0;

                    if (row_cnt > 0 && col_cnt > 0) begin
                        state  <= ST_COMPUTE;
                        ch_idx <= 0;
                        k_idx  <= 0;
                        acc    <= {{24{bias[15]}}, bias};
                    end
                    else begin
                        state <= ST_STORE;
                    end
                end

                ST_COMPUTE: begin
                    acc <= acc_next;

                    if (k_idx == 2'd3) begin
                        k_idx <= 0;
                        if (ch_idx == C_IN - 1) begin
                            // 모든 채널 완료 → 출력
                            acc_out_raw <= acc_next;
                            state <= ST_OUTPUT;
                        end
                        else begin
                            ch_idx <= ch_idx + 1;
                        end
                    end
                    else begin
                        k_idx <= k_idx + 1;
                    end
                end

                ST_OUTPUT: begin
                    out_valid <= 1'b1;
                    state     <= ST_STORE;
                end

                ST_STORE: begin
                    // 이전 열 값 저장 (다음 위치의 "좌" 열로 사용)
                    for (i = 0; i < C_IN; i = i + 1) begin
                        prev_col_prev_row[i] <= line_buf[i][col_cnt];
                        prev_col_cur_row[i]  <= $signed(pixel_latch[i*16 +: 16]);
                    end

                    // 라인버퍼에 현재 행 저장
                    for (i = 0; i < C_IN; i = i + 1)
                        line_buf[i][col_cnt] <= $signed(pixel_latch[i*16 +: 16]);

                    // 카운터 업데이트
                    if (col_cnt == IMG_W_IN - 1) begin
                        col_cnt <= 0;
                        row_cnt <= row_cnt + 1;
                    end
                    else begin
                        col_cnt <= col_cnt + 1;
                    end

                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
