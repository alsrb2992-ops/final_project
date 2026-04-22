// =============================================================================
// conv_dw.v  —  Depthwise Convolution 3×3 (Q4.12)
// =============================================================================
// 채널별 독립 3×3 conv
// DSC1 DW: C_IN=8,  63×63 → 61×61
// DSC2 DW: C_IN=16, 30×30 → 28×28
//
// [수정사항]
//   커널 9개 MAC 병렬화 + 2단 파이프라인 (conv3x3 방식 동일)
//     Stage1: ch_idx 채널의 9개 곱셈 동시 수행 → prod_reg 래치
//     Stage2: 덧셈트리(9개 합산) → acc 누산 + ReLU → out_buf
//   kidx 루프 제거, ch_idx 루프만 유지
//
//   1픽셀당 사이클:
//     (ST_STAGE1 + ST_STAGE2) × C_IN + ST_OUTPUT
//     = 2 × C_IN + 1
//     DSC1 DW (C_IN=8):  17 cycle  (기존 74  → 4.4배 향상)
//     DSC2 DW (C_IN=16): 33 cycle  (기존 146 → 4.4배 향상)
// =============================================================================

module conv_dw #(
    parameter C_IN        = 8,
    parameter IMG_W_IN    = 63,
    parameter WEIGHT_FILE = ""
) (
    input  wire clk,
    input  wire rst_n,

    input  wire                   in_valid,
    input  wire [C_IN*16-1:0]    pixel_in,

    output reg                    out_valid,
    output reg  [C_IN*16-1:0]    feature_out
);

    // ── 가중치 ROM ────────────────────────────────────────────────────
    localparam WEIGHT_DEPTH = C_IN * 9;
    (* rom_style = "block" *) reg signed [15:0] weight_rom [0:WEIGHT_DEPTH-1];

    initial begin
        if (WEIGHT_FILE != "")
            $readmemh(WEIGHT_FILE, weight_rom);
    end

    // ── 편향 ROM ──────────────────────────────────────────────────────
    reg signed [15:0] bias_rom [0:C_IN-1];
    integer bi;
    initial begin
        for (bi = 0; bi < C_IN; bi = bi + 1)
            bias_rom[bi] = 16'sd0;
    end

    // ── 라인버퍼 + 윈도우 (C_IN 각각, 병렬) ──────────────────────────
    wire [15:0]     lb_row0 [0:C_IN-1];
    wire [15:0]     lb_row1 [0:C_IN-1];
    wire [15:0]     lb_row2 [0:C_IN-1];
    wire [C_IN-1:0] lb_valid;

    wire [16*9-1:0] win_flat  [0:C_IN-1];
    wire [C_IN-1:0] win_valid;

    genvar ch;
    generate
        for (ch = 0; ch < C_IN; ch = ch + 1) begin : LB_DW
            line_buffer #(
                .DATA_WIDTH (16),
                .IMG_WIDTH  (IMG_W_IN)
            ) u_lb (
                .clk      (clk),
                .rst_n    (rst_n),
                .in_valid (in_valid),
                .pixel_in (pixel_in[ch*16 +: 16]),
                .row0     (lb_row0[ch]),
                .row1     (lb_row1[ch]),
                .row2     (lb_row2[ch]),
                .out_valid(lb_valid[ch])
            );
        end
    endgenerate

    generate
        for (ch = 0; ch < C_IN; ch = ch + 1) begin : WIN_DW
            window_3x3 #(.DATA_WIDTH(16)) u_win (
                .clk        (clk),
                .rst_n      (rst_n),
                .in_valid   (lb_valid[ch]),
                .row0       (lb_row0[ch]),
                .row1       (lb_row1[ch]),
                .row2       (lb_row2[ch]),
                .window_flat(win_flat[ch]),
                .out_valid  (win_valid[ch])
            );
        end
    endgenerate

    wire all_win_valid = &win_valid;

    // ── 윈도우 래치 (ST_IDLE → ST_STAGE1 전환 시 캡처) ───────────────
    reg [16*9-1:0] win_latch [0:C_IN-1];

    // ── FSM 상태 정의 ─────────────────────────────────────────────────
    localparam ST_IDLE   = 2'd0;
    localparam ST_STAGE1 = 2'd1;  // 9개 곱셈 (조합) → prod_reg 래치
    localparam ST_STAGE2 = 2'd2;  // 덧셈트리 → acc 누산 + ReLU
    localparam ST_OUTPUT = 2'd3;

    reg [1:0]                 state;
    reg [$clog2(C_IN)-1:0]   ch_idx;
    reg signed [39:0]         acc;
    reg signed [15:0]         out_buf [0:C_IN-1];

    // ── Stage1: 현재 ch_idx 채널의 9개 MAC 병렬 (조합 논리) ──────────
    wire signed [31:0] prod_comb [0:8];
    wire signed [39:0] prod_scaled_comb [0:8];

    genvar ki;
    generate
        for (ki = 0; ki < 9; ki = ki + 1) begin : KMAC
            (* use_dsp = "yes" *)
            assign prod_comb[ki] = $signed(win_latch[ch_idx][ki*16 +: 16])
                                 * weight_rom[ch_idx * 9 + ki];
            assign prod_scaled_comb[ki] = {{20{prod_comb[ki][31]}},
                                            prod_comb[ki][31:12]};
        end
    endgenerate

    // ── Stage1 → Stage2 파이프라인 레지스터 ──────────────────────────
    reg signed [39:0] prod_reg [0:8];

    integer ri;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (ri = 0; ri < 9; ri = ri + 1)
                prod_reg[ri] <= 40'sd0;
        end
        else if (state == ST_STAGE1) begin
            for (ri = 0; ri < 9; ri = ri + 1)
                prod_reg[ri] <= prod_scaled_comb[ri];
        end
    end

    // ── Stage2: 덧셈트리 (9개 고정, generate 불필요) ─────────────────
    wire signed [39:0] kernel_sum =
        prod_reg[0] + prod_reg[1] + prod_reg[2] +
        prod_reg[3] + prod_reg[4] + prod_reg[5] +
        prod_reg[6] + prod_reg[7] + prod_reg[8];

    wire signed [39:0] acc_next = acc + kernel_sum;

    // ── ReLU + 클리핑 ─────────────────────────────────────────────────
    wire signed [15:0] relu_out;
    assign relu_out = acc_next[39]                ? 16'sd0    :
                      (acc_next > 40'sh0000_7FFF) ? 16'sh7FFF :
                      acc_next[15:0];

    // ── 메인 FSM ──────────────────────────────────────────────────────
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            ch_idx      <= 0;
            acc         <= 0;
            out_valid   <= 1'b0;
            feature_out <= 0;
            for (i = 0; i < C_IN; i = i + 1) begin
                out_buf[i]   <= 16'sd0;
                win_latch[i] <= 0;
            end
        end
        else begin
            out_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (all_win_valid) begin
                        // 윈도우 캡처 (FSM 진행 중 win_flat 변경 방지)
                        for (i = 0; i < C_IN; i = i + 1)
                            win_latch[i] <= win_flat[i];
                        state  <= ST_STAGE1;
                        ch_idx <= 0;
                        acc    <= {{24{bias_rom[0][15]}}, bias_rom[0]};
                    end
                end

                ST_STAGE1: begin
                    // prod_comb 계산 → prod_reg 래치 (위 always블록)
                    // 다음 사이클(ST_STAGE2)에서 덧셈트리 수행
                    state <= ST_STAGE2;
                end

                ST_STAGE2: begin
                    // prod_reg → kernel_sum → acc 누산
                    acc <= acc_next;

                    // 현재 채널 ReLU → out_buf 저장
                    out_buf[ch_idx] <= relu_out;

                    if (ch_idx == C_IN - 1) begin
                        state <= ST_OUTPUT;
                    end
                    else begin
                        ch_idx <= ch_idx + 1;
                        acc    <= {{24{bias_rom[ch_idx+1][15]}},
                                    bias_rom[ch_idx+1]};
                        state  <= ST_STAGE1;
                    end
                end

                ST_OUTPUT: begin
                    for (i = 0; i < C_IN; i = i + 1)
                        feature_out[i*16 +: 16] <= out_buf[i];
                    out_valid <= 1'b1;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule