// =============================================================================
// cnn_top.v  —  PersonGridCNN 가속기 최상위 모듈 (Q4.12, 128×128)
// =============================================================================
// 파이프라인:
//   입력 (128×128×3, RGB444 → Q4.12 16bit)
//   → Conv1 (3×3, 3→8, pad=0) → ReLU → 126×126×8
//   → MaxPool 2×2              → 63×63×8
//   → DSC1 (DW 3×3 8→8 → PW 1×1 8→16 → Pool) → 30×30×16
//   → DSC2 (DW 3×3 16→16 → PW 1×1 16→32 → Pool) → 14×14×32
//   → Conv_out (2×2, 32→1, no ReLU) → 13×13×1 logit
//   → Sigmoid LUT → 13×13 확률맵 (8bit)
//
// 데이터 폭: Q4.12 (16bit signed) 전체 경로
// 누산기: 40bit
// 가중치: 16bit signed, $readmemh로 로드
//
// 인터페이스:
//   pixel_in: RGB 각 채널 16bit (R/G/B 각각 Q4.12)
//   grid_prob: 8bit 확률 (0~255), 13×13=169개 직렬 스트리밍ddddddd
// =============================================================================

module cnn_top (
    input  wire        clk,
    input  wire        rst_n,

    // 픽셀 스트리밍 입력 (RGB 각 16bit Q4.12)
    input  wire        pixel_valid,
    input  wire [15:0] pixel_r,
    input  wire [15:0] pixel_g,
    input  wire [15:0] pixel_b,

    // 격자 확률맵 출력 (13×13=169픽셀 직렬)
    output wire        grid_valid,
    output wire [7:0]  grid_prob,

    // 추론 완료 신호
    output reg         inference_done
);

    // ── 입력 채널 묶음 (3ch × 16bit = 48bit) ────────────────────────
    wire [47:0] pixel_in_3ch = {pixel_b, pixel_g, pixel_r};

    // =====================================================================
    // Stage 1: Conv1 (3×3, 3→8) + MaxPool 2×2
    // 128×128×3 → Conv → 126×126×8 → Pool → 63×63×8
    // =====================================================================
    wire                conv1_valid;
    wire [8*16-1:0]     conv1_out;   // 8ch × 16bit = 128bit

    conv3x3 #(
        .C_IN        (3),
        .C_OUT       (8),
        .IMG_W_IN    (128),
        .WEIGHT_FILE ("../../../../dummy_weights/conv1_w.hex")
    ) u_conv1 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (pixel_valid),
        .pixel_in   (pixel_in_3ch),
        .out_valid  (conv1_valid),
        .feature_out(conv1_out)
    );

    // MaxPool 8채널 각각 (126×126 → 63×63)
    wire [7:0]       pool1_valid_arr;
    wire [8*16-1:0]  pool1_out;

    genvar ch1;
    generate
        for (ch1 = 0; ch1 < 8; ch1 = ch1 + 1) begin : POOL1
            maxpool2x2 #(
                .DATA_WIDTH (16),
                .IMG_WIDTH  (126)
            ) u_pool1 (
                .clk      (clk),
                .rst_n    (rst_n),
                .in_valid (conv1_valid),
                .pixel_in ($signed(conv1_out[ch1*16 +: 16])),
                .out_valid(pool1_valid_arr[ch1]),
                .max_out  (pool1_out[ch1*16 +: 16])
            );
        end
    endgenerate

    wire pool1_valid = &pool1_valid_arr;

    // =====================================================================
    // Stage 2: DSC1 (DW 8→8, PW 8→16, Pool)
    // 63×63×8 → DW → 61×61×8 → PW → 61×61×16 → Pool → 30×30×16
    // =====================================================================
    wire                dsc1_valid;
    wire [16*16-1:0]   dsc1_out;    // 16ch × 16bit = 256bit

    layer_dsc #(
        .C_IN           (8),
        .C_OUT          (16),
        .IMG_W_IN       (63),
        .DW_WEIGHT_FILE ("../../../../dummy_weights/dsc1_dw_w.hex"),
        .PW_WEIGHT_FILE ("../../../../dummy_weights/dsc1_pw_w.hex")
    ) u_dsc1 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (pool1_valid),
        .pixel_in   (pool1_out),
        .out_valid  (dsc1_valid),
        .feature_out(dsc1_out)
    );

    // =====================================================================
    // Stage 3: DSC2 (DW 16→16, PW 16→32, Pool)
    // 30×30×16 → DW → 28×28×16 → PW → 28×28×32 → Pool → 14×14×32
    // =====================================================================
    wire                dsc2_valid;
    wire [32*16-1:0]   dsc2_out;    // 32ch × 16bit = 512bit

    layer_dsc #(
        .C_IN           (16),
        .C_OUT          (32),
        .IMG_W_IN       (30),
        .DW_WEIGHT_FILE ("../../../../dummy_weights/dsc2_dw_w.hex"),
        .PW_WEIGHT_FILE ("../../../../dummy_weights/dsc2_pw_w.hex")
    ) u_dsc2 (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (dsc1_valid),
        .pixel_in   (dsc1_out),
        .out_valid  (dsc2_valid),
        .feature_out(dsc2_out)
    );

    // =====================================================================
    // Stage 4: Conv_out (2×2, 32→1, no ReLU) → 13×13×1 logit
    // =====================================================================
    wire               conv_out_valid;
    wire signed [39:0] conv_out_raw;

    conv2x2 #(
        .C_IN        (32),
        .IMG_W_IN    (14),
        .WEIGHT_FILE ("../../../../dummy_weights/conv_out_w.hex")
    ) u_conv_out (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (dsc2_valid),
        .pixel_in   (dsc2_out),
        .out_valid  (conv_out_valid),
        .acc_out_raw(conv_out_raw)
    );

    // =====================================================================
    // Stage 5: Sigmoid LUT → 13×13 격자 확률맵 (8bit)
    // =====================================================================
    sigmoid_lut u_sigmoid (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (conv_out_valid),
        .in_acc   (conv_out_raw),
        .out_valid(grid_valid),
        .prob_out (grid_prob)
    );

    // =====================================================================
    // 추론 완료 카운터 (13×13 = 169개 출력 후 inference_done 펄스)
    // =====================================================================
    reg [7:0] out_cnt; // 0~168

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_cnt        <= 8'd0;
            inference_done <= 1'b0;
        end
        else begin
            inference_done <= 1'b0;
            if (grid_valid) begin
                if (out_cnt == 8'd168) begin
                    out_cnt        <= 8'd0;
                    inference_done <= 1'b1;
                end
                else begin
                    out_cnt <= out_cnt + 1;
                end
            end
        end
    end

endmodule