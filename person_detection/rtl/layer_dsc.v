// =============================================================================
// layer_dsc.v  —  Depthwise Separable Convolution 레이어 (Q4.12)
// =============================================================================
// DW Conv 3×3 → ReLU(내장) → PW Conv 1×1 → ReLU(내장) → MaxPool 2×2
//
// 치수 검증 (128×128 입력 아키텍처):
//   DSC1: 63×63×8  → DW→61×61×8  → PW→61×61×16  → Pool→30×30×16
//   DSC2: 30×30×16 → DW→28×28×16 → PW→28×28×32  → Pool→14×14×32
// =============================================================================

module layer_dsc #(
    parameter C_IN           = 8,
    parameter C_OUT          = 16,
    parameter IMG_W_IN       = 63,
    parameter DW_WEIGHT_FILE = "",
    parameter PW_WEIGHT_FILE = ""
) (
    input  wire clk,
    input  wire rst_n,

    input  wire                   in_valid,
    input  wire [C_IN*16-1:0]    pixel_in,

    output wire                   out_valid,
    output wire [C_OUT*16-1:0]   feature_out
);

    // DW 출력 가로: pad=0 → IMG_W_IN - 2
    localparam IMG_W_DW = IMG_W_IN - 2;

    // ── DW Conv 3×3 ──────────────────────────────────────────────────
    wire                  dw_valid;
    wire [C_IN*16-1:0]   dw_out;

    conv_dw #(
        .C_IN        (C_IN),
        .IMG_W_IN    (IMG_W_IN),
        .WEIGHT_FILE (DW_WEIGHT_FILE)
    ) u_dw (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (in_valid),
        .pixel_in   (pixel_in),
        .out_valid  (dw_valid),
        .feature_out(dw_out)
    );

    // ── PW Conv 1×1 ──────────────────────────────────────────────────
    wire                  pw_valid;
    wire [C_OUT*16-1:0]  pw_out;
    wire signed [39:0]   pw_raw;  // USE_RELU=1 이므로 미사용

    conv1x1 #(
        .C_IN        (C_IN),
        .C_OUT       (C_OUT),
        .USE_RELU    (1),
        .WEIGHT_FILE (PW_WEIGHT_FILE)
    ) u_pw (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (dw_valid),
        .pixel_in   (dw_out),
        .out_valid  (pw_valid),
        .feature_out(pw_out),
        .acc_out_raw(pw_raw)
    );

    // ── MaxPool 2×2 (C_OUT 채널 각각) ────────────────────────────────
    wire [C_OUT-1:0]     pool_valid_arr;
    wire [C_OUT*16-1:0]  pool_out_arr;

    genvar ch;
    generate
        for (ch = 0; ch < C_OUT; ch = ch + 1) begin : POOL
            maxpool2x2 #(
                .DATA_WIDTH (16),
                .IMG_WIDTH  (IMG_W_DW)
            ) u_pool (
                .clk      (clk),
                .rst_n    (rst_n),
                .in_valid (pw_valid),
                .pixel_in ($signed(pw_out[ch*16 +: 16])),
                .out_valid(pool_valid_arr[ch]),
                .max_out  (pool_out_arr[ch*16 +: 16])
            );
        end
    endgenerate

    assign out_valid   = &pool_valid_arr;
    assign feature_out = pool_out_arr;

endmodule
