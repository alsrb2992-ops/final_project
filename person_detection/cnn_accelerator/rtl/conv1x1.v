// =============================================================================
// conv1x1.v  —  1×1 Convolution (PW Conv 공용, Q4.12)
// =============================================================================
// 사용처:
//   DSC1 PW: C_IN=8→C_OUT=16,  USE_RELU=1
//   DSC2 PW: C_IN=16→C_OUT=32, USE_RELU=1
//
// [수정사항]
//   C_IN개 MAC 병렬화 + 2단 파이프라인 (conv3x3 방식 동일)
//     Stage1: C_IN개 곱셈 동시 수행 → prod_reg 래치
//     Stage2: 덧셈트리(C_IN개 합산) → acc 누산
//   ict 루프 제거, oct 루프만 유지
//
//   1픽셀당 사이클:
//     (ST_STAGE1 + ST_STAGE2) × C_OUT + ST_OUTPUT
//     = 2 × C_OUT + 1
//     DSC1 PW (C_IN=8,  C_OUT=16): 33 cycle  (기존 130 → 4.0배 향상)
//     DSC2 PW (C_IN=16, C_OUT=32): 65 cycle  (기존 514 → 7.9배 향상)
// =============================================================================

module conv1x1 #(
    parameter C_IN        = 8,
    parameter C_OUT       = 16,
    parameter USE_RELU    = 1,
    parameter WEIGHT_FILE = ""
) (
    input  wire clk,
    input  wire rst_n,

    input  wire                   in_valid,
    input  wire [C_IN*16-1:0]    pixel_in,

    output reg                    out_valid,
    output reg  [C_OUT*16-1:0]   feature_out,
    output reg  signed [39:0]    acc_out_raw    // USE_RELU=0 시 raw 누산값
);

    // ── 가중치 ROM ────────────────────────────────────────────────────
    localparam WEIGHT_DEPTH = C_OUT * C_IN;
    (* rom_style = "block" *) reg signed [15:0] weight_rom [0:WEIGHT_DEPTH-1];

    initial begin
        if (WEIGHT_FILE != "")
            $readmemh(WEIGHT_FILE, weight_rom);
    end

    // ── 편향 ROM ──────────────────────────────────────────────────────
    reg signed [15:0] bias_rom [0:C_OUT-1];
    integer bi;
    initial begin
        for (bi = 0; bi < C_OUT; bi = bi + 1)
            bias_rom[bi] = 16'sd0;
    end

    // ── FSM 상태 정의 ─────────────────────────────────────────────────
    localparam ST_IDLE   = 2'd0;
    localparam ST_STAGE1 = 2'd1;  // C_IN개 곱셈 (조합) → prod_reg 래치
    localparam ST_STAGE2 = 2'd2;  // 덧셈트리 → acc 누산
    localparam ST_OUTPUT = 2'd3;

    reg [1:0]                  state;
    reg [$clog2(C_OUT)-1:0]   oct;
    reg signed [39:0]          acc;

    reg [C_IN*16-1:0]  pixel_latch;
    reg signed [15:0]  out_buf [0:C_OUT-1];
    reg signed [39:0]  raw_buf;

    // ── Stage1: C_IN개 곱셈 병렬 (조합 논리) ─────────────────────────
    // oct번째 출력 채널의 C_IN개 가중치와 입력 픽셀을 동시에 곱셈
    wire signed [31:0] prod_comb [0:C_IN-1];
    wire signed [39:0] prod_scaled_comb [0:C_IN-1];

    genvar ci;
    generate
        for (ci = 0; ci < C_IN; ci = ci + 1) begin : PMAC
            (* use_dsp = "yes" *)
            assign prod_comb[ci] = $signed(pixel_latch[ci*16 +: 16])
                                 * weight_rom[oct * C_IN + ci];
            assign prod_scaled_comb[ci] = {{20{prod_comb[ci][31]}},
                                            prod_comb[ci][31:12]};
        end
    endgenerate

    // ── Stage1 → Stage2 파이프라인 레지스터 ──────────────────────────
    reg signed [39:0] prod_reg [0:C_IN-1];

    integer ri;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (ri = 0; ri < C_IN; ri = ri + 1)
                prod_reg[ri] <= 40'sd0;
        end
        else if (state == ST_STAGE1) begin
            for (ri = 0; ri < C_IN; ri = ri + 1)
                prod_reg[ri] <= prod_scaled_comb[ri];
        end
    end

    // ── Stage2: 덧셈트리 (prod_reg C_IN개 합산) ──────────────────────
    // Verilog-2001 호환: generate로 누적합 계산
    // C_IN이 파라미터이므로 for문 대신 wire 배열로 트리 구성
    wire signed [39:0] tree_sum;

    // C_IN개의 prod_reg를 모두 더함
    // Verilog-2001에서 파라미터 기반 가변 길이 덧셈트리는
    // 아래와 같이 generate + 중간 wire 배열로 구현
    wire signed [39:0] partial [0:C_IN-1];
    genvar ti;
    generate
        assign partial[0] = prod_reg[0];
        for (ti = 1; ti < C_IN; ti = ti + 1) begin : TREE
            assign partial[ti] = partial[ti-1] + prod_reg[ti];
        end
    endgenerate
    assign tree_sum = partial[C_IN-1];

    wire signed [39:0] acc_next = acc + tree_sum;

    // ── ReLU + 클리핑 ─────────────────────────────────────────────────
    wire signed [15:0] relu_out;
    assign relu_out = acc_next[39]                 ? 16'sd0       :
                      (acc_next > 40'sh0000_7FFF)  ? 16'sh7FFF    :
                      acc_next[15:0];

    // ── 메인 FSM ──────────────────────────────────────────────────────
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            oct         <= 0;
            acc         <= 0;
            out_valid   <= 1'b0;
            feature_out <= 0;
            pixel_latch <= 0;
            raw_buf     <= 0;
            for (i = 0; i < C_OUT; i = i + 1)
                out_buf[i] <= 16'sd0;
        end
        else begin
            out_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (in_valid) begin
                        pixel_latch <= pixel_in;
                        state       <= ST_STAGE1;
                        oct         <= 0;
                        acc         <= {{24{bias_rom[0][15]}}, bias_rom[0]};
                    end
                end

                ST_STAGE1: begin
                    // 이 사이클에 prod_comb 계산 → prod_reg 래치 (위 always블록)
                    // 다음 사이클(ST_STAGE2)에서 덧셈트리 수행
                    state <= ST_STAGE2;
                end

                ST_STAGE2: begin
                    // prod_reg → tree_sum → acc 누산
                    acc <= acc_next;

                    // 출력 채널 1개 완료 → ReLU 또는 raw 저장
                    if (USE_RELU) begin
                        if (acc_next[39])
                            out_buf[oct] <= 16'sd0;
                        else if (acc_next > 40'sh0000_7FFF)
                            out_buf[oct] <= 16'sh7FFF;
                        else
                            out_buf[oct] <= acc_next[15:0];
                    end
                    else begin
                        raw_buf <= acc_next;
                    end

                    if (oct == C_OUT - 1) begin
                        state <= ST_OUTPUT;
                    end
                    else begin
                        oct   <= oct + 1;
                        acc   <= {{24{bias_rom[oct+1][15]}}, bias_rom[oct+1]};
                        state <= ST_STAGE1;
                    end
                end

                ST_OUTPUT: begin
                    if (USE_RELU) begin
                        for (i = 0; i < C_OUT; i = i + 1)
                            feature_out[i*16 +: 16] <= out_buf[i];
                    end
                    else begin
                        acc_out_raw <= raw_buf;
                    end
                    out_valid <= 1'b1;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule