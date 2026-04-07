// =============================================================================
// conv3x3.v
// -----------------------------------------------------------------------------
// 3×3 Convolution (C_out 시분할)
//
// Q4.12 데이터 경로:
//   가중치/활성화: signed 16bit
//   곱셈: 16×16 = 32bit → >>12 → signed 40bit 누산
//   출력: ReLU + 클리핑 → signed 16bit (Q4.12)
//
// Conv1: C_IN=3, C_OUT=8, IMG_W_IN=128, pad=0
//   입력 128×128×3, 출력 126×126×8
//   1픽셀당 사이클: 9 × C_IN × C_OUT = 216 (+ IDLE/OUTPUT = 218)
//
// [수정사항]
//   1. Q4.4 → Q4.12 (8bit → 16bit)
//   2. acc_next 패턴 → 마지막 MAC 누락 버그 수정
//   3. (* use_dsp = "yes" *) DSP48E1 강제 매핑
//   4. IMG_W_IN 기본값 128
//   5. 누산기 40bit
//   6. bias 16bit
// =============================================================================

module conv3x3 #(
    parameter C_IN        = 3,
    parameter C_OUT       = 8,
    parameter IMG_W_IN    = 128,
    parameter WEIGHT_FILE = "conv1_w.hex"
) (
    input  wire clk,
    input  wire rst_n,

    input  wire                    in_valid,
    input  wire [C_IN*16-1:0]     pixel_in,

    output reg                     out_valid,
    output reg  [C_OUT*16-1:0]    feature_out
);

    // ── 가중치 ROM ────────────────────────────────────────────────────
    localparam WEIGHT_DEPTH = C_OUT * C_IN * 9;
    // reg signed [15:0] weight_rom [0:WEIGHT_DEPTH-1];
    (* rom_style = "block" *) reg signed [15:0] weight_rom [0:WEIGHT_DEPTH-1];
    initial begin
        if (WEIGHT_FILE != "")
            $readmemh(WEIGHT_FILE, weight_rom);
    end

    // ── 편향 ROM (Q4.12) ──────────────────────────────────────────────
    reg signed [15:0] bias_rom [0:C_OUT-1];
    integer bi;
    initial begin
        for (bi = 0; bi < C_OUT; bi = bi + 1)
            bias_rom[bi] = 16'sd0;
    end

    // ── 라인버퍼 ──────────────────────────────────────────────────────
    wire [15:0] lb_row0 [0:C_IN-1];
    wire [15:0] lb_row1 [0:C_IN-1];
    wire [15:0] lb_row2 [0:C_IN-1];
    wire [C_IN-1:0] lb_valid;

    genvar ch;
    generate
        for (ch = 0; ch < C_IN; ch = ch + 1) begin : LB
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

    // ── 윈도우 레지스터 ───────────────────────────────────────────────
    wire [16*9-1:0] win_flat [0:C_IN-1];
    wire [C_IN-1:0] win_valid;

    generate
        for (ch = 0; ch < C_IN; ch = ch + 1) begin : WIN
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

    // ── FSM ───────────────────────────────────────────────────────────
    localparam ST_IDLE    = 2'd0;
    localparam ST_COMPUTE = 2'd1;
    localparam ST_OUTPUT  = 2'd2;

    reg [1:0] state;
    reg [$clog2(C_OUT)-1:0] oct;
    reg [$clog2(C_IN)-1:0]  ict;
    reg [3:0]               kidx;

    reg signed [39:0] acc;
    reg signed [15:0] out_buf [0:C_OUT-1];

    // 가중치 주소
    wire [$clog2(WEIGHT_DEPTH)-1:0] w_addr = oct * (C_IN * 9) + ict * 9 + kidx;
    wire signed [15:0] cur_pixel  = $signed(win_flat[ict][kidx*16 +: 16]);
    wire signed [15:0] cur_weight = weight_rom[w_addr];

    // 곱셈 — DSP48E1 강제 사용
    (* use_dsp = "yes" *)
    wire signed [31:0] product = cur_pixel * cur_weight;

    // Q8.24 → Q4.12: >>12
    wire signed [39:0] prod_scaled = {{20{product[31]}}, product[31:12]};

    // acc_next: 버그 수정 핵심
    wire signed [39:0] acc_next = acc + prod_scaled;

    // ReLU + 클리핑
    wire signed [15:0] relu_out;
    assign relu_out = acc_next[39]            ? 16'sd0 :
                      (acc_next > 40'sd32767) ? 16'sh7FFF :
                      acc_next[15:0];

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            oct         <= 0;
            ict         <= 0;
            kidx        <= 0;
            acc         <= 0;
            out_valid   <= 1'b0;
            feature_out <= 0;
            for (i = 0; i < C_OUT; i = i + 1)
                out_buf[i] <= 16'sd0;
        end
        else begin
            out_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (all_win_valid) begin
                        state <= ST_COMPUTE;
                        oct   <= 0;
                        ict   <= 0;
                        kidx  <= 0;
                        acc   <= {{24{bias_rom[0][15]}}, bias_rom[0]};
                    end
                end

                ST_COMPUTE: begin
                    acc <= acc_next;

                    if (kidx == 4'd8) begin
                        kidx <= 0;
                        if (ict == C_IN - 1) begin
                            // 모든 입력 채널 완료 → ReLU (acc_next 사용)
                            out_buf[oct] <= relu_out;
                            ict <= 0;

                            if (oct == C_OUT - 1) begin
                                state <= ST_OUTPUT;
                            end
                            else begin
                                oct <= oct + 1;
                                acc <= {{24{bias_rom[oct+1][15]}}, bias_rom[oct+1]};
                            end
                        end
                        else begin
                            ict <= ict + 1;
                        end
                    end
                    else begin
                        kidx <= kidx + 1;
                    end
                end

                ST_OUTPUT: begin
                    for (i = 0; i < C_OUT; i = i + 1)
                        feature_out[i*16 +: 16] <= out_buf[i];
                    out_valid <= 1'b1;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
