// =============================================================================
// conv_dw.v  —  Depthwise Convolution 3×3 (Q4.12)
// =============================================================================
// 채널별 독립 3×3 conv, C_IN 채널 시분할 처리
// DSC1 DW: C_IN=8,  63×63→61×61
// DSC2 DW: C_IN=16, 30×30→28×28
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
    reg signed [15:0] weight_rom [0:WEIGHT_DEPTH-1];

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

    // ── 라인버퍼 + 윈도우 (C_IN 각각) ────────────────────────────────
    wire [15:0]     lb_row0 [0:C_IN-1];
    wire [15:0]     lb_row1 [0:C_IN-1];
    wire [15:0]     lb_row2 [0:C_IN-1];
    wire [C_IN-1:0] lb_valid;

    wire [16*9-1:0]  win_flat  [0:C_IN-1];
    wire [C_IN-1:0]  win_valid;

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

    // ── 윈도우 래치 ──────────────────────────────────────────────────
    reg [16*9-1:0] win_latch [0:C_IN-1];

    // ── FSM ──────────────────────────────────────────────────────────
    localparam ST_IDLE    = 2'd0;
    localparam ST_COMPUTE = 2'd1;
    localparam ST_OUTPUT  = 2'd2;

    reg [1:0]                  state;
    reg [$clog2(C_IN)-1:0]    ch_idx;
    reg [3:0]                  kidx;
    reg signed [39:0]          acc;
    reg signed [15:0]          out_buf [0:C_IN-1];

    wire signed [15:0] cur_pixel  = $signed(win_latch[ch_idx][kidx*16 +: 16]);
    wire signed [15:0] cur_weight = weight_rom[ch_idx * 9 + kidx];

    wire signed [31:0] product     = cur_pixel * cur_weight;
    wire signed [39:0] prod_scaled = {{20{product[31]}}, product[31:12]};
    wire signed [39:0] acc_next    = acc + prod_scaled;

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            ch_idx      <= 0;
            kidx        <= 0;
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
                        for (i = 0; i < C_IN; i = i + 1)
                            win_latch[i] <= win_flat[i];
                        state  <= ST_COMPUTE;
                        ch_idx <= 0;
                        kidx   <= 0;
                        acc    <= {{24{bias_rom[0][15]}}, bias_rom[0]};
                    end
                end

                ST_COMPUTE: begin
                    acc <= acc_next;

                    if (kidx == 4'd8) begin
                        kidx <= 0;
                        // ReLU on acc_next (마지막 MAC 반영)
                        if (acc_next[39])
                            out_buf[ch_idx] <= 16'sd0;
                        else if (acc_next > 40'sh0000_7FFF)
                            out_buf[ch_idx] <= 16'sh7FFF;
                        else
                            out_buf[ch_idx] <= acc_next[15:0];

                        if (ch_idx == C_IN - 1) begin
                            state <= ST_OUTPUT;
                        end
                        else begin
                            ch_idx <= ch_idx + 1;
                            acc    <= {{24{bias_rom[ch_idx+1][15]}}, bias_rom[ch_idx+1]};
                        end
                    end
                    else begin
                        kidx <= kidx + 1;
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
