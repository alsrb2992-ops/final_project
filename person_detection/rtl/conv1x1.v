// =============================================================================
// conv1x1.v  —  1×1 Convolution (PW Conv / Conv_out 공용, Q4.12)
// =============================================================================
// 사용처:
//   DSC1 PW: C_IN=8→C_OUT=16,  USE_RELU=1
//   DSC2 PW: C_IN=16→C_OUT=32, USE_RELU=1
//   Conv_out: C_IN=32→C_OUT=1, USE_RELU=0 (Sigmoid 입력용 raw 출력)
//
// [주의] Conv_out은 실제로 2×2 커널이지만 별도 conv2x2 모듈로 처리.
//        이 모듈은 순수 1×1 pointwise만 담당.
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

// initial begin
//         acc_out_raw = 40'sd0;
//     end

    // ── 가중치 ROM ────────────────────────────────────────────────────
    // localparam WEIGHT_DEPTH = C_OUT * C_IN;
    // reg signed [15:0] weight_rom [0:WEIGHT_DEPTH-1];

    // initial begin
    //     if (WEIGHT_FILE != "")
    //         $readmemh(WEIGHT_FILE, weight_rom);
    // end
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

    // ── FSM ──────────────────────────────────────────────────────────
    localparam ST_IDLE    = 2'd0;
    localparam ST_COMPUTE = 2'd1;
    localparam ST_OUTPUT  = 2'd2;

    reg [1:0]                   state;
    reg [$clog2(C_OUT+1)-1:0]  oct;
    reg [$clog2(C_IN+1)-1:0]   ict;
    reg signed [39:0]           acc;

    reg [C_IN*16-1:0]  pixel_latch;
    reg signed [15:0]  out_buf [0:C_OUT-1];
    reg signed [39:0]  raw_buf;

    wire signed [15:0] cur_pixel  = $signed(pixel_latch[ict*16 +: 16]);
    wire signed [15:0] cur_weight = weight_rom[oct * C_IN + ict];

    wire signed [31:0] product     = cur_pixel * cur_weight;
    wire signed [39:0] prod_scaled = {{20{product[31]}}, product[31:12]};
    wire signed [39:0] acc_next    = acc + prod_scaled;

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            oct         <= 0;
            ict         <= 0;
            acc         <= 0;
            out_valid   <= 1'b0;
            feature_out <= 0;
            // acc_out_raw <= 0; // 비동기 리셋으로 인한 DRC 문제 발생 제거
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
                        state       <= ST_COMPUTE;
                        oct         <= 0;
                        ict         <= 0;
                        acc         <= {{24{bias_rom[0][15]}}, bias_rom[0]};
                    end
                end

                ST_COMPUTE: begin
                    acc <= acc_next;

                    if (ict == C_IN - 1) begin
                        ict <= 0;

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
                            oct <= oct + 1;
                            acc <= {{24{bias_rom[oct+1][15]}}, bias_rom[oct+1]};
                        end
                    end
                    else begin
                        ict <= ict + 1;
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
