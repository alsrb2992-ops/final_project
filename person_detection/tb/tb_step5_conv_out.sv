// tb_step5_conv_out.sv: CNN 모듈 검증 Step 5
//
// 대상 모듈: conv2x2 (C_IN=32, C_OUT=1, IMG_W_IN=14)
//     2x2 Conv, 32채널->1채널, ReLU 없음 (raw 누산값 출력)
// 입력: 128x128x3 픽셀 스트리밍 (고정값)
// 확인 항목:
//     [1] conv_out_valid 총 횟수 = 169 (13*13)
//     [2] 첫 번째 conv_out_valid까지 걸리는 클럭 수
//     [3] conv_out_raw xxxx 없는지 확인 (40bit signed)

`timescale 1ns / 1ps

module tb_step5_conv_out;
    reg clk, rst_n;

    reg        pixel_valid;
    reg [47:0] pixel_in_3ch;    // 3ch x 16bit

    wire         conv1_valid;
    wire [127:0] conv1_out;      // 8ch x 16bit

    wire   [7:0] pool1_valid_arr;
    wire [127:0] pool1_out;          // 8ch x 16bit
    wire         pool1_valid;

    wire         dsc1_valid;
    wire [255:0] dsc1_out;      // 16ch x 16bit

    wire         dsc2_valid;
    wire [511:0] dsc2_out;      // 32ch x 16bit

    wire               conv_out_valid;
    wire signed [39:0] conv_out_raw;      // 40bit 누산값 (ReLU 전)

    conv3x3 #(.C_IN(3), .C_OUT(8), .IMG_W_IN(128), .WEIGHT_FILE("../../../../dummy_weights/conv1_w.hex")) u_conv1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(pixel_valid), .pixel_in(pixel_in_3ch),
        .out_valid(conv1_valid), .feature_out(conv1_out));

    genvar ch1;
    generate
        for (ch1=0; ch1<8; ch1++) begin : POOL1
            maxpool2x2 #(.DATA_WIDTH(16), .IMG_WIDTH(126)) u_pool1 (
                .clk(clk), .rst_n(rst_n),
                .in_valid(conv1_valid), .pixel_in(conv1_out[ch1*16 +: 16]),
                .out_valid(pool1_valid_arr[ch1]), .max_out(pool1_out[ch1*16 +: 16]));
        end
    endgenerate

    assign pool1_valid = &pool1_valid_arr;

    layer_dsc #(
        .C_IN(8), .C_OUT(16), .IMG_W_IN(63),
        .DW_WEIGHT_FILE("../../../../dummy_weights/dsc1_dw_w.hex"),
        .PW_WEIGHT_FILE("../../../../dummy_weights/dsc1_pw_w.hex")
    ) u_dsc1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(pool1_valid), .pixel_in(pool1_out),
        .out_valid(dsc1_valid), .feature_out(dsc1_out));

    layer_dsc #(
        .C_IN(16), .C_OUT(32), .IMG_W_IN(30),
        .DW_WEIGHT_FILE("../../../../dummy_weights/dsc2_dw_w.hex"),
        .PW_WEIGHT_FILE("../../../../dummy_weights/dsc2_pw_w.hex")
    ) u_dsc2 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(dsc1_valid), .pixel_in(dsc1_out),
        .out_valid(dsc2_valid), .feature_out(dsc2_out));

    conv2x2 #(.C_IN(32), .IMG_W_IN(14), .WEIGHT_FILE("../../../../dummy_weights/conv_out_w.hex")) u_conv_out (
        .clk(clk), .rst_n(rst_n),
        .in_valid(dsc2_valid), .pixel_in(dsc2_out),
        .out_valid(conv_out_valid), .acc_out_raw(conv_out_raw));

    initial clk = 0;
    always #10 clk = ~clk;

    // 카운터
    integer cycle_cnt = 0;
    integer conv_out_cnt = 0;          // conv_out_valid 총 횟수
    integer first_valid_cycle = -1;    // 첫 conv_out_valid 클럭 시점

    always @(posedge clk) begin
        // 클럭 카운터
        cycle_cnt <= cycle_cnt + 1;

        if (conv_out_valid) begin
            // [1] conv_out_valid 횟수 카운트
            conv_out_cnt <= conv_out_cnt + 1;

            // [2] 첫 번째 conv_out_valid 타이밍 기록
            if (first_valid_cycle == -1) first_valid_cycle <= cycle_cnt;

            // [3] conv_out_raw 샘플 출력 (처음 5개)
            if (conv_out_cnt < 5) $display("[SAMPLE] conv_out_raw[%0d] = %0d (0x%010X)",
                                           conv_out_cnt, conv_out_raw, conv_out_raw);
        end
    end

    integer pass_cnt;
    initial begin
        pass_cnt = 0;
        rst_n = 0;
        pixel_valid = 0;
        pixel_in_3ch = {16'h0400, 16'h0800, 16'h1000};    // Q4.12 고정값 (B=0.25, G=0.5, R=1.0)

        repeat(3) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);
        $display("========== Step 5 Start: + conv_out ==========");

        // 128x128 픽셀 입력 -> conv_out_valid 169회까지 유지
        pixel_valid = 1;
        wait(conv_out_cnt >= 169);
        pixel_valid = 0;
        $display("[INFO] Pixel input done");
        $display("[INFO] First conv_out_valid at cycle %0d", first_valid_cycle);

        @(posedge clk);
        $display("----------------------------------------------");

        // [1] conv_out_valid 횟수
        if (conv_out_cnt == 169) begin
            $display("[PASS] conv_out_valid count: %0d / 169", conv_out_cnt);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] conv_out_valid count: %0d / 169", conv_out_cnt);
        end

        // [2] 첫 conv_out_valid 타이밍
        if (first_valid_cycle > 0) begin
            $display("[PASS] First conv_out_valid at cycle: %0d", first_valid_cycle);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] conv_out_valid not detected");
        end

        $display("----------------------------------------------");
        $display("[SUMMARY] Step 5 result: %0d / 2 PASS", pass_cnt);
        #100 $finish;
    end

    // 디버그 출력 (500k 클럭마다)
    always @(posedge clk) begin
        if ((cycle_cnt % 500_000 == 0) && (cycle_cnt > 0)) begin
            $display("[DEBUG] cycle: %0d conv_out_cnt=%0d pixel_valid=%b conv_out_valid=%b",
                     cycle_cnt, conv_out_cnt, pixel_valid, conv_out_valid);
        end
    end

    // 타임아웃
    initial begin
        #200_000_000;
        $display("[TIMEOUT] Simulation time limit exceeded");
        $display("[TIMEOUT] cycle=%0d, conv_out_cnt=%0d", cycle_cnt, conv_out_cnt);
        $finish;
    end

endmodule
