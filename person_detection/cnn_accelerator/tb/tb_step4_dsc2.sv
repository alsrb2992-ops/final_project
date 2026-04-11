// tb_step4_dsc2.sv: CNN 모듈 검증 Step 4
//
// 대상 모듈: layer_dsc (C_IN=16, C_OUT=32, IMG_W_IN=30)
// 입력: 128x128x3 픽셀 스트리밍 (고정값)
// 확인 항목:
//     [1] dsc_valid 총 횟수 = 196 (14*14)
//     [2] 첫 번째 dsc_valid까지 걸리는 클럭 수
//     [3] dsc2_out xxxx 없는지 확인

`timescale 1ns / 1ps

module tb_step4_dsc2;
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

    initial clk = 0;
    always #10 clk = ~clk;

    // 카운터
    integer cycle_cnt = 0;
    integer dsc2_cnt = 0;              // dsc2_valid 총 횟수
    integer first_valid_cycle = -1;    // 첫 dsc2_valid 클럭 시점

    always @(posedge clk) begin
        // 클럭 카운터
        cycle_cnt <= cycle_cnt + 1;

        if (dsc2_valid) begin
            // [1] dsc2_valid 횟수 카운트
            dsc2_cnt <= dsc2_cnt + 1;

            // [2] 첫 번째 dsc2_valid 타이밍 기록
            if (first_valid_cycle == -1) first_valid_cycle <= cycle_cnt;

            // [3] dsc2_out 샘플 출력 (처음 3개만)
            if (dsc2_cnt < 3) begin
                $display("[SAMPLE] dsc2_out[%0d] ch0:%04X ch1:%04X ch2:%04X ch3:%04X",
                         dsc2_cnt, dsc2_out[15:0], dsc2_out[31:16], dsc2_out[47:32], dsc2_out[63:48]);
            end
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
        $display("========== Step 4 Start: conv1 + MaxPool + DSC1 + DSC2 ==========");

        // dsc2_valid 196회 나올 때까지 pixel_valid 유지
        pixel_valid = 1;
        wait(dsc2_cnt >= 196);
        pixel_valid = 0;
        $display("[INFO] Pixel input done");
        $display("[INFO] First dsc2_valid at cycle: %0d", first_valid_cycle);

        @(posedge clk);
        $display("-----------------------------------------------------------------");

        // [1] dsc2_valid 횟수
        if (dsc2_cnt == 196) begin
            $display("[PASS] dsc2_valid count: %0d / 196", dsc2_cnt);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] dsc2_valid count: %0d / 196", dsc2_cnt);
        end

        // [2] 첫 dsc2_valid 타이밍
        if (first_valid_cycle > 0) begin
            $display("[PASS] First dsc2_valid at cycle: %0d", first_valid_cycle);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] dsc2_valid not detected");
        end

        $display("----------------------------------------------------------------");
        $display("[SUMMARY] Step 4 result: %0d / 2 PASS", pass_cnt);
        #100 $finish;
    end

    // 디버그 출력 (500k 클럭마다)
    always @(posedge clk) begin
        if ((cycle_cnt % 500_000 == 0) && (cycle_cnt > 0)) begin
            $display("[DEBUG] cycle: %0d dsc2_cnt=%0d pixel_valid=%b dsc2_valid=%b",
                     cycle_cnt, dsc2_cnt, pixel_valid, dsc2_valid);
        end
    end

    // 타임아웃
    initial begin
        #200_000_000;
        $display("[TIMEOUT] Simulation time limit exceeded");
        $display("[TIMEOUT] cycle=%0d dsc2_cnt=%0d", cycle_cnt, dsc2_cnt);
        $finish;
    end

endmodule
