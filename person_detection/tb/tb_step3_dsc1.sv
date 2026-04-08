// tb_step3_dsc1.sv: CNN 모듈 검증 Step 3
//
// 대상 모듈: layer_dsc (C_IN=8, C_OUT=16, IMG_W_IN=63)
//     내부 구성: conv_dw -> conv1x1 -> maxpool2x2 x16
// 입력: 128x128x3 픽셀 스트리밍 (고정값)
// 확인 항목:
//     [1] dsc_valid 총 횟수 = 900 (30x30)
//     [2] 첫 번째 dsc_valid까지 걸리는 클럭 수
//     [3] dsc_out XXX 없는지 확인
//
// 파이프라인 흐름:
//     conv3x3: 128x128x3 -> 126x126x8 (픽셀당 48클럭)
//     maxpool: 126x126x8 -> 63x63x8
//     conv_dw: 63x63x8 -> 61x61x8 (픽셀당 17클럭)
//     conv1x1: 61x61x8 -> 61x61x16 (픽셀당 33클럭)
//     maxpool: 61x61x16 -> 30x30x16

`timescale 1ns / 1ps

module tb_step3_dsc1;
    reg clk, rst_n;

    reg        pixel_valid;
    reg [47:0] pixel_in_3ch;    // 3ch x 16bit

    // Stage 1: conv3x3 출력
    wire         conv1_valid;
    wire [127:0] conv1_out;      // 8ch x 16bit

    // Stage 1: MaxPool 출력
    wire   [7:0] pool1_valid_arr;
    wire [127:0] pool1_out;                         // 8ch x 16bit
    wire         pool1_valid = &pool1_valid_arr;

    // Stage 2: DSC1 출력
    wire         dsc1_valid;
    wire [255:0] dsc1_out;      // 16ch x 16bit

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

    layer_dsc #(
        .C_IN(8), .C_OUT(16), .IMG_W_IN(63),
        .DW_WEIGHT_FILE("../../../../dummy_weights/dsc1_dw_w.hex"),
        .PW_WEIGHT_FILE("../../../../dummy_weights/dsc1_pw_w.hex")
    ) u_dsc1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(pool1_valid), .pixel_in(pool1_out),
        .out_valid(dsc1_valid), .feature_out(dsc1_out));

    initial clk = 0;
    always #10 clk = ~clk;

    // 카운터
    integer cycle_cnt = 0;
    integer dsc1_cnt = 0;              // dsc1_valid 총 횟수
    integer first_valid_cycle = -1;    // 첫 dsc1_valid 클럭 시점

    always @(posedge clk) begin
        // 전체 클럭 카운터
        cycle_cnt <= cycle_cnt + 1;

        if (dsc1_valid) begin
            // [1] dsc1_valid 횟수 카운트
            dsc1_cnt <= dsc1_cnt + 1;

            // [2] 첫 번째 dsc1_valid 타이밍 기록
            if (first_valid_cycle == -1) first_valid_cycle <= cycle_cnt;

            // [3] dsc1_out 샘플 출력
            if (dsc1_cnt < 3) begin
                $display("[SAMPLE] dsc1_out[%0d] ch0:%04X ch1:%04X ch2:%04X ch3:%04X",
                         dsc1_cnt, dsc1_out[15:0], dsc1_out[31:16], dsc1_out[47:32], dsc1_out[63:48]);
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
        $display("========== Step 3 Start: conv1 + MaxPool + DSC1 ==========");

        // dsc1_valid 900회 나올 때까지 pixel_valid 유지
        pixel_valid = 1;
        wait(dsc1_cnt >= 900);
        pixel_valid = 0;

        @(posedge clk);
        $display("----------------------------------------------------------");

        // [1] dsc1_valid 총 횟수 확인 (30x30 = 900)
        if (dsc1_cnt == 900) begin
            $display("[PASS] dsc1_valid count: %0d / 900", dsc1_cnt);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] dsc1_valid count: %0d / 900", dsc1_cnt);
        end

        // [2] 첫 번째 dsc1_valid 타이밍 확인
        if (first_valid_cycle > 0) begin
            $display("[PASS] First dsc1_valid at cycle: %0d", first_valid_cycle);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] dsc1_valid not detected");
        end

        $display("-----------------------------------------------------------");
        $display("[SUMMARY] Step 3 result: %0d / 2 PASS", pass_cnt);
        #100 $finish;
    end

    // 진행상황 출력 (500k 클럭마다)
    always @(posedge clk) begin
        if ((cycle_cnt % 500_000 == 0) && (cycle_cnt > 0)) begin
            $display("[DEBUG] cycle=%0d dsc1_cnt=%0d pool1_valid=%b dsc1_valid=%b",
                     cycle_cnt, dsc1_cnt, pool1_valid, dsc1_valid);
        end
    end

    // 타임아웃
    initial begin
        #500_000_000;
        $display("[TIMEOUT] Simulation time limit exceeded");
        $display("[TIMEOUT] cycle=%0d dsc1_cnt=%0d", cycle_cnt, dsc1_cnt);
        $finish;
    end

endmodule
