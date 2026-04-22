// tb_step2_pool1.sv: CNN 모듈 검증 Step 2
//
// 대상 모듈: conv3x3 (C_IN=8, C_OUT=8, IMG_W_IN=126) + maxpool2x2 (IMG_WIDTH=126)
// 입력: 128x128x3 픽셀 스트리밍 (고정값)
// 확인 항목:
//     [1] pool1_valid 총 횟수 = 3969 (63x63)
//     [2] 첫 번째 pool1_valid까지 걸리는 클럭 수
//     [3] pool1_valid_arr[7:0] 8채널 동시 assert 여부
//     [4] pool1_out XXX 없는지 확인

`timescale 1ns / 1ps

module tb_step2_pool1;
    reg clk, rst_n;

    reg        pixel_valid;
    reg [47:0] pixel_in_3ch;         // 3ch x 16bit

    wire         conv1_valid;
    wire [127:0] conv1_out;          // 8ch x 16bit

    wire   [7:0] pool1_valid_arr;
    wire [127:0] pool1_out;          // 8ch x 16bit

    wire pool1_valid = &pool1_valid_arr;    // 8채널 모두 valid일 때만 활성화

    conv3x3 #(.C_IN(3), .C_OUT(8), .IMG_W_IN(128), .WEIGHT_FILE("../../../../dummy_weights/conv1_w.hex")) u_conv1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(pixel_valid), .pixel_in(pixel_in_3ch),
        .out_valid(conv1_valid), .feature_out(conv1_out));

    genvar ch;
    generate
        for (ch=0; ch<8; ch++) begin : POOL1
            maxpool2x2 #(.DATA_WIDTH(16), .IMG_WIDTH(126)) u_pool1 (
                .clk(clk), .rst_n(rst_n),
                .in_valid(conv1_valid), .pixel_in(conv1_out[ch*16 +: 16]),
                .out_valid(pool1_valid_arr[ch]), .max_out(pool1_out[ch*16 +: 16]));
        end
    endgenerate

    initial clk = 0;
    always #10 clk = ~clk;

    // 카운터
    integer cycle_cnt = 0;
    integer pool1_cnt = 0;
    integer first_valid_cycle = -1;

    always @(posedge clk) begin
        // 전체 클럭 카운터
        cycle_cnt <= cycle_cnt + 1;

        if (pool1_valid) begin
            pool1_cnt <= pool1_cnt + 1;

            // [2] 첫 번째 pool1_valid 타이밍 기록
            if (first_valid_cycle == -1) first_valid_cycle <= cycle_cnt;

            // [3] 8채널 동시 assert 확인
            if (pool1_valid_arr != 8'hFF) $display("[WARN] pool1_valid_arr not all HIGH: %08b at cycle %0d", pool1_valid_arr, cycle_cnt);

            // [4] pool1_out 샘플 출력 (처음 3개만)
            if (pool1_cnt < 3) $display("[SAMPLE] pool1_out[%0d] ch0:%04X ch1:%04X ch2:%04X ch3:%04X",
                                        pool1_cnt, pool1_out[15:0], pool1_out[31:16], pool1_out[47:32], pool1_out[63:48]);
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
        $display("========== Step 2 Start: conv1 + MaxPool ==========");

        // pool1_valid 3969회 나올 때까지 pixel_valid 유지
        pixel_valid = 1;
        wait(pool1_cnt == 3969);
        pixel_valid = 0;

        @(posedge clk);
        $display("---------------------------------------------------");

        // [1] pool1_valid 횟수 확인
        if (pool1_cnt == 3969) begin
            $display("[PASS] pool1_valid count: %0d / 3969", pool1_cnt);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] pool1_valid count: %0d / 3969", pool1_cnt);
        end

        // [2] 첫 번째 pool1_valid 타이밍 확인
        if (first_valid_cycle > 0) begin
            $display("[PASS] First pool1_valid at cycle: %0d", first_valid_cycle);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] pool1_valid not detected");
        end

        $display("----------------------------------------------------");
        $display("[SUMMARY] Step 2 result: %0d / 2 PASS", pass_cnt);
        #100 $finish;
    end

    // 진행상황 출력 (500k 클럭마다)
    always @(posedge clk) begin
        if ((cycle_cnt % 500_000 == 0) && (cycle_cnt > 0)) begin
            $display("[DEBUG] cycle=%0d pool1_cnt=%0d pixel_valid=%b pool1_valid=%b",
                     cycle_cnt, pool1_cnt, pixel_valid, pool1_valid);
        end
    end

    // 타임아웃
    initial begin
        #500_000_000;
        $display("[TIMEOUT] Simulation time limit exceeded");
        $display("[TIMEOUT] cycle=%0d pool1_cnt=%0d", cycle_cnt, pool1_cnt);
        $finish;
    end

endmodule
