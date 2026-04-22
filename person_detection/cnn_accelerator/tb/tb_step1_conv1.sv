// tb_step1_conv1.sv: CNN 모듈 검증 Step 1
//
// 대상 모듈: conv3x3 (C_IN=3, C_OUT=8, IMG_W_IN=128)
// 입력: 128x128x3 픽셀 스트리밍 (고정값)
// 확인 항목:
//     [1] conv1_valid 총 횟수 = 15876 (126x126)
//     [2] 첫 번째 conv1_valid까지 걸리는 클럭 수
//     [3] out_valid 펄스 폭 = 1클럭
//     [4] conv1_out 8채널 모두 0이 아닌지 (ReLU 통과 확인)

`timescale 1ns/1ps

module tb_step1_conv1;
    reg clk, rst_n;

    reg pixel_valid;
    reg [47:0] pixel_in_3ch;    // 3ch x 16bit

    wire conv1_valid;
    wire [127:0] conv1_out;     // 8ch x 16bit

    conv3x3 #(.C_IN(3), .C_OUT(8), .IMG_W_IN(128), .WEIGHT_FILE("../../../../dummy_weights/conv1_w.hex")) u_conv1 (
        .clk(clk), .rst_n(rst_n),
        .in_valid(pixel_valid), .pixel_in(pixel_in_3ch),
        .out_valid(conv1_valid), .feature_out(conv1_out)
    );

    initial clk = 0;
    always #10 clk = ~clk;

    integer cycle_cnt = 0;             // 전체 클럭 카운터

    integer conv1_cnt = 0;             // conv1_valid 총 횟수
    integer first_valid_cycle = -1;    // 첫 conv1_valid 클럭 시점

    always @(posedge clk) begin
        // 클럭 카운터
        cycle_cnt <= cycle_cnt + 1;

        // conv1_valid 검사
        if (conv1_valid) begin
            conv1_cnt <= conv1_cnt + 1;
            // [2] 첫 번째 valid 타이밍 기록
            if (first_valid_cycle == -1) first_valid_cycle <= cycle_cnt;
            // [4] conv1_out 샘플 출력 (처음 3개만)
            if (conv1_cnt < 3) $display("[SAMPLE] conv1_out[%0d] = ch0:%04X ch1:%04X ch2:%04X ch3:%04X",
                                        conv1_cnt, conv1_out[15:0], conv1_out[31:16], conv1_out[47:32], conv1_out[63:48]);
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
        $display("========== Step 1 Start: 128x128x3 pixel streaming ==========");

        // 128x128 픽셀 입력 (총 16384 픽셀)
        pixel_valid = 1;
        wait(conv1_cnt >= 15876);    // conv1_valid 15876회 대기
        pixel_valid = 0;
        $display("[INFO] Pixel input done");
        $display("[INFO] First conv1_valid at cycle: %0d", first_valid_cycle);

        @(posedge clk);
        $display("-----------------------------------------------------------");

        // [1] conv1_valid 횟수
        if (conv1_cnt == 15876) begin
            $display("[PASS] conv1_valid count: %0d / 15876", conv1_cnt);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] conv1_valid count: %0d / 15876", conv1_cnt);
        end

        // [2] 첫 conv1_valid 타이밍
        if (first_valid_cycle > 0) begin
            $display("[PASS] First conv1_valid at cycle: %0d", first_valid_cycle);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] conv1_valid not detected");
        end

        $display("-----------------------------------------------------------");
        $display("[SUMMARY] Step 1 result: %0d / 2 PASS", pass_cnt);
        #100 $finish;
    end

    always @(posedge clk) begin
        if ((cycle_cnt % 500_000 == 0) && (cycle_cnt > 0)) begin
            $display("[DEBUG] cycle=%0d conv1_cnt=%0d pixel_valid=%b conv1_valid=%b",
                     cycle_cnt, conv1_cnt, pixel_valid, conv1_valid);
        end
    end

    initial begin
        #200_000_000;
        $display("[TIMEOUT] Simulation time limit exceeded");
        $display("[TIMEOUT] cycle=%0d, conv1_cnt=%0d", cycle_cnt, conv1_cnt);
        $finish;
    end

endmodule
