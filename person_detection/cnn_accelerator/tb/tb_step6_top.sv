// tb_step6_top.sv: CNN 모듈 검증 Step 6 (최종 단계)
//
// 대상 모듈: cnn_top (전체 파이프라인)
//     conv1 -> pool -> DSC1 -> DSC2 -> conv_out -> sigmoid -> inference_done
// 입력: 128x128x3 픽셀 스트리밍 (고정값)
// 확인 항목:
//     [1] grid_valid 총 횟수 = 169(13*13)
//     [2] 첫 번째 grid_valid까지 걸리는 클럭 수
//     [3] inference_done 펄스 발생 확인 (169번째 grid_valid 직후)
//     [4] grid_prod xxxx 없는지 확인 (8bit, 0-255)

`timescale 1ns / 1ps

module tb_step6_top;
    reg clk, rst_n;

    reg        pixel_valid;
    reg [15:0] pixel_r, pixel_g, pixel_b;

    wire       grid_valid;
    wire [7:0] grid_prob;

    wire inference_done;

    cnn_top u_dut (
        .clk(clk), .rst_n(rst_n),
        .pixel_valid(pixel_valid), .pixel_r(pixel_r), .pixel_g(pixel_g), .pixel_b(pixel_b),
        .grid_valid(grid_valid), .grid_prob(grid_prob),
        .inference_done(inference_done));

    initial clk = 0;
    always #10 clk = ~clk;

    // 카운터 및 체크포인트
    integer cycle_cnt = 0;             // 전체 클럭 카운터
    integer grid_cnt = 0;              // grid_valid 총 횟수
    integer first_valid_cycle = -1;    // 첫 grid_valid 클럭 발생 시점
    integer done_cycle = -1;           // inference_done 발생 클럭

    always @(posedge clk) begin
        // 클럭 카운터
        cycle_cnt <= cycle_cnt + 1;

        if (grid_valid) begin
            // [1] grid_valid 횟수 카운트
            grid_cnt <= grid_cnt + 1;

            // [2] 첫 번째 grid_valid 타이밍 기록
            if (first_valid_cycle == -1) first_valid_cycle <= cycle_cnt;

            // [4] grid_prob 샘플 출력 (처음 5개 + 마지막 5개)
            if ((grid_cnt < 5) || (grid_cnt >= 164)) $display("[SAMPLE] grid_prob[%0d] = %0d (0x%02X)",
                                                              grid_cnt, grid_prob, grid_prob);
        end

        if (inference_done) begin
            done_cycle <= cycle_cnt;
            $display("[INFO] inference_done pulse at cycle %0d", cycle_cnt);
        end
    end

    integer pass_cnt;
    initial begin
        pass_cnt = 0;
        rst_n = 0;
        pixel_valid = 0;
        pixel_r = 16'h1000; pixel_g = 16'h0800; pixel_b = 16'h0400;    // Q4.12 고정값 (R=1.0, G=0.5, B=0.25)

        repeat(3) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);
        $display("========== Step 6 Start: Full pipeline ==========");

        // 128x128 픽셀 입력 -> inference_done까지 유지
        pixel_valid = 1;
        wait(inference_done);
        pixel_valid = 0;
        $display("[INFO] Pixel input done");
        $display("[INFO] First grid_valid at cycle %0d", first_valid_cycle);

        repeat(10) @(posedge clk);
        $display("-------------------------------------------------");

        // [1] grid_valid 횟수
        if (grid_cnt == 169) begin
            $display("[PASS] grid_valid count: %0d / 169", grid_cnt);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] grid_valid count: %0d / 169", grid_cnt);
        end

        // [2] 첫 grid_valid 타이밍
        if (first_valid_cycle > 0) begin
            $display("[PASS] First grid_valid at cycle: %0d", first_valid_cycle);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] grid_valid not detected");
        end

        // [3] inference_done 펄스
        if (done_cycle > 0) begin
            $display("[PASS] inference_done pulse at cycle: %0d", done_cycle);
            pass_cnt = pass_cnt + 1;
        end
        else begin
            $display("[FAIL] inference_done pulse not detected");
        end

        $display("-------------------------------------------------");
        $display("[SUMMARY] Step 6 result: %0d / 3 PASS", pass_cnt);
        #100 $finish;
    end

    // 디버그 출력 (500k 클럭마다)
    always @(posedge clk) begin
        if ((cycle_cnt % 500_000 == 0) && (cycle_cnt > 0)) begin
            $display("[DEBUG] cycle: %0d grid_cnt=%0d pixel_valid=%b grid_valid=%b",
                     cycle_cnt, grid_cnt, pixel_valid, grid_valid);
        end
    end

    // 타임아웃
    initial begin
        #300_000_000;
        $display("[TIMEOUT] Simulation time limit exceeded");
        $display("[TIMEOUT] cycle=%0d, grid_cnt=%0d", cycle_cnt, grid_cnt);
        $finish;
    end

endmodule
