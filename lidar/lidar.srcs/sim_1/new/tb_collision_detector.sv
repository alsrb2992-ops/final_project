`timescale 1ns / 1ps
`include "lidar_define.vh"

module tb_collision_detector ();

    // ===== DUT 신호 =====
    reg         clk;
    reg         rst_n;
    reg  [13:0] distance;
    reg  [ 8:0] angle;
    reg         data_valid;
    reg         round_done;

    wire        brake_signal;
    wire        warning_signal;
    wire        side_warning_signal;
    wire [13:0] left_min_distance;
    wire [13:0] right_min_distance;

    // ===== DUT 인스턴스 =====
    collision_detector #(
        .FRONT_ANGLE_DEG      (9'd45),
        .RIGHT_START_ANGLE_DEG(9'd45),
        .RIGHT_END_ANGLE_DEG  (9'd90),
        .LEFT_START_ANGLE_DEG (9'd270),
        .LEFT_END_ANGLE_DEG   (9'd315),
        .BRAKE_DIST_MM        (14'd300),
        .WARN_DIST_MM         (14'd600),
        .SIDE_DIST_MM         (14'd300),
        .HYSTERESIS_MM        (14'd100)
    ) dut (
        .clk                (clk),
        .rst_n              (rst_n),
        .distance           (distance),
        .angle              (angle),
        .data_valid         (data_valid),
        .round_done         (round_done),
        .brake_signal       (brake_signal),
        .warning_signal     (warning_signal),
        .side_warning_signal(side_warning_signal),
        .left_min_distance  (left_min_distance),
        .right_min_distance (right_min_distance)
    );

    // ===== 클럭 생성 (125MHz) =====
    initial clk = 0;
    always #4 clk = ~clk;

    // ===== 테스트 결과 카운터 =====
    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num = 0;

    // ===== 헬퍼 태스크 =====

    // 한 포인트 데이터 전송
    task send_point;
        input [8:0] ang;
        input [13:0] w_dist;
        begin
            @(posedge clk);
            angle      = ang;
            distance   = w_dist;
            data_valid = 1'b1;
            @(posedge clk);
            data_valid = 1'b0;
            @(posedge clk);
        end
    endtask

    // round_done 펄스 발생
    task do_round_done;
        begin
            @(posedge clk);
            round_done = 1'b1;
            @(posedge clk);
            round_done = 1'b0;
            repeat (3) @(posedge clk);
        end
    endtask

    // 1회전 시뮬레이션 (왼쪽/오른쪽 거리 지정)
    task simulate_one_round;
        input [13:0] left_dist;
        input [13:0] right_dist;
        begin
            // 오른쪽 영역 (45~90도)
            send_point(9'd50, right_dist);
            send_point(9'd60, right_dist + 14'd20);
            send_point(9'd70, right_dist + 14'd10);
            send_point(9'd80, right_dist + 14'd30);

            // 전방/후방 (측면과 무관한 영역)
            send_point(9'd0, 14'd2000);
            send_point(9'd180, 14'd2000);

            // 왼쪽 영역 (270~315도)
            send_point(9'd275, left_dist);
            send_point(9'd285, left_dist + 14'd20);
            send_point(9'd295, left_dist + 14'd10);
            send_point(9'd305, left_dist + 14'd30);

            // round_done
            do_round_done();
        end
    endtask

    // 결과 검증 (FIX: uut → dut)
    task check_result;
        input [79:0] test_name;
        input expected_left_warn;
        input expected_right_warn;
        input expected_side_warn;
        begin
            test_num = test_num + 1;

            @(posedge clk);

            if (dut.c_left_warning_signal  == expected_left_warn &&
                dut.c_right_warning_signal == expected_right_warn &&
                side_warning_signal        == expected_side_warn) begin
                $display("[PASS] Test %0d: %s", test_num, test_name);
                $display(
                    "       left_warn=%b (exp=%b), right_warn=%b (exp=%b), side_warn=%b (exp=%b)",
                    dut.c_left_warning_signal, expected_left_warn,
                    dut.c_right_warning_signal, expected_right_warn,
                    side_warning_signal, expected_side_warn);
                $display("       left_min=%0d mm, right_min=%0d mm",
                         left_min_distance, right_min_distance);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Test %0d: %s", test_num, test_name);
                $display(
                    "       left_warn=%b (exp=%b), right_warn=%b (exp=%b), side_warn=%b (exp=%b)",
                    dut.c_left_warning_signal, expected_left_warn,
                    dut.c_right_warning_signal, expected_right_warn,
                    side_warning_signal, expected_side_warn);
                $display("       left_min=%0d mm, right_min=%0d mm",
                         left_min_distance, right_min_distance);
                fail_count = fail_count + 1;
            end
            $display("");
        end
    endtask

    // ===== 메인 테스트 =====
    initial begin
        $display("==================================================");
        $display(" Collision Detector - Side Warning Testbench");
        $display("==================================================");
        $display(" SIDE_ON_THRESHOLD  = 300mm");
        $display(" SIDE_OFF_THRESHOLD = 400mm (히스테리시스)");
        $display("==================================================\n");

        // 초기화
        rst_n      = 0;
        distance   = 14'd0;
        angle      = 9'd0;
        data_valid = 1'b0;
        round_done = 1'b0;
        #100;
        rst_n = 1;
        #100;

        // ============================================================
        // Test 1: 양쪽 다 멀리 있음 → 경고 없음
        // ============================================================
        $display("--- Case 1: 양쪽 다 멀리 (left=1000, right=1000) ---");
        simulate_one_round(14'd1000, 14'd1000);
        check_result("BothFar   ", 1'b0,  // left_warn
                     1'b0,  // right_warn
                     1'b0);  // side_warn

        // ============================================================
        // Test 2: 왼쪽만 가까움 → left_warning만 ON
        // ============================================================
        $display("--- Case 2: 왼쪽만 가까움 (left=200, right=1000) ---");
        simulate_one_round(14'd200, 14'd1000);
        check_result("LeftClose ", 1'b1,  // left_warn ON
                     1'b0,  // right_warn OFF
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 3: 오른쪽만 가까움 → right_warning만 ON
        // ============================================================
        $display(
            "--- Case 3: 오른쪽만 가까움 (left=1000, right=200) ---");
        simulate_one_round(14'd1000, 14'd200);
        check_result("RightClose", 1'b0,  // left_warn OFF
                     1'b1,  // right_warn ON
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 4: 양쪽 다 가까움 (좁은 복도) → 둘 다 ON
        // ============================================================
        $display("--- Case 4: 양쪽 다 가까움 (left=200, right=250) ---");
        simulate_one_round(14'd200, 14'd250);
        check_result("BothClose ", 1'b1,  // left_warn ON
                     1'b1,  // right_warn ON
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 5: 히스테리시스 경계 ON (299mm < 300mm → ON)
        // ============================================================
        $display("--- Case 5: 히스테리시스 경계 ON (left=299) ---");
        simulate_one_round(14'd299, 14'd1000);
        check_result("HystOn299 ", 1'b1,  // left_warn ON (299 < 300)
                     1'b0,  // right_warn OFF
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 6: 히스테리시스 중간 (350mm, 이전 ON → 유지)
        // ============================================================
        $display(
            "--- Case 6: 히스테리시스 중간 (left=350, 이전 ON) ---");
        simulate_one_round(14'd350, 14'd1000);
        check_result("HystMid350", 1'b1,  // left_warn 유지 (300 < 350 < 400)
                     1'b0,  // right_warn OFF
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 7: 히스테리시스 OFF 돌파 (400mm >= 400mm → OFF)
        // ============================================================
        $display("--- Case 7: 히스테리시스 OFF 돌파 (left=400) ---");
        simulate_one_round(14'd400, 14'd1000);
        check_result("HystOff400", 1'b0,  // left_warn OFF (400 >= 400)
                     1'b0,  // right_warn OFF
                     1'b0);  // side_warn OFF

        // ============================================================
        // Test 8: OFF → 다시 가까워짐 → ON
        // ============================================================
        $display("--- Case 8: 다시 가까워짐 (left=250) ---");
        simulate_one_round(14'd250, 14'd1000);
        check_result("ReClose250", 1'b1,  // left_warn ON (250 < 300)
                     1'b0,  // right_warn OFF
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 9: 좌우 교대 전환
        // ============================================================
        $display(
            "--- Case 9: 좌우 교대 (left 가까움 → right 가까움) ---");
        simulate_one_round(14'd200, 14'd1000);
        $display("  [Setup] left=200, right=1000");
        simulate_one_round(14'd1000, 14'd200);
        check_result("Swap L->R ", 1'b0,  // left_warn OFF (1000 >= 400)
                     1'b1,  // right_warn ON (200 < 300)
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 10: 매 회전 갱신 확인 (left: 200 → 500)
        // ============================================================
        $display(
            "--- Case 10: 매 회전 갱신 (1회전: left=200 → 2회전: left=500) ---");
        simulate_one_round(14'd200, 14'd1000);
        $display("  [Round 1] left_min=%0d, left_warn=%b", left_min_distance,
                 dut.c_left_warning_signal);
        simulate_one_round(14'd500, 14'd1000);
        check_result("RoundUpd  ", 1'b0,  // left_warn OFF (500 >= 400)
                     1'b0,  // right_warn OFF
                     1'b0);  // side_warn OFF

        // ============================================================
        // Test 11: 매우 좁은 복도 (양쪽 150mm)
        // ============================================================
        $display("--- Case 11: 매우 좁은 복도 (left=150, right=150) ---");
        simulate_one_round(14'd150, 14'd150);
        check_result("Narrow150 ", 1'b1,  // left_warn ON
                     1'b1,  // right_warn ON
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 12: 비대칭 좁은 복도 (left=100, right=280)
        // ============================================================
        $display(
            "--- Case 12: 비대칭 좁은 복도 (left=100, right=280) ---");
        simulate_one_round(14'd100, 14'd280);
        check_result("Asym100280", 1'b1,  // left_warn ON (100 < 300)
                     1'b1,  // right_warn ON (280 < 300)
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 13: 한쪽만 히스테리시스, 다른쪽은 멀리
        // ============================================================
        $display(
            "--- Case 13: left=350 (히스테리시스 유지), right=2000 ---");
        simulate_one_round(14'd350, 14'd2000);
        check_result("HystLonly ",
                     1'b1,  // left_warn 유지 (300 < 350 < 400, 이전 ON)
                     1'b0,  // right_warn OFF (2000 >= 400)
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 14: 양쪽 히스테리시스 구간 (이전 ON)
        // ============================================================
        $display(
            "--- Case 14: 양쪽 히스테리시스 (left=350, right=350) ---");
        simulate_one_round(14'd200, 14'd200);
        $display("  [Setup] Both close: left=200, right=200");
        simulate_one_round(14'd350, 14'd350);
        check_result("HystBoth  ", 1'b1,  // left_warn 유지
                     1'b1,  // right_warn 유지
                     1'b1);  // side_warn ON

        // ============================================================
        // Test 15: distance=0 무시 확인
        // ============================================================
        $display("--- Case 15: distance=0 포인트 무시 ---");
        simulate_one_round(14'd1000, 14'd1000);
        send_point(9'd280, 14'd0);  // 왼쪽이지만 dist=0 → 무시
        send_point(9'd280, 14'd200);  // 유효한 값
        send_point(9'd60, 14'd0);  // 오른쪽이지만 dist=0 → 무시
        send_point(9'd60, 14'd1000);
        do_round_done();
        check_result("Dist0Skip ", 1'b1,  // left_warn ON (200 < 300)
                     1'b0,  // right_warn OFF (1000 >= 400)
                     1'b1);  // side_warn ON

        // ============================================================
        // 결과 요약
        // ============================================================
        $display("==================================================");
        $display(" Test Results: %0d PASS / %0d FAIL (Total: %0d)", pass_count,
                 fail_count, pass_count + fail_count);
        $display("==================================================");

        if (fail_count == 0) $display(" >>> ALL TESTS PASSED! <<<");
        else $display(" >>> %0d TESTS FAILED <<<", fail_count);

        $display("==================================================\n");

        #100;
        $finish;
    end

endmodule
