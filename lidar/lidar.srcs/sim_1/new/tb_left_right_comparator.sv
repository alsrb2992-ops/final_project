`timescale 1ns / 1ps
`include "lidar_define.vh"

module tb_left_right_comparator ();

    // ===== DUT 신호 =====
    reg         clk;
    reg         rst_n;
    reg  [13:0] left_min_distance;
    reg  [13:0] right_min_distance;
    reg         warning_signal;

    wire [ 2:0] direction_degree;

    // ===== DUT 인스턴스 =====
    // tick 이 빠르게 생성되도록 DIR_CHANGE_FREQUENCY 를 크게 설정
    left_right_comparator #(
        .CLK_FREQ            (125_000_000),
        .DIR_CHANGE_FREQUENCY(12_500_000),   // 10 클럭마다 tick
        .TURN_THRESHOLD_MM   (14'd800),
        .BIG_TURN_DIFF_MM    (14'd100)
    ) dut (
        .clk               (clk),
        .rst_n             (rst_n),
        .left_min_distance (left_min_distance),
        .right_min_distance(right_min_distance),
        .warning_signal    (warning_signal),
        .direction_degree  (direction_degree)
    );

    // ===== 클럭 생성 (125MHz) =====
    initial clk = 0;
    always #4 clk = ~clk;

    // ===== 테스트 결과 카운터 =====
    integer pass_count = 0;
    integer fail_count = 0;
    integer test_num = 0;

    // ===== tick 대기 태스크 =====
    // DIR_CHANGE_COUNT = 125M / 12.5M = 10 클럭
    task wait_tick;
        begin
            repeat (12) @(posedge clk);  // tick + 안정화 여유
        end
    endtask

    // ===== 결과 검증 태스크 =====
    task check_result;
        input [119:0] test_name;  // 15 chars
        input [2:0] expected_dir;
        reg [31:0] exp_str;
        begin
            test_num = test_num + 1;

            @(posedge clk);

            if (direction_degree == expected_dir) begin
                $display("[PASS] Test %0d: %s", test_num, test_name);
                $display(
                    "       direction=%0d (exp=%0d) | left=%0d, right=%0d, warn=%b",
                    direction_degree, expected_dir, left_min_distance,
                    right_min_distance, warning_signal);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Test %0d: %s", test_num, test_name);
                $display(
                    "       direction=%0d (exp=%0d) | left=%0d, right=%0d, warn=%b",
                    direction_degree, expected_dir, left_min_distance,
                    right_min_distance, warning_signal);
                fail_count = fail_count + 1;
            end
            $display("");
        end
    endtask

    // define 값 출력용 함수
    function [23:0] dir_name;
        input [2:0] dir;
        begin
            case (dir)
                `CENTER:           dir_name = "CTR";
                `TURN_RIGHT_BIG:   dir_name = "RBG";
                `TURN_RIGHT_SMALL: dir_name = "RSM";
                `TURN_LEFT_BIG:    dir_name = "LBG";
                `TURN_LEFT_SMALL:  dir_name = "LSM";
                default:           dir_name = "???";
            endcase
        end
    endfunction

    // ===== 메인 테스트 =====
    initial begin
        $display("==================================================");
        $display(" left_right_comparator Testbench");
        $display("==================================================");
        $display(" TURN_THRESHOLD_MM = 800mm");
        $display(" BIG_TURN_DIFF_MM  = 100mm");
        $display(" DIR_CHANGE_FREQ   = 12.5MHz (10 clk tick)");
        $display(" CENTER=%0d, R_BIG=%0d, R_SML=%0d, L_BIG=%0d, L_SML=%0d",
                 `CENTER, `TURN_RIGHT_BIG, `TURN_RIGHT_SMALL, `TURN_LEFT_BIG,
                 `TURN_LEFT_SMALL);
        $display("==================================================\n");

        // 초기화
        rst_n              = 1'b0;
        left_min_distance  = 14'h3FFF;
        right_min_distance = 14'h3FFF;
        warning_signal     = 1'b0;
        #100;
        rst_n = 1'b1;
        #100;

        // ============================================================
        // [그룹 1] 직진 케이스
        // ============================================================
        $display("========== Group 1: 직진 케이스 ==========\n");

        // Test 1: 양쪽 다 충분히 멀리 → CENTER
        $display(
            "--- Case 1: 양쪽 충분히 멀리 (left=1000, right=1000) ---");
        left_min_distance  = 14'd1000;
        right_min_distance = 14'd1000;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("BothFar_CTR    ", `CENTER);

        // Test 2: 양쪽 다 threshold 정확히 초과 → CENTER
        $display(
            "--- Case 2: 양쪽 threshold 초과 (left=801, right=801) ---");
        left_min_distance  = 14'd801;
        right_min_distance = 14'd801;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("ThreshExact_CTR", `CENTER);

        // Test 3: 양쪽 같고 threshold 이하지만 차이 없음 → CENTER
        $display("--- Case 3: 양쪽 같음 (left=500, right=500) ---");
        left_min_distance  = 14'd500;
        right_min_distance = 14'd500;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("BothEqual_CTR  ", `CENTER);

        // ============================================================
        // [그룹 2] warning_signal=0, 방향 판단 케이스
        // ============================================================
        $display(
            "========== Group 2: 방향 판단 케이스 (warning=0) ==========\n");

        // Test 4: 왼쪽이 훨씬 가까움 (차이 > 100) → TURN_RIGHT_BIG
        $display(
            "--- Case 4: 왼쪽 훨씬 가까움 (left=200, right=500) diff=300 ---");
        left_min_distance  = 14'd200;
        right_min_distance = 14'd500;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("LClose_RBig    ", `TURN_RIGHT_BIG);

        // Test 5: 왼쪽이 조금 가까움 (차이 <= 100) → TURN_RIGHT_SMALL
        $display(
            "--- Case 5: 왼쪽 조금 가까움 (left=300, right=380) diff=80 ---");
        left_min_distance  = 14'd300;
        right_min_distance = 14'd380;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("LClose_RSmall  ", `TURN_RIGHT_SMALL);

        // Test 6: 오른쪽이 훨씬 가까움 (차이 > 100) → TURN_LEFT_BIG
        $display(
            "--- Case 6: 오른쪽 훨씬 가까움 (left=500, right=200) diff=300 ---");
        left_min_distance  = 14'd500;
        right_min_distance = 14'd200;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("RClose_LBig    ", `TURN_LEFT_BIG);

        // Test 7: 오른쪽이 조금 가까움 (차이 <= 100) → TURN_LEFT_SMALL
        $display(
            "--- Case 7: 오른쪽 조금 가까움 (left=380, right=300) diff=80 ---");
        left_min_distance  = 14'd380;
        right_min_distance = 14'd300;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("RClose_LSmall  ", `TURN_LEFT_SMALL);

        // Test 8: 차이가 정확히 BIG_TURN_DIFF_MM (100) → SMALL (초과가 아니므로)
        $display(
            "--- Case 8: 차이 정확히 100 (left=200, right=300) diff=100 ---");
        left_min_distance  = 14'd200;
        right_min_distance = 14'd300;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("DiffExact100   ", `TURN_RIGHT_SMALL);

        // Test 9: 차이가 101 → BIG
        $display("--- Case 9: 차이 101 (left=200, right=301) diff=101 ---");
        left_min_distance  = 14'd200;
        right_min_distance = 14'd301;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("DiffOver101    ", `TURN_RIGHT_BIG);

        // Test 10: 한쪽만 threshold 이하 (left=700, right=1000) → 왼쪽이 가까우니 오른쪽으로
        $display(
            "--- Case 10: 한쪽만 threshold 이하 (left=700, right=1000) ---");
        left_min_distance  = 14'd700;
        right_min_distance = 14'd1000;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("OneUnder_RBIG  ", `TURN_RIGHT_BIG);

        // ============================================================
        // [그룹 3] warning_signal=1 케이스 (긴급 회피)
        // ============================================================
        $display("========== Group 3: warning=1 긴급 케이스 ==========\n");

        // Test 11: warning=1, 왼쪽이 더 가까움 → TURN_RIGHT_BIG (무조건 BIG)
        $display(
            "--- Case 11: warning=1, 왼쪽 가까움 (left=100, right=500) ---");
        left_min_distance  = 14'd100;
        right_min_distance = 14'd500;
        warning_signal     = 1'b1;
        wait_tick();
        check_result("Warn_LClose_RBG", `TURN_RIGHT_BIG);

        // Test 12: warning=1, 오른쪽이 더 가까움 → TURN_LEFT_BIG (무조건 BIG)
        $display(
            "--- Case 12: warning=1, 오른쪽 가까움 (left=500, right=100) ---");
        left_min_distance  = 14'd500;
        right_min_distance = 14'd100;
        warning_signal     = 1'b1;
        wait_tick();
        check_result("Warn_RClose_LBG", `TURN_LEFT_BIG);

        // Test 13: warning=1, 차이 작아도 무조건 BIG (차이=50)
        $display(
            "--- Case 13: warning=1, 작은 차이도 BIG (left=250, right=300) ---");
        left_min_distance  = 14'd250;
        right_min_distance = 14'd300;
        warning_signal     = 1'b1;
        wait_tick();
        check_result("Warn_SmDif_RBG ", `TURN_RIGHT_BIG);

        // Test 14: warning=1, 양쪽 같음 → TURN_LEFT_BIG (else 분기)
        $display(
            "--- Case 14: warning=1, 양쪽 같음 (left=300, right=300) ---");
        left_min_distance  = 14'd300;
        right_min_distance = 14'd300;
        warning_signal     = 1'b1;
        wait_tick();
        check_result("Warn_Equal_LBG ", `TURN_LEFT_BIG);

        // ============================================================
        // [그룹 4] warning 전환 케이스
        // ============================================================
        $display("========== Group 4: warning 전환 케이스 ==========\n");

        // Test 15: warning ON → OFF 후 정상 판단으로 복귀
        $display(
            "--- Case 15: warning OFF 후 복귀 (left=1000, right=1000) ---");
        left_min_distance  = 14'd1000;
        right_min_distance = 14'd1000;
        warning_signal     = 1'b0;
        wait_tick();
        check_result("WarnOff_CTR    ", `CENTER);

        // Test 16: warning OFF → ON 즉시 반응
        $display(
            "--- Case 16: warning OFF→ON 즉시 반응 (left=200, right=800) ---");
        left_min_distance  = 14'd200;
        right_min_distance = 14'd800;
        warning_signal     = 1'b0;
        wait_tick();
        $display("  [Before warn] direction=%0d (%s)", direction_degree,
                 dir_name(direction_degree));
        warning_signal = 1'b1;
        wait_tick();
        check_result("WarnOn_RBIG    ", `TURN_RIGHT_BIG);

        // ============================================================
        // [그룹 5] tick 동작 확인
        // ============================================================
        $display("========== Group 5: tick 동작 확인 ==========\n");

        // Test 17: tick 전에는 방향이 바뀌지 않아야 함
        $display("--- Case 17: tick 전 방향 유지 확인 ---");
        begin
            reg [2:0] before_dir;
            // 현재 방향 저장
            before_dir         = direction_degree;

            // 입력 바꾸고 tick 전에 확인
            left_min_distance  = 14'd100;
            right_min_distance = 14'd900;
            warning_signal     = 1'b0;
            @(posedge clk);  // tick 기다리지 않고 바로 확인

            // tick 이전이므로 이전 방향 유지
            test_num = test_num + 1;
            if (direction_degree == before_dir) begin
                $display(
                    "[PASS] Test %0d: TickHold - direction still %0d before tick",
                    test_num, direction_degree);
                pass_count = pass_count + 1;
            end else begin
                $display(
                    "[FAIL] Test %0d: TickHold - direction changed to %0d without tick!",
                    test_num, direction_degree);
                fail_count = fail_count + 1;
            end
            $display("");

            // tick 후에는 바뀌어야 함
            wait_tick();
            check_result("AfterTick_RBIG ", `TURN_RIGHT_BIG);
        end

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
