// ============================================================
// tb_collision_detector_random.sv
// collision_detector + round_detector random testbench
// 1000 test with scoreboard + reference model
// each test: randomly choose between
//   - pure random points
//   - warn->brake transition scenario
// ============================================================
`timescale 1ns / 1ps

module tb_collision_detector;

    localparam CLK_PERIOD = 20;
    localparam NUM_TESTS = 1000;
    localparam FRONT_ANGLE_DEG = 9'd20;
    localparam BRAKE_DIST_MM = 14'd500;
    localparam WARN_DIST_MM = BRAKE_DIST_MM * 2;  // 1000mm

    logic          clk;
    logic          rst_n;
    logic   [13:0] distance;
    logic   [ 8:0] angle;
    logic          data_valid;
    logic          pkt_start;
    logic          ct_start_bit;
    logic          brake_signal;
    logic          warning_signal;
    logic          round_done;

    // ============================================================
    // Scoreboard
    // ============================================================
    integer        total_tests;
    integer        pass_count;
    integer        fail_count;
    integer        scenario_count;  // warn->brake 시나리오 실행 횟수

    task sb_init;
        total_tests    = 0;
        pass_count     = 0;
        fail_count     = 0;
        scenario_count = 0;
    endtask

    task sb_check(input integer test_num, input string test_desc,
                  input logic actual, input logic expected,
                  input string signal_name);
        total_tests = total_tests + 1;
        if (actual === expected) begin
            pass_count = pass_count + 1;
        end else begin
            fail_count = fail_count + 1;
            $display("[FAIL] Test%0d (%s) | %s = %b (expected: %b)", test_num,
                     test_desc, signal_name, actual, expected);
        end
    endtask

    task sb_report;
        $display("========================================");
        $display(" SCOREBOARD REPORT");
        $display("  Total checks   : %0d", total_tests);
        $display("  PASS           : %0d", pass_count);
        $display("  FAIL           : %0d", fail_count);
        $display("  Warn->Brake    : %0d times", scenario_count);
        if (fail_count == 0) $display("  Result: ALL PASS");
        else $display("  Result: %0d FAILED", fail_count);
        $display("========================================");
    endtask

    // ============================================================
    // Reference Model
    // ============================================================
    logic ref_brake;
    logic ref_warning;
    logic ref_danger_in_round;
    logic ref_warn_in_round;

    task ref_init;
        ref_brake           = 1'b0;
        ref_warning         = 1'b0;
        ref_danger_in_round = 1'b0;
        ref_warn_in_round   = 1'b0;
    endtask

    task ref_send_point(input logic [8:0] ang_in, input logic [13:0] dist_in);
        logic in_front;
        in_front = (ang_in <= FRONT_ANGLE_DEG) ||
                   (ang_in >= (9'd360 - FRONT_ANGLE_DEG));

        if (in_front) begin
            if (dist_in <= BRAKE_DIST_MM && dist_in != 14'd0) begin
                ref_brake           = 1'b1;
                ref_warning         = 1'b1;
                ref_danger_in_round = 1'b1;
                ref_warn_in_round   = 1'b1;
            end else if (dist_in <= WARN_DIST_MM && dist_in != 14'd0) begin
                ref_warning       = 1'b1;
                ref_warn_in_round = 1'b1;
            end
        end
    endtask

    task ref_round_done;
        if (!ref_danger_in_round) ref_brake = 1'b0;
        if (!ref_warn_in_round) ref_warning = 1'b0;
        ref_danger_in_round = 1'b0;
        ref_warn_in_round   = 1'b0;
    endtask

    // ============================================================
    // DUT
    // ============================================================
    round_detector u_round (
        .pkt_start   (pkt_start),
        .ct_start_bit(ct_start_bit),
        .round_done  (round_done)
    );

    collision_detector #(
        .FRONT_ANGLE_DEG(FRONT_ANGLE_DEG),
        .BRAKE_DIST_MM  (BRAKE_DIST_MM)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .distance      (distance),
        .angle         (angle),
        .data_valid    (data_valid),
        .round_done    (round_done),
        .brake_signal  (brake_signal),
        .warning_signal(warning_signal)
    );

    always #(CLK_PERIOD / 2) clk = ~clk;

    // ============================================================
    // Tasks
    // ============================================================
    task send_point(input logic [8:0] ang_in, input logic [13:0] dist_in);
        @(negedge clk);
        angle      = ang_in;
        distance   = dist_in;
        data_valid = 1'b1;
        @(posedge clk);
        #1;
        data_valid = 1'b0;
    endtask

    task send_round_done;
        @(posedge clk);
        pkt_start    = 1'b1;
        ct_start_bit = 1'b1;
        @(posedge clk);
        pkt_start    = 1'b0;
        ct_start_bit = 1'b0;
        @(posedge clk);
    endtask

    // ----------------------------------------------------------
    // 포인트 1개 전송 + 레퍼런스 동기화 + 즉시 체크
    // ----------------------------------------------------------
    task do_point(input integer t_num, input logic [8:0] ang_in,
                  input logic [13:0] dist_in);
        string desc;
        ref_send_point(ang_in, dist_in);
        send_point(ang_in, dist_in);
        @(posedge clk);
        $sformat(desc, "T%0d ang=%0d dist=%0d", t_num, ang_in, dist_in);
        sb_check(t_num, desc, brake_signal, ref_brake, "brake  ");
        sb_check(t_num, desc, warning_signal, ref_warning, "warning");
    endtask

    // ----------------------------------------------------------
    // round_done 전송 + 레퍼런스 동기화 + 해제 체크
    // ----------------------------------------------------------
    task do_round_done(input integer t_num, input string phase);
        string desc;
        ref_round_done();
        send_round_done();
        $sformat(desc, "T%0d %s after_round", t_num, phase);
        sb_check(t_num, desc, brake_signal, ref_brake, "brake  ");
        sb_check(t_num, desc, warning_signal, ref_warning, "warning");
    endtask

    // ============================================================
    // Variables
    // ============================================================
    integer        j;
    integer        test_num;
    integer        num_points;
    logic   [ 8:0] rand_angle;
    logic   [13:0] rand_dist;
    logic   [13:0] warn_dist;
    logic   [13:0] brake_dist;
    integer        scenario_type;  // 0=random, 1=warn->brake

    // ============================================================
    // Test
    // ============================================================
    initial begin
        clk          = 0;
        rst_n        = 0;
        distance     = 0;
        angle        = 0;
        data_valid   = 0;
        pkt_start    = 0;
        ct_start_bit = 0;

        sb_init();
        ref_init();

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        $display("========================================");
        $display(" Mixed Random Test: %0d rounds", NUM_TESTS);
        $display("  FRONT_ANGLE_DEG = %0d", FRONT_ANGLE_DEG);
        $display("  BRAKE_DIST_MM   = %0d", BRAKE_DIST_MM);
        $display("  WARN_DIST_MM    = %0d", WARN_DIST_MM);
        $display("  Scenario mix: ~30%% warn->brake");
        $display("========================================");

        for (test_num = 1; test_num <= NUM_TESTS; test_num++) begin

            // 30% 확률로 warn->brake 시나리오 선택
            scenario_type = ($urandom % 10 < 3) ? 1 : 0;

            if (scenario_type == 1) begin
                // ============================================
                // Scenario: warning -> brake
                // 회전1: 전방 warn 거리 → warning=1 brake=0
                //        round_done 1번째
                // 회전2: warn 포인트 먼저 → warning=1 brake=0
                //        brake 포인트 수신 → brake=1 즉시
                //        (round_done 2번째 오기 전!)
                //        round_done 2번째 → brake 유지
                // ============================================
                scenario_count = scenario_count + 1;

                // warn 거리: BRAKE+1 ~ WARN 사이 랜덤
                warn_dist  = BRAKE_DIST_MM + 1
                             + ($urandom % (WARN_DIST_MM - BRAKE_DIST_MM));
                // brake 거리: 1 ~ BRAKE 사이 랜덤
                brake_dist = 1 + ($urandom % BRAKE_DIST_MM);

                // --- 회전 1: warn 포인트만 ---
                do_point(test_num, 9'd5, warn_dist);
                do_round_done(test_num, "S_round1");

                // --- 회전 2: warn 먼저, 그 다음 brake ---
                // warn 포인트
                do_point(test_num, 9'd5, warn_dist);

                // brake 포인트 (round_done 2번째 전!)
                do_point(test_num, 9'd5, brake_dist);

                // round_done 2번째
                do_round_done(test_num, "S_round2");

                // --- 정리: 안전 거리로 해제 ---
                do_point(test_num, 9'd5, 14'd1500);
                do_round_done(test_num, "S_cleanup");

            end else begin
                // ============================================
                // Pure random test
                // ============================================
                num_points = ($urandom % 10) + 1;

                for (j = 0; j < num_points; j++) begin
                    rand_angle = $urandom % 360;
                    rand_dist  = $urandom % 2001;
                    do_point(test_num, rand_angle, rand_dist);
                end

                do_round_done(test_num, "R_round");
            end

            // 100회마다 진행상황 출력
            if (test_num % 100 == 0)
                $display(
                    "  Progress: %0d/%0d | PASS=%0d FAIL=%0d | Scenario=%0d",
                    test_num,
                    NUM_TESTS,
                    pass_count,
                    fail_count,
                    scenario_count
                );
        end

        sb_report();
        $finish;
    end

endmodule
