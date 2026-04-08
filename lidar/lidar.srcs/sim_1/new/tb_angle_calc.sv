// ============================================================
// tb_angle_calc.sv
// angle_calc 각도 계산 검증 테스트벤치
//
// 검증 항목:
//   1. FSA = LSA (LSN=1, 시작패킷) → angle = FSA
//   2. 선형 보간 정확도 (여러 각도 조합)
//   3. Wrap-around (359° → 5° 등)
//   4. fsa_lsa_valid → S1 사이 타이밍 (CS 2바이트 딜레이)
//   5. 연속 패킷 (이전 패킷 accum 이 다음 패킷에 영향 없는지)
// ============================================================
`timescale 1ns / 1ps

module tb_angle_calc;

    localparam CLK_PERIOD = 8;  // 125MHz

    logic clk, rst_n;
    logic [15:0] fsa_raw, lsa_raw;
    logic [7:0] lsn;
    logic       fsa_lsa_valid;
    logic       si_valid;
    logic       pkt_start;
    logic [8:0] angle_deg;
    logic       angle_valid;

    // ============================================================
    // Scoreboard
    // ============================================================
    integer total_checks, pass_count, fail_count;

    task sb_init;
        total_checks = 0;
        pass_count   = 0;
        fail_count   = 0;
    endtask

    task sb_check(input integer test_num, input string desc,
                  input integer actual, input integer expected,
                  input integer tolerance  // 허용 오차 (정수 각도)
    );
        integer diff;
        total_checks++;
        diff = actual - expected;
        if (diff < 0) diff = -diff;
        if (diff <= tolerance) begin
            pass_count++;
            $display("  [PASS] T%0d | %s | angle=%0d (exp=%0d ±%0d)",
                     test_num, desc, actual, expected, tolerance);
        end else begin
            fail_count++;
            $display("  [FAIL] T%0d | %s | angle=%0d (exp=%0d ±%0d) <<<",
                     test_num, desc, actual, expected, tolerance);
        end
    endtask

    task sb_report;
        $display("========================================");
        $display("  SCOREBOARD: Total=%0d PASS=%0d FAIL=%0d", total_checks,
                 pass_count, fail_count);
        if (fail_count == 0) $display("  Result: ALL PASS");
        else $display("  Result: %0d FAILED", fail_count);
        $display("========================================");
    endtask

    // ============================================================
    // DUT
    // ============================================================
    angle_calc dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .fsa_raw      (fsa_raw),
        .lsa_raw      (lsa_raw),
        .lsn          (lsn),
        .fsa_lsa_valid(fsa_lsa_valid),
        .si_valid     (si_valid),
        .pkt_start    (pkt_start),
        .angle_deg    (angle_deg),
        .angle_valid  (angle_valid)
    );

    always #(CLK_PERIOD / 2) clk = ~clk;

    // ============================================================
    // 변환 함수
    // ============================================================

    // 각도 → FSA raw (angle * 128 + 1)
    function automatic logic [15:0] to_fsa(input integer ang);
        to_fsa = (ang[8:0] << 7) | 16'h0001;
    endfunction

    // FSA raw → 각도 역산 (DUT 와 동일)
    function automatic integer fsa_to_angle(input logic [15:0] fsa);
        fsa_to_angle = fsa[15:7];
    endfunction

    // 선형 보간 기대값 (float 근사)
    function automatic integer expected_angle(
        input integer fsa_ang, input integer lsa_ang, input integer lsn_val,
        input integer idx       // 0-based
    );
        integer diff, wrapped;
        // diff 시계방향
        if (lsa_ang >= fsa_ang) diff = lsa_ang - fsa_ang;
        else diff = 360 - fsa_ang + lsa_ang;

        if (lsn_val == 1) expected_angle = fsa_ang;
        else expected_angle = fsa_ang + (diff * idx) / (lsn_val - 1);

        // 360 wrap
        if (expected_angle >= 360) expected_angle = expected_angle - 360;
    endfunction

    // ============================================================
    // 패킷 시뮬레이션 task
    // Si 전송하면서 angle_valid 를 감시해 captured_angles 에 직접 저장
    // ============================================================
    task send_packet(input integer fsa_ang, input integer lsa_ang,
                     input integer lsn_val);
        // pkt_start
        @(posedge clk);
        pkt_start     = 1'b1;
        fsa_lsa_valid = 1'b0;
        si_valid      = 1'b0;
        @(posedge clk);
        pkt_start = 1'b0;

        // FSA/LSA 세팅
        fsa_raw   = to_fsa(fsa_ang);
        lsa_raw   = to_fsa(lsa_ang);
        lsn       = lsn_val[7:0];

        // fsa_lsa_valid 펄스
        @(posedge clk);
        fsa_lsa_valid = 1'b1;
        @(posedge clk);
        fsa_lsa_valid = 1'b0;

        // CS 2바이트 딜레이
        repeat (2) @(posedge clk);

        // Si N개 전송 + angle_valid 수집
        for (int i = 0; i < lsn_val; i++) begin
            @(posedge clk);
            si_valid = 1'b1;
            @(posedge clk);
            si_valid = 1'b0;
            // angle_valid 최대 3클럭 대기
            repeat (3) begin
                @(posedge clk);
                if (angle_valid) begin
                    captured_angles[angle_cnt] = angle_deg;
                    angle_cnt = angle_cnt + 1;
                end
            end
        end
        repeat (3) @(posedge clk);
    endtask

    // ============================================================
    // 결과 수집
    // captured_angles 는 angle_valid 펄스마다 자동 저장
    // angle_cnt 는 initial 블록에서만 관리 (충돌 방지)
    // ============================================================
    logic   [8:0] captured_angles[0:63];
    integer       angle_cnt;

    // angle_valid 시 captured_angles 저장 (always_ff, angle_cnt 사용 안함)
    // → initial 블록에서 wait(angle_valid) 로 직접 수집

    // ============================================================
    // Test
    // ============================================================
    integer       test_num;
    integer fsa_a, lsa_a, lsn_v;
    integer exp_ang;

    initial begin
        clk           = 0;
        rst_n         = 0;
        fsa_raw       = 0;
        lsa_raw       = 0;
        lsn           = 1;
        fsa_lsa_valid = 0;
        si_valid      = 0;
        pkt_start     = 0;
        test_num      = 0;

        sb_init();

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        $display("========================================");
        $display("  angle_calc testbench");
        $display("========================================");

        // ------------------------------------------------
        // Test 1: LSN=1 시작패킷 → FSA 그대로
        // ------------------------------------------------
        test_num++;
        fsa_a = 90;
        lsa_a = 90;
        lsn_v = 1;
        $display("--- T%0d: LSN=1 FSA=90° ---", test_num);

        send_packet(fsa_a, lsa_a, lsn_v);
        repeat (5) @(posedge clk);

        sb_check(test_num, "LSN=1 angle=90", captured_angles[0], fsa_to_angle(
                 to_fsa(fsa_a)), 1);

        // ------------------------------------------------
        // Test 2: LSN=2 → FSA, LSA 두 포인트
        // ------------------------------------------------
        test_num++;
        fsa_a = 10;
        lsa_a = 20;
        lsn_v = 2;
        $display("--- T%0d: LSN=2 FSA=10 LSA=20 ---", test_num);

        send_packet(fsa_a, lsa_a, lsn_v);
        repeat (5) @(posedge clk);

        sb_check(test_num, "LSN=2 S1=10", captured_angles[0], fsa_to_angle(
                 to_fsa(10)), 1);
        sb_check(test_num, "LSN=2 S2=20", captured_angles[1], fsa_to_angle(
                 to_fsa(20)), 1);

        // ------------------------------------------------
        // Test 3: LSN=5 선형 보간
        // FSA=0 LSA=40 → 0, 10, 20, 30, 40
        // ------------------------------------------------
        test_num++;
        fsa_a = 0;
        lsa_a = 40;
        lsn_v = 5;
        $display("--- T%0d: LSN=5 FSA=0 LSA=40 (step=10) ---", test_num);

        send_packet(fsa_a, lsa_a, lsn_v);
        repeat (5) @(posedge clk);

        for (int i = 0; i < lsn_v; i++) begin
            exp_ang = expected_angle(
                fsa_to_angle(
                    to_fsa(fsa_a)
                ),
                fsa_to_angle(
                    to_fsa(lsa_a)
                ),
                lsn_v,
                i
            );
            sb_check(test_num, $sformatf("LSN=5 S%0d", i + 1),
                     captured_angles[i], exp_ang, 1);
        end

        // ------------------------------------------------
        // Test 4: LSN=40 (실제 X4PRO 패킷)
        // FSA=200 LSA=220 → 0.513도 step
        // ------------------------------------------------
        test_num++;
        fsa_a = 200;
        lsa_a = 220;
        lsn_v = 40;
        $display("--- T%0d: LSN=40 FSA=200 LSA=220 ---", test_num);

        send_packet(fsa_a, lsa_a, lsn_v);
        repeat (5) @(posedge clk);

        // 첫번째, 중간, 마지막만 확인
        sb_check(test_num, "LSN=40 S1", captured_angles[0], fsa_to_angle(
                 to_fsa(200)), 1);
        sb_check(test_num, "LSN=40 S20", captured_angles[19], expected_angle(
                 fsa_to_angle(to_fsa(200)), fsa_to_angle(to_fsa(220)), 40, 19),
                 1);
        sb_check(test_num, "LSN=40 S40", captured_angles[39], fsa_to_angle(
                 to_fsa(220)), 1);

        // ------------------------------------------------
        // Test 5: Wrap-around 350° → 10° (시계방향 20도)
        // ------------------------------------------------
        test_num++;
        fsa_a = 350;
        lsa_a = 10;
        lsn_v = 5;
        $display("--- T%0d: Wrap 350->10 LSN=5 ---", test_num);

        send_packet(fsa_a, lsa_a, lsn_v);
        repeat (5) @(posedge clk);

        // 기대값: 350, 355, 0, 5, 10
        sb_check(test_num, "Wrap S1=350", captured_angles[0], fsa_to_angle(
                 to_fsa(350)), 1);
        sb_check(test_num, "Wrap S3=0", captured_angles[2], 0, 1);
        sb_check(test_num, "Wrap S5=10", captured_angles[4], fsa_to_angle(
                 to_fsa(10)), 1);

        // ------------------------------------------------
        // Test 6: 연속 패킷 (이전 accum 영향 없는지)
        // ------------------------------------------------
        test_num++;
        $display("--- T%0d: 연속 패킷 독립성 ---", test_num);

        // 첫 패킷
        send_packet(100, 120, 5);
        repeat (3) @(posedge clk);

        // 두번째 패킷 (완전히 다른 각도)
        send_packet(50, 70, 5);
        repeat (5) @(posedge clk);

        sb_check(test_num, "2nd pkt S1=50", captured_angles[0], fsa_to_angle(
                 to_fsa(50)), 1);
        sb_check(test_num, "2nd pkt S5=70", captured_angles[4], fsa_to_angle(
                 to_fsa(70)), 1);

        // ------------------------------------------------
        // Test 7: FSA = LSA (diff=0) → 모든 포인트 동일 각도
        // ------------------------------------------------
        test_num++;
        fsa_a = 180;
        lsa_a = 180;
        lsn_v = 4;
        $display("--- T%0d: FSA=LSA=180 diff=0 ---", test_num);

        send_packet(fsa_a, lsa_a, lsn_v);
        repeat (5) @(posedge clk);

        for (int i = 0; i < lsn_v; i++) begin
            sb_check(test_num, $sformatf("diff=0 S%0d=180", i + 1),
                     captured_angles[i], fsa_to_angle(to_fsa(180)), 1);
        end

        // ------------------------------------------------
        // Test 8: fsa_lsa_valid 타이밍 (CS 딜레이 없이 보내면?)
        // S1 이 너무 빨리 오는 경우 → 틀린 값 나와야 함
        // ------------------------------------------------
        test_num++;
        $display("--- T%0d: fsa_lsa_valid 직후 바로 si_valid ---",
                 test_num);

        @(posedge clk);
        pkt_start = 1'b1;
        @(posedge clk);
        pkt_start = 1'b0;

        fsa_raw   = to_fsa(30);
        lsa_raw   = to_fsa(50);
        lsn       = 8'd3;

        @(posedge clk);
        fsa_lsa_valid = 1'b1;
        @(posedge clk);
        fsa_lsa_valid = 1'b0;

        // CS 딜레이 없이 바로 si_valid (타이밍 위반 케이스)
        @(posedge clk);
        si_valid = 1'b1;
        @(posedge clk);
        si_valid = 1'b0;

        repeat (5) @(posedge clk);
        $display(
            "  [INFO] T%0d: early si_valid -> angle=%0d (step 미계산, 이전값 사용)",
            test_num, captured_angles[0]);

        sb_report();
        $finish;
    end

    // 파형 덤프 (Vivado 시뮬레이션용)
    initial begin
        $dumpfile("tb_angle_calc.vcd");
        $dumpvars(0, tb_angle_calc);
    end

endmodule
