// ============================================================
// tb_lidar_top_random.sv
// lidar_top random testbench
// - 각도: 0~359도 랜덤
// - 거리: 120~10000mm 유효 범위 랜덤
// - scoreboard
// ============================================================
`timescale 1ns / 1ps

module tb_lidar_top;

    localparam CLK_FREQ = 125_000_000;
    localparam CLK_PERIOD = 8;
    localparam SIM_BAUD_RATE = 6_250_000;
    localparam BIT_PERIOD = CLK_FREQ / SIM_BAUD_RATE;  // 20 클럭

    localparam FRONT_ANGLE_DEG = 9'd20; // warning/brake 각도 범위 ============
    localparam BRAKE_DIST_MM = 14'd500; // 제동 거리 임계값 ===============
    localparam WARN_DIST_MM = BRAKE_DIST_MM * 2;  // 1000mm

    // 유효 거리 범위
    localparam DIST_MIN_MM = 14'd120;
    localparam DIST_MAX_MM = 14'd10000;

    localparam NUM_TESTS = 1000;  // 반복 횟수

    // 패킷 수신 + 처리 대기 클럭
    localparam PKT_CLKS = 12 * 10 * BIT_PERIOD;  // 2400 클럭
    localparam WAIT_CLKS = PKT_CLKS + 200;

    logic   clk;
    logic   rst_n;
    logic   lidar_rx;
    logic   brake_gpio;
    logic   warning_led;

    // ============================================================
    // Scoreboard
    // ============================================================
    integer total_checks;
    integer pass_count;
    integer fail_count;

    task sb_init;
        total_checks = 0;
        pass_count   = 0;
        fail_count   = 0;
    endtask

    task sb_check(input integer test_num, input string test_desc,
                  input logic actual, input logic expected,
                  input string signal_name);
        total_checks = total_checks + 1;
        if (actual === 1'bx || actual === 1'bz) begin
            fail_count = fail_count + 1;
            $display("[%0t] [UNKN] T%0d | %s | %s=x/z (exp:%b) <<<", $time,
                     test_num, test_desc, signal_name, expected);
        end else if (actual === expected) begin
            pass_count = pass_count + 1;
            $display("[%0t] [PASS] T%0d | %s | %s=%b", $time, test_num,
                     test_desc, signal_name, actual);
        end else begin
            fail_count = fail_count + 1;
            $display("[%0t] [FAIL] T%0d | %s | %s=%b (exp:%b) <<<", $time,
                     test_num, test_desc, signal_name, actual, expected);
        end
    endtask

    task sb_report;
        $display("============================================");
        $display("  SCOREBOARD REPORT");
        $display("  Total checks : %0d", total_checks);
        $display("  PASS         : %0d", pass_count);
        $display("  FAIL         : %0d", fail_count);
        if (fail_count == 0) $display("  Result       : ALL PASS");
        else $display("  Result       : %0d FAILED", fail_count);
        $display("============================================");
    endtask

    // ============================================================
    // DUT
    // ============================================================
    lidar_top #(
        .CLK_FREQ       (CLK_FREQ),
        .BAUD_RATE      (SIM_BAUD_RATE),
        .FRONT_ANGLE_DEG(FRONT_ANGLE_DEG),
        .BRAKE_DIST_MM  (BRAKE_DIST_MM),
        .HOLD_MS        (32'd5)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .lidar_rx   (lidar_rx),
        .brake_gpio (brake_gpio),
        .warning_led(warning_led)
    );

    always #(CLK_PERIOD / 2) clk = ~clk;

    // ============================================================
    // UART tasks
    // ============================================================
    task send_byte(input logic [7:0] data);
        integer i;
        lidar_rx = 1'b0;
        repeat (BIT_PERIOD) @(posedge clk);
        for (i = 0; i < 8; i++) begin
            lidar_rx = data[i];
            repeat (BIT_PERIOD) @(posedge clk);
        end
        lidar_rx = 1'b1;
        repeat (BIT_PERIOD) @(posedge clk);
    endtask

    task send_lidar_packet(input logic is_start, input logic [15:0] fsa_raw,
                           input logic [15:0] lsa_raw,
                           input logic [15:0] si_raw);
        logic [7:0] ct;
        ct = is_start ? 8'h01 : 8'h00;
        send_byte(8'hAA);
        send_byte(8'h55);
        send_byte(ct);
        send_byte(8'h01);  // LSN=1
        send_byte(fsa_raw[7:0]);
        send_byte(fsa_raw[15:8]);
        send_byte(lsa_raw[7:0]);
        send_byte(lsa_raw[15:8]);
        send_byte(8'h00);  // CS (무시)
        send_byte(8'h00);
        send_byte(si_raw[7:0]);
        send_byte(si_raw[15:8]);
    endtask

    // ============================================================
    // 변환 함수
    // ============================================================

    // 각도 → FSA raw
    // FSA = angle * 128 + 1 (패리티)
    function automatic logic [15:0] angle_to_fsa(input logic [8:0] ang);
        angle_to_fsa = (ang << 7) | 16'h0001;
    endfunction

    // FSA raw → 각도 역산 (DUT 가 실제 파싱하는 값)
    function automatic logic [8:0] fsa_to_angle(input logic [15:0] fsa);
        fsa_to_angle = (fsa >> 1) >> 6;
    endfunction

    // 거리 → Si raw
    function automatic logic [15:0] dist_to_si(input logic [13:0] dist_mm);
        logic [5:0] di_low;
        logic [7:0] di_high;
        di_low     = dist_mm[5:0];
        di_high    = dist_mm[13:6];
        dist_to_si = {di_high, di_low, 2'b00};
    endfunction

    // Si raw → 거리 역산 (DUT 가 실제 파싱하는 값)
    function automatic logic [13:0] si_to_dist(input logic [15:0] si_raw);
        logic [5:0] di_low;
        logic [7:0] di_high;
        di_low     = si_raw[7:2];
        di_high    = si_raw[15:8];
        si_to_dist = {di_high, di_low};
    endfunction

    // 기대값 계산 (DUT 파싱값 기준)
    function automatic logic is_brake_expected(input logic [8:0] ang,
                                               input logic [13:0] dist_mm);
        logic in_front;
        in_front = (ang <= FRONT_ANGLE_DEG) ||
                   (ang >= (9'd360 - FRONT_ANGLE_DEG));
        is_brake_expected = in_front &&
                            (dist_mm <= BRAKE_DIST_MM) &&
                            (dist_mm != 0);
    endfunction

    function automatic logic is_warn_expected(input logic [8:0] ang,
                                              input logic [13:0] dist_mm);
        logic in_front;
        in_front = (ang <= FRONT_ANGLE_DEG) ||
                   (ang >= (9'd360 - FRONT_ANGLE_DEG));
        is_warn_expected = in_front &&
                           (dist_mm <= WARN_DIST_MM) &&
                           (dist_mm != 0);
    endfunction

    // ============================================================
    // Test variables
    // ============================================================
    integer        test_num;
    logic   [ 8:0] t_angle;
    logic   [ 8:0] t_angle_parsed;  // DUT 파싱 각도
    logic   [13:0] t_dist_mm;
    logic   [13:0] t_dist_parsed;  // DUT 파싱 거리
    logic   [15:0] t_fsa;
    logic   [15:0] t_si;
    logic          exp_brake;
    logic          exp_warn;
    string         t_desc;
    integer        rand_angle;
    integer        rand_dist;

    // ============================================================
    // Test
    // ============================================================
    initial begin
        clk      = 0;
        rst_n    = 0;
        lidar_rx = 1;

        sb_init();

        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (10) @(posedge clk);

        $display("============================================");
        $display("  lidar_top random testbench");
        $display("  Angle : 0 ~ 359 deg (random)");
        $display("  Dist  : %0d ~ %0d mm (random)", DIST_MIN_MM, DIST_MAX_MM);
        $display("  Tests : %0d", NUM_TESTS);
        $display("============================================");

        repeat (NUM_TESTS) begin
            test_num = test_num + 1;

            // --------------------------------------------------
            // 랜덤 각도: 0 ~ 359
            // 랜덤 거리: 120 ~ 10000 mm (유효 범위)
            // --------------------------------------------------
            rand_angle = $urandom % 360;
            rand_dist  = DIST_MIN_MM + ($urandom % (DIST_MAX_MM - DIST_MIN_MM + 1));

            t_angle = rand_angle[8:0];
            t_dist_mm = rand_dist[13:0];
            t_fsa = angle_to_fsa(t_angle);
            t_si = dist_to_si(t_dist_mm);

            // DUT 가 실제 파싱할 값으로 기대값 계산
            t_angle_parsed = fsa_to_angle(t_fsa);
            t_dist_parsed = si_to_dist(t_si);

            exp_brake = is_brake_expected(t_angle_parsed, t_dist_parsed);
            exp_warn = is_warn_expected(t_angle_parsed, t_dist_parsed);

            $sformat(t_desc, "ang=%0d parsed=%0d dist=%0dmm parsed=%0dmm",
                     t_angle, t_angle_parsed, t_dist_mm, t_dist_parsed);
            $display("--- T%0d: %s (exp b=%b w=%b) ---", test_num, t_desc,
                     exp_brake, exp_warn);

            // 시작 패킷 (round_done 트리거)
            send_lidar_packet(1'b1, t_fsa, t_fsa, t_si);

            // 포인트 패킷
            send_lidar_packet(1'b0, t_fsa, t_fsa, t_si);

            // 처리 완료 대기
            repeat (WAIT_CLKS) @(posedge clk);

            // 체크
            sb_check(test_num, t_desc, dut.brake_sig, exp_brake, "brake  ");
            sb_check(test_num, t_desc, dut.warn_sig, exp_warn, "warning");

            // --------------------------------------------------
            // 초기화: 같은 각도로 안전 거리 전송 → brake 해제
            // --------------------------------------------------
            send_lidar_packet(1'b0, t_fsa, t_fsa, dist_to_si(14'd5000));
            send_lidar_packet(1'b1, t_fsa, t_fsa, dist_to_si(14'd5000));
            repeat (WAIT_CLKS) @(posedge clk);
        end

        sb_report();
        $finish;
    end

    initial begin
        $monitor("[%0t] brake=%b warn=%b", $time, dut.brake_sig, dut.warn_sig);
    end

endmodule
