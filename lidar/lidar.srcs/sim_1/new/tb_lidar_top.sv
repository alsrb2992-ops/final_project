// ============================================================
// tb_lidar_top.sv
// lidar_top full testbench
// - 각도: 경계값 기반 (0, 19, 20, 21, 339, 340, 341, 359, 90, 180)
// - 거리: 300 / 600 / 1200 mm 3가지
// - 10회 랜덤 조합 테스트
// - scoreboard
// ============================================================
`timescale 1ns / 1ps

module tb_lidar_top;

    localparam CLK_FREQ = 125_000_000;
    localparam CLK_PERIOD = 8;  // 125MHz → 8ns

    // ----------------------------------------------------------
    // SIM_BAUD_RATE: 시뮬레이션 전용 고속 baud rate
    // 실제 HW baud rate (128_000) 대신 사용
    // BIT_PERIOD = CLK_FREQ / SIM_BAUD_RATE 클럭 수
    //
    //   128_000  → BIT_PERIOD = 976  클럭 (실제, 느림)
    //   6_250_000→ BIT_PERIOD = 20   클럭 (시뮬용, 빠름)
    //
    // DUT 에도 동일하게 적용해야 타이밍 맞음
    // ----------------------------------------------------------
    localparam SIM_BAUD_RATE = 6_250_000;
    localparam BIT_PERIOD = CLK_FREQ / SIM_BAUD_RATE;  // 20 클럭

    localparam FRONT_ANGLE_DEG = 9'd20;
    localparam BRAKE_DIST_MM = 14'd500;
    localparam WARN_DIST_MM = BRAKE_DIST_MM * 2;  // 1000mm

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
        if (actual === expected) begin
            pass_count = pass_count + 1;
            $display("  [PASS] T%0d | %s | %s=%b", test_num, test_desc,
                     signal_name, actual);
        end else begin
            fail_count = fail_count + 1;
            $display("  [FAIL] T%0d | %s | %s=%b (exp:%b) <<<", test_num,
                     test_desc, signal_name, actual, expected);
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
        .BAUD_RATE      (SIM_BAUD_RATE),    // 시뮬용 고속 baud rate
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
    // UART send tasks
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

    // 패킷 전송 (LSN=1 고정, Si 1개)
    task send_lidar_packet(input logic is_start,  // CT[bit0]
                           input logic [15:0] fsa_raw,
                           input logic [15:0] lsa_raw,
                           input logic [15:0] si_raw);
        logic [7:0] ct;
        ct = is_start ? 8'h01 : 8'h00;

        send_byte(8'hAA);  // PH LSB
        send_byte(8'h55);  // PH MSB
        send_byte(ct);  // CT
        send_byte(8'h01);  // LSN=1
        send_byte(fsa_raw[7:0]);  // FSA LSB
        send_byte(fsa_raw[15:8]);  // FSA MSB
        send_byte(lsa_raw[7:0]);  // LSA LSB
        send_byte(lsa_raw[15:8]);  // LSA MSB
        send_byte(8'h00);  // CS LSB (테스트용 무시)
        send_byte(8'h00);  // CS MSB
        send_byte(si_raw[7:0]);  // Si LSB
        send_byte(si_raw[15:8]);  // Si MSB
    endtask

    // ============================================================
    // 각도 → FSA raw 변환
    // FSA = angle * 64 * 2 + 1 (패리티)
    // ============================================================
    function automatic logic [15:0] angle_to_fsa(input logic [8:0] ang);
        angle_to_fsa = (ang * 16'd128) | 16'h0001;
    endfunction

    // 거리 → Si raw 변환
    // Di[5:0] = dist_mm % 64, Di[13:6] = dist_mm / 64
    // Si = (Di[5:0] << 2) | (Di[13:6] << 8), IS=0
    function automatic logic [15:0] dist_to_si(input logic [13:0] dist_mm);
        logic [5:0] di_low;
        logic [7:0] di_high;
        di_low     = dist_mm[5:0];
        di_high    = dist_mm[13:6];
        dist_to_si = {di_high, di_low[5:2], 2'b00};
    endfunction

    // 기대값 계산
    function automatic logic is_brake_expected(input logic [8:0] ang,
                                               input logic [13:0] dist_mm);
        logic in_front;
        in_front = (ang <= FRONT_ANGLE_DEG) ||
                   (ang >= (9'd360 - FRONT_ANGLE_DEG));
        is_brake_expected = in_front && (dist_mm <= BRAKE_DIST_MM)
                            && (dist_mm != 0);
    endfunction

    function automatic logic is_warn_expected(input logic [8:0] ang,
                                              input logic [13:0] dist_mm);
        logic in_front;
        in_front = (ang <= FRONT_ANGLE_DEG) ||
                   (ang >= (9'd360 - FRONT_ANGLE_DEG));
        is_warn_expected = in_front && (dist_mm <= WARN_DIST_MM)
                           && (dist_mm != 0);
    endfunction

    // ============================================================
    // 경계값 각도 테이블 (10가지)
    // ============================================================
    logic   [ 8:0] angle_table    [0:9];
    logic   [13:0] dist_mm_table  [0:2];  // 300, 600, 1200

    // ============================================================
    // Test variables
    // ============================================================
    integer        i;
    integer        test_num;
    integer        rand_angle_idx;
    integer        rand_dist_idx;
    logic   [ 8:0] t_angle;
    logic   [13:0] t_dist_mm;
    logic   [15:0] t_fsa;
    logic   [15:0] t_si;
    logic          exp_brake;
    logic          exp_warn;
    string         t_desc;

    // 결과 반영 대기 클럭 수
    localparam WAIT_CLKS = BIT_PERIOD * 15;  // 패킷 수신 후 충분히 대기

    // ============================================================
    // Test
    // ============================================================
    initial begin
        // 각도 경계값 테이블 초기화
        angle_table[0]   = 9'd0;  // 정면
        angle_table[1]   = 9'd19;  // 경계 안쪽
        angle_table[2]   = 9'd20;  // 경계 (포함)
        angle_table[3]   = 9'd21;  // 경계 바깥쪽
        angle_table[4]   = 9'd90;  // 측면
        angle_table[5]   = 9'd180;  // 후방
        angle_table[6]   = 9'd339;  // 후방 전방존 바깥
        angle_table[7]   = 9'd340;  // 후방 전방존 경계 (포함)
        angle_table[8]   = 9'd341;  // 후방 전방존 안쪽
        angle_table[9]   = 9'd359;  // 정면 직전

        // 거리 테이블: 300(brake), 600(warn), 1200(safe)
        dist_mm_table[0] = 14'd300;  // <= BRAKE_DIST  → brake
        dist_mm_table[1] = 14'd600;  // <= WARN_DIST   → warn only
        dist_mm_table[2] = 14'd1200;  // > WARN_DIST    → safe

        clk              = 0;
        rst_n            = 0;
        lidar_rx         = 1;

        sb_init();

        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (10) @(posedge clk);

        $display("============================================");
        $display("  lidar_top testbench");
        $display("  Angle table: boundary values (10 types)");
        $display("  Dist  table: 300 / 600 / 1200 mm");
        $display("  10 random combination tests");
        $display("============================================");

        for (test_num = 1; test_num <= 10; test_num++) begin

            // 랜덤으로 각도, 거리 인덱스 선택
            rand_angle_idx = $urandom % 10;
            rand_dist_idx  = $urandom % 3;

            t_angle        = angle_table[rand_angle_idx];
            t_dist_mm      = dist_mm_table[rand_dist_idx];
            t_fsa          = angle_to_fsa(t_angle);
            t_si           = dist_to_si(t_dist_mm);

            exp_brake      = is_brake_expected(t_angle, t_dist_mm);
            exp_warn       = is_warn_expected(t_angle, t_dist_mm);

            $sformat(t_desc, "ang=%0d dist_mm=%0d", t_angle, t_dist_mm);
            $display("--- Test %0d: %s (exp brake=%b warn=%b) ---", test_num,
                     t_desc, exp_brake, exp_warn);

            // 시작 패킷 (CT[bit0]=1) 전송 → round_done 트리거
            send_lidar_packet(1'b1, t_fsa, t_fsa, t_si);

            // 포인트 패킷 (CT[bit0]=0) 전송
            send_lidar_packet(1'b0, t_fsa, t_fsa, t_si);

            // 결과 반영 대기
            repeat (WAIT_CLKS) @(posedge clk);

            // 체크
            sb_check(test_num, t_desc, brake_gpio, exp_brake, "brake  ");
            sb_check(test_num, t_desc, warning_led, exp_warn, "warning");

            // 다음 테스트 전 초기화: 안전 거리로 회전 완료
            send_lidar_packet(1'b1, angle_to_fsa(9'd90), angle_to_fsa(9'd90),
                              dist_to_si(14'd1200));
            repeat (WAIT_CLKS) @(posedge clk);
        end

        sb_report();
        $finish;
    end

    initial begin
        $monitor("t=%0t | brake=%b warn=%b", $time, brake_gpio, warning_led);
    end

endmodule
