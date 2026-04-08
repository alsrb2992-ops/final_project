// ============================================================
// tb_lidar_top_random.sv
// lidar_top 랜덤 테스트벤치
// - 경계값 각도/거리 위주로 brake/warn 자주 발생
// ============================================================
`timescale 1ns / 1ps

module tb_lidar_top;

    localparam CLK_FREQ = 125_000_000;
    localparam CLK_PERIOD = 8;
    localparam SIM_BAUD_RATE = 6_250_000;
    localparam BIT_PERIOD = CLK_FREQ / SIM_BAUD_RATE;

    localparam FRONT_ANGLE_DEG = 9'd20;
    localparam BRAKE_DIST_MM = 14'd500;
    localparam WARN_DIST_MM = 14'd1000;

    localparam NUM_TESTS = 100;
    localparam WAIT_CLKS = 12 * 10 * BIT_PERIOD + 200;

    logic clk, rst_n, lidar_rx;
    logic brake_gpio, warning_led;

    // ============================================================
    // Scoreboard
    // ============================================================
    integer total_checks, pass_count, fail_count;

    task sb_init;
        total_checks = 0;
        pass_count   = 0;
        fail_count   = 0;
    endtask

    task sb_check(input integer test_num, input string desc, input logic actual,
                  input logic expected, input string sig);
        total_checks++;
        if (actual === 1'bx || actual === 1'bz) begin
            fail_count++;
            $display("[%0t] [UNKN] T%0d | %s | %s=X (exp:%b) <<<", $time,
                     test_num, desc, sig, expected);
        end else if (actual === expected) begin
            pass_count++;
            $display("[%0t] [PASS] T%0d | %s | %s=%b", $time, test_num, desc,
                     sig, actual);
        end else begin
            fail_count++;
            $display("[%0t] [FAIL] T%0d | %s | %s=%b (exp:%b) <<<", $time,
                     test_num, desc, sig, actual, expected);
        end
    endtask

    task sb_report;
        $display("============================================");
        $display("  Total:%0d  PASS:%0d  FAIL:%0d", total_checks, pass_count,
                 fail_count);
        if (fail_count == 0) $display("  ALL PASS");
        else $display("  %0d FAILED", fail_count);
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
    // UART
    // ============================================================
    task send_byte(input logic [7:0] d);
        integer i;
        lidar_rx = 0;
        repeat (BIT_PERIOD) @(posedge clk);
        for (i = 0; i < 8; i++) begin
            lidar_rx = d[i];
            repeat (BIT_PERIOD) @(posedge clk);
        end
        lidar_rx = 1;
        repeat (BIT_PERIOD) @(posedge clk);
    endtask

    task send_pkt(input logic is_start, input logic [15:0] fsa,
                  input logic [15:0] si);
        send_byte(8'hAA);
        send_byte(8'h55);
        send_byte(is_start ? 8'h01 : 8'h00);
        send_byte(8'h01);  // LSN=1
        send_byte(fsa[7:0]);
        send_byte(fsa[15:8]);
        send_byte(fsa[7:0]);
        send_byte(fsa[15:8]);  // LSA = FSA
        send_byte(8'h00);
        send_byte(8'h00);  // CS 무시
        send_byte(si[7:0]);
        send_byte(si[15:8]);
    endtask

    // ============================================================
    // 변환 함수
    // ============================================================
    function automatic logic [15:0] to_fsa(input logic [8:0] ang);
        to_fsa = (ang << 7) | 16'h0001;
    endfunction

    function automatic logic [8:0] to_angle(input logic [15:0] fsa);
        to_angle = fsa[15:7];
    endfunction

    function automatic logic [15:0] to_si(input logic [13:0] dist_1);
        to_si = {dist_1[13:6], dist_1[5:0], 2'b00};
    endfunction

    function automatic logic [13:0] to_dist(input logic [15:0] si);
        to_dist = {si[15:8], si[7:2]};
    endfunction

    function automatic logic chk_brake(input logic [8:0] ang,
                                       input logic [13:0] dist_1);
        logic in_front;
        in_front  = (ang <= FRONT_ANGLE_DEG) ||
                    (ang >= (9'd360 - FRONT_ANGLE_DEG));
        chk_brake = in_front && (dist_1 > 0) && (dist_1 <= BRAKE_DIST_MM);
    endfunction

    function automatic logic chk_warn(input logic [8:0] ang,
                                      input logic [13:0] dist_1);
        logic in_front;
        in_front = (ang <= FRONT_ANGLE_DEG) ||
                   (ang >= (9'd360 - FRONT_ANGLE_DEG));
        chk_warn = in_front && (dist_1 > 0) && (dist_1 <= WARN_DIST_MM);
    endfunction

    // ============================================================
    // 경계값 테이블
    // 각도: 전방존 안팎 경계 + 측면 + 후방
    // 거리: brake/warn 경계값 위아래
    // ============================================================
    localparam NA = 14;
    localparam ND = 8;

    logic   [ 8:0] atbl     [0:NA-1];
    logic   [13:0] dtbl     [0:ND-1];

    // ============================================================
    // 테스트 변수
    // ============================================================
    integer        test_num;
    integer ai, di;
    logic [8:0] t_ang, t_ang_p;
    logic [13:0] t_dist, t_dist_p;
    logic [15:0] t_fsa, t_si;
    logic exp_b, exp_w;
    string t_desc;

    // ============================================================
    // Test
    // ============================================================
    initial begin
        // 각도 테이블 초기화
        atbl[0]  = 9'd0;  // 정면
        atbl[1]  = 9'd1;  // 전방 안쪽
        atbl[2]  = FRONT_ANGLE_DEG - 9'd1;  // 경계 안쪽
        atbl[3]  = FRONT_ANGLE_DEG;  // 경계 (포함)
        atbl[4]  = FRONT_ANGLE_DEG + 9'd1;  // 경계 바깥
        atbl[5]  = 9'd90;  // 측면
        atbl[6]  = 9'd180;  // 후방
        atbl[7]  = 9'd270;  // 측면
        atbl[8]  = 9'd360 - FRONT_ANGLE_DEG - 9'd1;  // 경계 바깥
        atbl[9]  = 9'd360 - FRONT_ANGLE_DEG;  // 경계 (포함)
        atbl[10] = 9'd360 - FRONT_ANGLE_DEG + 9'd1;  // 경계 안쪽
        atbl[11] = 9'd355;  // 전방 안쪽
        atbl[12] = 9'd358;  // 전방 안쪽
        atbl[13] = 9'd359;  // 정면 직전

        // 거리 테이블 초기화
        dtbl[0]  = 14'd120;  // 최소 유효거리 (brake)
        dtbl[1]  = 14'd300;  // brake
        dtbl[2]  = BRAKE_DIST_MM - 14'd1;  // brake 경계 안쪽
        dtbl[3]  = BRAKE_DIST_MM;  // brake 경계 (포함)
        dtbl[4]  = BRAKE_DIST_MM + 14'd1;  // warn only 시작
        dtbl[5]  = WARN_DIST_MM - 14'd1;  // warn 경계 안쪽
        dtbl[6]  = WARN_DIST_MM;  // warn 경계 (포함)
        dtbl[7]  = WARN_DIST_MM + 14'd100;  // safe

        clk      = 0;
        rst_n    = 0;
        lidar_rx = 1;
        test_num = 0;
        sb_init();

        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (10) @(posedge clk);

        $display("============================================");
        $display("  lidar_top random testbench");
        $display("  FRONT=±%0d° BRAKE=%0dmm WARN=%0dmm", FRONT_ANGLE_DEG,
                 BRAKE_DIST_MM, WARN_DIST_MM);
        $display("  Tests: %0d", NUM_TESTS);
        $display("============================================");

        repeat (NUM_TESTS) begin
            test_num = test_num + 1;

            // 랜덤으로 각도/거리 인덱스 선택
            ai = $urandom % NA;
            di = $urandom % ND;

            t_ang   = atbl[ai];
            t_dist  = dtbl[di];
            t_fsa   = to_fsa(t_ang);
            t_si    = to_si(t_dist);

            // DUT 파싱값 기준으로 기대값 계산
            t_ang_p  = to_angle(t_fsa);
            t_dist_p = to_dist(t_si);
            exp_b    = chk_brake(t_ang_p, t_dist_p);
            exp_w    = chk_warn(t_ang_p, t_dist_p);

            $sformat(t_desc, "ang=%0d dist_1=%0dmm", t_ang_p, t_dist_p);
            $display("--- T%0d: %s (exp b=%b w=%b) ---", test_num, t_desc,
                     exp_b, exp_w);

            // 시작 패킷
            send_pkt(1'b1, t_fsa, t_si);
            // 포인트 패킷
            send_pkt(1'b0, t_fsa, t_si);
            repeat (WAIT_CLKS) @(posedge clk);

            sb_check(test_num, t_desc, dut.brake_sig, exp_b, "brake");
            sb_check(test_num, t_desc, dut.warn_sig, exp_w, "warn ");

            // 초기화: 안전 거리로 brake 해제
            send_pkt(1'b0, t_fsa, to_si(14'd5000));
            send_pkt(1'b1, t_fsa, to_si(14'd5000));
            repeat (WAIT_CLKS) @(posedge clk);
        end

        sb_report();
        $finish;
    end

    initial begin
        $monitor("[%0t] brake=%b warn=%b", $time, dut.brake_sig, dut.warn_sig);
    end

endmodule
