// ============================================================
// tb_collision_detector.sv
// collision_detector module standalone testbench
// ============================================================
`timescale 1ns / 1ps

module tb_collision_detector ();

    localparam CLK_PERIOD = 20;  // 50MHz

    logic        clk;
    logic        rst_n;
    logic [13:0] distance;
    logic [ 8:0] angle;
    logic        data_valid;
    logic        pkt_done;
    logic        brake_signal;
    logic        warning_signal;

    // DUT
    collision_detector #(
        .FRONT_ANGLE_DEG(9'd20),
        .BRAKE_DIST_MM  (14'd500)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .distance      (distance),
        .angle         (angle),
        .data_valid    (data_valid),
        .pkt_done      (pkt_done),
        .brake_signal  (brake_signal),
        .warning_signal(warning_signal)
    );

    always #(CLK_PERIOD / 2) clk = ~clk;

    // ----------------------------------------------------------
    // 포인트 1개 입력 태스크
    // ----------------------------------------------------------
    task send_point(input logic [8:0] ang, input logic [13:0] distan);
        @(posedge clk);
        angle      <= ang;
        distance   <= distan;
        data_valid <= 1'b1;
        @(posedge clk);
        data_valid <= 1'b0;
    endtask

    // 패킷 완료 태스크
    task done_packet();
        @(posedge clk);
        pkt_done <= 1'b1;
        @(posedge clk);
        pkt_done <= 1'b0;
        @(posedge clk);  // 결과 반영 대기
    endtask

    // 결과 체크 태스크
    task check(input string test_name, input logic exp_brake,
               input logic exp_warn);
        $display("--- %s ---", test_name);
        $display("  brake_signal   = %b (expected: %b) %s", brake_signal,
                 exp_brake, (brake_signal === exp_brake) ? "PASS" : "FAIL");
        $display("  warning_signal = %b (expected: %b) %s", warning_signal,
                 exp_warn, (warning_signal === exp_warn) ? "PASS" : "FAIL");
    endtask

    // ----------------------------------------------------------
    // Test
    // ----------------------------------------------------------
    initial begin
        clk        = 0;
        rst_n      = 0;
        distance   = 0;
        angle      = 0;
        data_valid = 0;
        pkt_done   = 0;

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        $display("========================================");
        $display(" collision_detector testbench start");
        $display("========================================");

        // ------------------------------------------------
        // Test 1: Front zone, distance < BRAKE_DIST
        // angle=5deg (front zone), dist=300mm -> brake=1
        // ------------------------------------------------
        send_point(9'd5, 14'd300);
        done_packet();
        check("Test1: Front(5deg) Dist=300mm", 1'b1, 1'b1);

        // ------------------------------------------------
        // Test 2: Front zone, BRAKE_DIST < dist < WARN_DIST
        // angle=5deg, dist=800mm -> warn=1 brake=0
        // ------------------------------------------------
        send_point(9'd5, 14'd800);
        done_packet();
        check("Test2: Front(5deg) Dist=800mm", 1'b0, 1'b1);

        // ------------------------------------------------
        // Test 3: Front zone, dist > WARN_DIST
        // angle=5deg, dist=1200mm -> brake=0 warn=0
        // ------------------------------------------------
        send_point(9'd5, 14'd1200);
        done_packet();
        check("Test3: Front(5deg) Dist=1200mm", 1'b0, 1'b0);

        // ------------------------------------------------
        // Test 4: Outside front zone
        // angle=90deg, dist=300mm -> brake=0 warn=0
        // ------------------------------------------------
        send_point(9'd90, 14'd300);
        done_packet();
        check("Test4: Side(90deg) Dist=300mm", 1'b0, 1'b0);

        // ------------------------------------------------
        // Test 5: Rear zone
        // angle=180deg, dist=300mm -> brake=0 warn=0
        // ------------------------------------------------
        send_point(9'd180, 14'd300);
        done_packet();
        check("Test5: Rear(180deg) Dist=300mm", 1'b0, 1'b0);

        // ------------------------------------------------
        // Test 6: Front zone boundary (exactly 20deg)
        // angle=20deg, dist=300mm -> brake=1
        // ------------------------------------------------
        send_point(9'd20, 14'd300);
        done_packet();
        check("Test6: Front boundary(20deg) Dist=300mm", 1'b1, 1'b1);

        // ------------------------------------------------
        // Test 7: Just outside front zone (21deg)
        // angle=21deg, dist=300mm -> brake=0
        // ------------------------------------------------
        send_point(9'd21, 14'd300);
        done_packet();
        check("Test7: Outside boundary(21deg) Dist=300mm", 1'b0, 1'b0);

        // ------------------------------------------------
        // Test 8: Rear front zone (340deg)
        // angle=340deg, dist=300mm -> brake=1
        // ------------------------------------------------
        send_point(9'd340, 14'd300);
        done_packet();
        check("Test8: Rear-front(340deg) Dist=300mm", 1'b1, 1'b1);

        // ------------------------------------------------
        // Test 9: Distance = 0 (invalid point)
        // angle=5deg, dist=0 -> brake=0
        // ------------------------------------------------
        send_point(9'd5, 14'd0);
        done_packet();
        check("Test9: Front(5deg) Dist=0 (invalid)", 1'b0, 1'b0);

        // ------------------------------------------------
        // Test 10: Multiple points in one packet
        // point1: side 90deg 300mm (no danger)
        // point2: front 5deg 300mm (danger)
        // -> brake=1 expected
        // ------------------------------------------------
        send_point(9'd90, 14'd300);  // side, safe
        send_point(9'd5, 14'd300);  // front, danger
        done_packet();
        check("Test10: Multi-point (side safe + front danger)", 1'b1, 1'b1);

        // ------------------------------------------------
        // Test 11: Multiple points, all safe
        // point1: front 5deg 1200mm
        // point2: front 10deg 1200mm
        // -> brake=0 warn=0
        // ------------------------------------------------
        send_point(9'd5, 14'd1200);
        send_point(9'd10, 14'd1200);
        done_packet();
        check("Test11: Multi-point all safe", 1'b0, 1'b0);

        $display("========================================");
        $display(" Simulation Done");
        $display("========================================");
        $finish;
    end

    // 모니터링
    initial begin
        $monitor("t=%0t | angle=%0d dist=%0d | brake=%b warn=%b", $time, angle,
                 distance, brake_signal, warning_signal);
    end

endmodule
