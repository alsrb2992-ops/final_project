// ============================================================
// tb_collision_detector.sv
// collision_detector module verification testbench
// ============================================================
`timescale 1ns / 1ps

module tb_collision_detector;

    // Clock and Reset
    logic        clk;
    logic        rst_n;

    // DUT inputs
    logic [13:0] c_distance;
    logic [ 8:0] c_angle;
    logic        c_data_valid;
    logic        c_round_done;

    // DUT outputs
    logic        c_brake_signal;
    logic        c_warning_signal;
    logic        c_left_warning_signal;
    logic        c_right_warning_signal;
    logic [13:0] c_left_min_distance;
    logic [13:0] c_right_min_distance;

    // Clock generation (10ns period = 100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // DUT instantiation
    collision_detector #(
        .FRONT_ANGLE_DEG      (45),
        .BEHIND_ANGLE_DEG     (40),
        .RIGHT_START_ANGLE_DEG(45),
        .RIGHT_END_ANGLE_DEG  (90),
        .LEFT_START_ANGLE_DEG (270),
        .LEFT_END_ANGLE_DEG   (315),
        .BRAKE_DIST_MM        (300),
        .WARN_DIST_MM         (400),
        .SIDE_DIST_MM         (300)
    ) dut (
        .clk                 (clk),
        .rst_n               (rst_n),
        .distance            (c_distance),
        .angle               (c_angle),
        .data_valid          (c_data_valid),
        .round_done          (c_round_done),
        .brake_signal        (c_brake_signal),
        .warning_signal      (c_warning_signal),
        .left_warning_signal (c_left_warning_signal),
        .right_warning_signal(c_right_warning_signal),
        .left_min_distance   (c_left_min_distance),
        .right_min_distance  (c_right_min_distance)
    );

    // ============================================================
    // Task: Send LiDAR data
    // ============================================================
    task send_lidar_data(input [8:0] ang, input [13:0] c_dist);
        begin
            @(posedge clk);
            c_angle <= ang;
            c_distance <= c_dist;
            c_data_valid <= 1'b1;
            @(posedge clk);
            c_data_valid <= 1'b0;
        end
    endtask

    // ============================================================
    // Task: Trigger round completion
    // ============================================================
    task trigger_round_done();
        begin
            @(posedge clk);
            c_round_done <= 1'b1;
            @(posedge clk);
            c_round_done <= 1'b0;
        end
    endtask

    // ============================================================
    // Task: Wait cycles
    // ============================================================
    task wait_cycles(input int cycles);
        begin
            repeat (cycles) @(posedge clk);
        end
    endtask

    // ============================================================
    // Task: Clear all warning signals completely
    // Sends safe data and triggers round_done until all signals are low
    // ============================================================
    task clear_all_warnings();
        int max_attempts;
        begin
            max_attempts = 5;  // Prevent infinite loop

            $display("  [CLEANUP] Clearing all warning signals...");

            while ((c_brake_signal || c_warning_signal || 
                    c_left_warning_signal || c_right_warning_signal) && 
                   max_attempts > 0) begin

                // Send safe distances across all zones
                send_lidar_data(9'd0, 14'd1000);  // Front safe
                send_lidar_data(9'd60, 14'd1000);  // Right safe
                send_lidar_data(9'd290, 14'd1000);  // Left safe
                wait_cycles(2);

                // Trigger round completion
                trigger_round_done();
                wait_cycles(2);

                $display(
                    "  [CLEANUP] Attempt %0d: brake=%b, warn=%b, left=%b, right=%b",
                    (6 - max_attempts), c_brake_signal, c_warning_signal,
                    c_left_warning_signal, c_right_warning_signal);

                max_attempts = max_attempts - 1;
            end

            if (!c_brake_signal && !c_warning_signal && 
                !c_left_warning_signal && !c_right_warning_signal) begin
                $display("  [CLEANUP] All signals cleared successfully\n");
            end else begin
                $display("  [CLEANUP] WARNING: Some signals still active!\n");
            end
        end
    endtask

    // ============================================================
    // Main test sequence
    // ============================================================
    initial begin
        // Waveform dump setup
        $dumpfile("collision_detector.vcd");
        $dumpvars(0, tb_collision_detector);

        // Initialization
        rst_n = 0;
        c_distance = 14'd0;
        c_angle = 9'd0;
        c_data_valid = 1'b0;
        c_round_done = 1'b0;

        // Release reset
        repeat (5) @(posedge clk);
        rst_n = 1;
        $display("\n[%0t ns] ========== Test Start ==========", $time);

        // ------------------------------------------------------------
        // TEST 1: Front brake zone (0deg, 250mm)
        // Expected: brake_signal = 1, warning_signal = 1
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 1] Front 250mm (brake zone)", $time);
        send_lidar_data(9'd0, 14'd250);
        wait_cycles(2);

        if (c_brake_signal && c_warning_signal)
            $display(
                "  PASS: brake=%b, warning=%b", c_brake_signal, c_warning_signal
            );
        else
            $display(
                "  FAIL: brake=%b, warning=%b", c_brake_signal, c_warning_signal
            );

        // ------------------------------------------------------------
        // TEST 2: Round done with danger present
        // Expected: signals remain active
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 2] Round done (danger was present)", $time);
        trigger_round_done();
        wait_cycles(2);

        if (c_brake_signal && c_warning_signal)
            $display(
                "  PASS: Signals maintained (brake=%b, warning=%b)",
                c_brake_signal,
                c_warning_signal
            );
        else
            $display(
                "  FAIL: Signals released (brake=%b, warning=%b)",
                c_brake_signal,
                c_warning_signal
            );

        // ------------------------------------------------------------
        // TEST 3: Safe round then release
        // Expected: signals released
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 3] Safe round then release", $time);
        send_lidar_data(9'd0, 14'd500);
        wait_cycles(2);
        trigger_round_done();
        wait_cycles(2);

        if (!c_brake_signal && !c_warning_signal)
            $display(
                "  PASS: Signals released (brake=%b, warning=%b)",
                c_brake_signal,
                c_warning_signal
            );
        else
            $display(
                "  FAIL: Signals maintained (brake=%b, warning=%b)",
                c_brake_signal,
                c_warning_signal
            );

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 4: Front warning only (0deg, 350mm)
        // Expected: brake_signal = 0, warning_signal = 1
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 4] Front 350mm (warning only)", $time);
        send_lidar_data(9'd0, 14'd350);
        wait_cycles(2);

        if (!c_brake_signal && c_warning_signal)
            $display(
                "  PASS: brake=%b, warning=%b", c_brake_signal, c_warning_signal
            );
        else
            $display(
                "  FAIL: brake=%b, warning=%b", c_brake_signal, c_warning_signal
            );

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 5: Left wall multiple measurements - minimum tracking
        // Expected: left_min_distance tracks the minimum value
        // ------------------------------------------------------------
        $display(
            "\n[%0t ns] [TEST 5] Left wall multiple measurements (min tracking)",
            $time);
        send_lidar_data(9'd270, 14'd400);  // Left zone, 400mm
        wait_cycles(1);
        $display("  After 270deg/400mm: left_min=%0d mm, left_warning=%b",
                 c_left_min_distance, c_left_warning_signal);

        send_lidar_data(9'd280, 14'd250);  // Left zone, 250mm (closer!)
        wait_cycles(1);
        $display("  After 280deg/250mm: left_min=%0d mm, left_warning=%b",
                 c_left_min_distance, c_left_warning_signal);

        send_lidar_data(9'd290, 14'd350);  // Left zone, 350mm (farther)
        wait_cycles(1);
        $display("  After 290deg/350mm: left_min=%0d mm, left_warning=%b",
                 c_left_min_distance, c_left_warning_signal);

        send_lidar_data(9'd300, 14'd180);  // Left zone, 180mm (closest!)
        wait_cycles(1);
        $display("  After 300deg/180mm: left_min=%0d mm, left_warning=%b",
                 c_left_min_distance, c_left_warning_signal);

        if (c_left_min_distance == 14'd180 && c_left_warning_signal)
            $display("  PASS: Minimum distance tracked correctly (180mm)");
        else
            $display(
                "  FAIL: left_min=%0d mm (expected 180mm), warning=%b",
                c_left_min_distance,
                c_left_warning_signal
            );

        trigger_round_done();
        wait_cycles(2);
        $display("  After round: left_warning=%b, left_min=%0d mm",
                 c_left_warning_signal, c_left_min_distance);

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 6: Right wall multiple measurements - minimum tracking
        // Expected: right_min_distance tracks the minimum value
        // ------------------------------------------------------------
        $display(
            "\n[%0t ns] [TEST 6] Right wall multiple measurements (min tracking)",
            $time);
        send_lidar_data(9'd45, 14'd500);  // Right zone, 500mm
        wait_cycles(1);
        $display("  After 45deg/500mm: right_min=%0d mm, right_warning=%b",
                 c_right_min_distance, c_right_warning_signal);

        send_lidar_data(9'd60, 14'd280);  // Right zone, 280mm (closer!)
        wait_cycles(1);
        $display("  After 60deg/280mm: right_min=%0d mm, right_warning=%b",
                 c_right_min_distance, c_right_warning_signal);

        send_lidar_data(9'd75, 14'd320);  // Right zone, 320mm (farther)
        wait_cycles(1);
        $display("  After 75deg/320mm: right_min=%0d mm, right_warning=%b",
                 c_right_min_distance, c_right_warning_signal);

        send_lidar_data(9'd90, 14'd150);  // Right zone, 150mm (closest!)
        wait_cycles(1);
        $display("  After 90deg/150mm: right_min=%0d mm, right_warning=%b",
                 c_right_min_distance, c_right_warning_signal);

        if (c_right_min_distance == 14'd150 && c_right_warning_signal)
            $display("  PASS: Minimum distance tracked correctly (150mm)");
        else
            $display(
                "  FAIL: right_min=%0d mm (expected 150mm), warning=%b",
                c_right_min_distance,
                c_right_warning_signal
            );

        trigger_round_done();
        wait_cycles(2);
        $display("  After round: right_warning=%b, right_min=%0d mm",
                 c_right_warning_signal, c_right_min_distance);

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 7: Left wall safe distance (no warning)
        // Expected: left_warning_signal = 0
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 7] Left wall safe distance (400mm+)", $time);
        send_lidar_data(9'd270, 14'd450);  // Safe distance
        wait_cycles(1);
        send_lidar_data(9'd290, 14'd500);  // Safe distance
        wait_cycles(1);
        send_lidar_data(9'd310, 14'd420);  // Safe distance
        wait_cycles(1);

        if (!c_left_warning_signal)
            $display(
                "  PASS: No warning for safe distance (warning=%b)",
                c_left_warning_signal
            );
        else
            $display(
                "  FAIL: Warning triggered incorrectly (warning=%b)",
                c_left_warning_signal
            );

        trigger_round_done();
        wait_cycles(2);

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 8: Right wall safe distance (no warning)
        // Expected: right_warning_signal = 0
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 8] Right wall safe distance (400mm+)",
                 $time);
        send_lidar_data(9'd50, 14'd450);  // Safe distance
        wait_cycles(1);
        send_lidar_data(9'd70, 14'd500);  // Safe distance
        wait_cycles(1);
        send_lidar_data(9'd85, 14'd420);  // Safe distance
        wait_cycles(1);

        if (!c_right_warning_signal)
            $display(
                "  PASS: No warning for safe distance (warning=%b)",
                c_right_warning_signal
            );
        else
            $display(
                "  FAIL: Warning triggered incorrectly (warning=%b)",
                c_right_warning_signal
            );

        trigger_round_done();
        wait_cycles(2);

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 9: Left wall boundary angle test (269deg and 316deg)
        // Expected: 269deg not in zone, 315deg in zone
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 9] Left wall boundary angle test", $time);
        send_lidar_data(9'd269, 14'd200);  // Just outside left zone
        wait_cycles(1);
        $display("  269deg/200mm: left_warning=%b (should be 0)",
                 c_left_warning_signal);

        send_lidar_data(9'd270, 14'd200);  // Left zone start
        wait_cycles(1);
        $display("  270deg/200mm: left_warning=%b (should be 1)",
                 c_left_warning_signal);

        send_lidar_data(9'd315, 14'd200);  // Left zone end
        wait_cycles(1);
        $display("  315deg/200mm: left_warning=%b (should be 1)",
                 c_left_warning_signal);

        trigger_round_done();
        wait_cycles(2);

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 10: Right wall boundary angle test (44deg and 91deg)
        // Expected: 44deg not in zone, 90deg in zone
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 10] Right wall boundary angle test", $time);
        send_lidar_data(9'd44, 14'd200);  // Just outside right zone
        wait_cycles(1);
        $display("  44deg/200mm: right_warning=%b (should be 0)",
                 c_right_warning_signal);

        send_lidar_data(9'd45, 14'd200);  // Right zone start
        wait_cycles(1);
        $display("  45deg/200mm: right_warning=%b (should be 1)",
                 c_right_warning_signal);

        send_lidar_data(9'd90, 14'd200);  // Right zone end
        wait_cycles(1);
        $display("  90deg/200mm: right_warning=%b (should be 1)",
                 c_right_warning_signal);

        trigger_round_done();
        wait_cycles(2);

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 11: Front boundary angle (45deg, 250mm)
        // Expected: in_front_zone so brake_signal = 1
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 11] Front boundary angle 45deg", $time);
        send_lidar_data(9'd45, 14'd250);
        wait_cycles(2);

        if (c_brake_signal)
            $display(
                "  PASS: 45deg recognized as front zone (brake=%b)",
                c_brake_signal
            );
        else
            $display(
                "  FAIL: 45deg not recognized as front zone (brake=%b)",
                c_brake_signal
            );

        trigger_round_done();
        wait_cycles(2);

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 12: Front opposite boundary (315deg, 250mm)
        // Expected: 360 - 45 = 315 so in_front_zone, brake_signal = 1
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 12] Front opposite boundary 315deg", $time);
        send_lidar_data(9'd315, 14'd250);
        wait_cycles(2);

        if (c_brake_signal)
            $display(
                "  PASS: 315deg recognized as front zone (brake=%b)",
                c_brake_signal
            );
        else
            $display(
                "  FAIL: 315deg not recognized as front zone (brake=%b)",
                c_brake_signal
            );

        trigger_round_done();
        wait_cycles(2);

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 13: Multi-directional danger in one round
        // Expected: front, left, right all activated
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 13] Multi-directional danger in one round",
                 $time);

        send_lidar_data(9'd0, 14'd250);  // Front
        wait_cycles(1);
        send_lidar_data(9'd60, 14'd200);  // Right
        wait_cycles(1);
        send_lidar_data(9'd290, 14'd180);  // Left
        wait_cycles(1);

        trigger_round_done();
        wait_cycles(2);

        $display("  Status: brake=%b, warning=%b, left=%b, right=%b",
                 c_brake_signal, c_warning_signal, c_left_warning_signal,
                 c_right_warning_signal);
        $display("  Distance: left_min=%0d mm, right_min=%0d mm",
                 c_left_min_distance, c_right_min_distance);

        clear_all_warnings();  // Clean up before next test

        // ------------------------------------------------------------
        // TEST 14: Enter safe zone and release all
        // ------------------------------------------------------------
        $display("\n[%0t ns] [TEST 14] Enter safe zone and release all", $time);

        send_lidar_data(9'd0, 14'd600);  // Front safe
        send_lidar_data(9'd60, 14'd500);  // Right safe
        send_lidar_data(9'd290, 14'd500);  // Left safe
        wait_cycles(2);

        trigger_round_done();
        wait_cycles(2);

        if (!c_brake_signal && !c_warning_signal && !c_left_warning_signal && !c_right_warning_signal)
            $display("  PASS: All signals released");
        else
            $display(
                "  FAIL: Some signals maintained (brake=%b, warn=%b, left=%b, right=%b)",
                c_brake_signal,
                c_warning_signal,
                c_left_warning_signal,
                c_right_warning_signal
            );

        // Finish
        wait_cycles(10);
        $display("\n[%0t ns] ========== Test End ==========\n", $time);
        $finish;
    end

    // ============================================================
    // Monitor: Display signal changes
    // ============================================================
    always @(posedge clk) begin
        if (c_data_valid) begin
            $display("[%0t ns] INPUT: angle=%3d deg, distance=%4d mm", $time,
                     c_angle, c_distance);
        end

        if (c_brake_signal || c_warning_signal || c_left_warning_signal || c_right_warning_signal) begin
            $display("[%0t ns] OUTPUT: BRAKE=%b, WARN=%b, LEFT=%b, RIGHT=%b",
                     $time, c_brake_signal, c_warning_signal,
                     c_left_warning_signal, c_right_warning_signal);
        end
    end

endmodule
