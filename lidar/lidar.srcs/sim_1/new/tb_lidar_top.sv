// ============================================================
// tb_lidar_top.sv
// ============================================================
`timescale 1ns / 1ps

module tb_lidar_top;

    localparam CLK_FREQ = 50_000_000;
    localparam BAUD_RATE = 115_200;
    localparam CLK_PERIOD = 20;
    localparam BIT_PERIOD = CLK_FREQ / BAUD_RATE;

    logic clk, rst_n;
    logic lidar_rx;
    logic brake_gpio;
    logic warning_led;

    lidar_top #(
        .CLK_FREQ       (CLK_FREQ),
        .BAUD_RATE      (BAUD_RATE),
        .FRONT_ANGLE_DEG(9'd20),
        .BRAKE_DIST_MM  (14'd500),
        .HOLD_MS        (32'd10)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .lidar_rx   (lidar_rx),
        .brake_gpio (brake_gpio),
        .warning_led(warning_led)
    );

    always #(CLK_PERIOD / 2) clk = ~clk;

    // ----------------------------------------------------------
    // UART send task
    // ----------------------------------------------------------
    task send_byte(input logic [7:0] data);
        integer i;
        lidar_rx = 1'b0;  // start bit
        repeat (BIT_PERIOD) @(posedge clk);
        for (i = 0; i < 8; i++) begin
            lidar_rx = data[i];  // data bits LSB first
            repeat (BIT_PERIOD) @(posedge clk);
        end
        lidar_rx = 1'b1;  // stop bit
        repeat (BIT_PERIOD) @(posedge clk);
    endtask

    task send_packet(input logic [7:0] ct, input logic [7:0] lsn,
                     input logic [15:0] fsa, input logic [15:0] lsa,
                     input logic [15:0] cs, input logic [15:0] si_data);
        send_byte(8'hAA);
        send_byte(8'h55);
        send_byte(ct);
        send_byte(lsn);
        send_byte(fsa[7:0]);
        send_byte(fsa[15:8]);
        send_byte(lsa[7:0]);
        send_byte(lsa[15:8]);
        send_byte(cs[7:0]);
        send_byte(cs[15:8]);
        send_byte(si_data[7:0]);
        send_byte(si_data[15:8]);
    endtask

    // ----------------------------------------------------------
    // Test
    // ----------------------------------------------------------
    initial begin
        clk      = 0;
        rst_n    = 0;
        lidar_rx = 1;
        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (10) @(posedge clk);

        // --------------------------------------------------
        // Test 1: Front 5deg, 300mm -> brake=1 expected
        // FSA = 5*64*2+1 = 641 = 0x0281
        // Si  = 300mm : Di[5:0]=44, Di[13:6]=4 -> 0x04B0, IS=0
        // --------------------------------------------------
        $display("=== Test 1: Front(5deg) Dist=300mm -> brake=1 expected ===");
        send_packet(8'h00, 8'h01, 16'h0281, 16'h0281, 16'h0000, 16'h04B0);
        repeat (1000) @(posedge clk);
        $display("brake_gpio  = %b (expected: 1)", brake_gpio);

        // --------------------------------------------------
        // Test 2: Front 5deg, 800mm -> warn=1 brake=0 expected
        // Si = 800mm : Di[5:0]=32, Di[13:6]=12 -> 0x0C80, IS=0
        // --------------------------------------------------
        $display(
            "=== Test 2: Front(5deg) Dist=800mm -> warn=1 brake=0 expected ===");
        send_packet(8'h00, 8'h01, 16'h0281, 16'h0281, 16'h0000, 16'h0C80);
        repeat (1000) @(posedge clk);
        $display("brake_gpio  = %b (expected: 0)", brake_gpio);
        $display("warning_led = %b (expected: 1)", warning_led);

        // --------------------------------------------------
        // Test 3: Side 90deg, 300mm -> brake=0 expected
        // FSA = 90*64*2+1 = 11521 = 0x2D01
        // --------------------------------------------------
        $display("=== Test 3: Side(90deg) Dist=300mm -> brake=0 expected ===");
        send_packet(8'h00, 8'h01, 16'h2D01, 16'h2D01, 16'h0000, 16'h04B0);
        repeat (1000) @(posedge clk);
        $display("brake_gpio  = %b (expected: 0)", brake_gpio);

        // --------------------------------------------------
        // Test 4: Front 5deg, 300mm, IS=2 -> filtered, brake=0
        // Si = 0x04B2 (IS=2 specular interference)
        // --------------------------------------------------
        $display(
            "=== Test 4: Front(5deg) Dist=300mm IS=2 -> brake=0 expected ===");
        send_packet(8'h00, 8'h01, 16'h0281, 16'h0281, 16'h0000, 16'h04B2);
        repeat (1000) @(posedge clk);
        $display("brake_gpio  = %b (expected: 0, filtered)", brake_gpio);

        $display("=== Simulation Done ===");
        $finish;
    end

    initial begin
        $monitor("t=%0t brake=%b warn=%b", $time, brake_gpio, warning_led);
    end

endmodule
