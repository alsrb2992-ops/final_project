`timescale 1ns / 1ps

module top #(
    parameter CLK_FREQ              = 125_000_000,
    parameter BAUD_RATE             = 128_000,
    parameter FRONT_ANGLE_DEG       = 9'd30,
    parameter BEHIND_ANGLE_DEG      = 9'd40,
    parameter RIGHT_START_ANGLE_DEG = 9'd30,
    parameter RIGHT_END_ANGLE_DEG   = 9'd90,
    parameter LEFT_START_ANGLE_DEG  = 9'd270,
    parameter LEFT_END_ANGLE_DEG    = 9'd330,
    parameter BRAKE_DIST_MM         = 14'd300,
    parameter WARN_DIST_MM          = 14'd600,
    parameter HOLD_MS               = 32'd200,
    parameter SIDE_HOLD_MS          = 32'd100,
    parameter TURN_THRESHOLD_MM     = 14'd800,
    parameter BIG_TURN_DIFF_MM      = 14'd100

) (
    input  logic       sysclk,
    input  logic       reset_n,
    input  logic       lidar_rx,
    input  logic       bluetooth_rx,
    output logic       pwm_servo,
    output logic       pwm_dc,
    output logic [1:0] dir_dc,
    output logic       brake_gpio,
    output logic       warning_led,
    output logic       side_warning_signal_gpio
);


    logic [2:0] direction_degree_gpio;
    wire rst_n = ~reset_n; // 리셋 신호는 active low이므로 반전하여 사용
    wire clk = sysclk;

    lidar_top #(
        .CLK_FREQ             (CLK_FREQ),
        .BAUD_RATE            (BAUD_RATE),
        .FRONT_ANGLE_DEG      (FRONT_ANGLE_DEG),
        .BEHIND_ANGLE_DEG     (BEHIND_ANGLE_DEG),
        .RIGHT_START_ANGLE_DEG(RIGHT_START_ANGLE_DEG),
        .RIGHT_END_ANGLE_DEG  (RIGHT_END_ANGLE_DEG),
        .LEFT_START_ANGLE_DEG (LEFT_START_ANGLE_DEG),
        .LEFT_END_ANGLE_DEG   (LEFT_END_ANGLE_DEG),
        .BRAKE_DIST_MM        (BRAKE_DIST_MM),
        .WARN_DIST_MM         (WARN_DIST_MM),
        .HOLD_MS              (HOLD_MS),
        .SIDE_HOLD_MS         (SIDE_HOLD_MS),
        .TURN_THRESHOLD_MM    (TURN_THRESHOLD_MM),
        .BIG_TURN_DIFF_MM     (BIG_TURN_DIFF_MM)
    ) u_lidar_top (
        .clk(clk),
        .rst_n(rst_n),
        .lidar_rx(lidar_rx),
        .brake_gpio(brake_gpio),
        .warning_led(warning_led),
        .side_warning_signal_gpio(side_warning_signal_gpio),
        .direction_degree_gpio(direction_degree_gpio)
    );

    RCCar_controll_unit u_RCCar_controll_unit (
        .clk             (clk),
        .reset_n         (rst_n),
        .bluetooth_rx    (bluetooth_rx),
        .stop            (stop),
        .brake_signal    (brake_gpio),
        .warning_signal  (warning_led),
        .direction_degree(direction_degree_gpio),
        .pwm_servo       (pwm_servo),
        .pwm_dc          (pwm_dc),
        .dir_dc          (dir_dc)
    );
endmodule
