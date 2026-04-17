`timescale 1ns / 1ps

module top (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       lidar_rx,
    input  logic       bluetooth_rx,
    output logic       pwm_servo,
    output logic       pwm_dc,
    output logic [1:0] dir_dc,
    output logic       brake_gpio,
    output logic       warning_led
);

    assign stop = brake_gpio; // 브레이크 신호는 라이다에서 받아옴

    lidar_top u_lidar_top (
        .clk(clk),
        .rst_n(rst_n),
        .lidar_rx(lidar_rx),
        .brake_gpio(brake_gpio),
        .warning_led(warning_led)
    );

    RCCar_controll_unit u_RCCar_controll_unit (
        .clk         (clk),
        .reset_n     (rst_n),
        .bluetooth_rx(bluetooth_rx),
        .stop        (stop),
        .pwm_servo   (pwm_servo),
        .pwm_dc      (pwm_dc),
        .dir_dc      (dir_dc)
    );
endmodule
