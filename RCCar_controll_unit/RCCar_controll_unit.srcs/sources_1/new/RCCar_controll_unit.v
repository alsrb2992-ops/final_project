`timescale 1ns / 1ps

module RCCar_controll_unit #(
    parameter CLK_FREQ = 125_000_000,
    parameter MAX_CHANGE = 8,  // 20ms마다 최대 변화량
    parameter MAX_DECEL_PER_CYCLE = 1000
) (
    input  wire       clk,
    input  wire       reset_n,
    input  wire       bluetooth_rx,
    input  wire       stop,
    input  wire       brake_signal,
    input  wire       warning_signal,
    input  wire [2:0] direction_degree,
    output wire       pwm_servo,
    output wire       pwm_dc,
    output wire [1:0] dir_dc
);

    wire [7:0] rx_data;
    wire rx_done;
    wire [3:0] car_control;
    wire auto_mode;

    uart_rx BLUETOOTH_RX (
        .clk(clk),
        .reset_n(reset_n),
        .rx(bluetooth_rx),
        .data_out(rx_data),
        .rx_done(rx_done)
    );

    command_decoder U_COMMAND_DECODER (
        .clk(clk),
        .reset_n(reset_n),
        .rx_data(rx_data),
        .rx_done(rx_done),
        .brake_signal(brake_signal),
        .auto_mode(auto_mode),
        .direction_degree(direction_degree),
        .car_control(car_control)
    );

    servo_motor_controller #(
        .CLK_FREQ  (CLK_FREQ),
        .MAX_CHANGE(MAX_CHANGE)
    ) U_SERVO_MOTOR_CONTROLLER (
        .clk(clk),
        .reset_n(reset_n),
        .car_control(car_control),
        .pwm_servo(pwm_servo)
    );

    dcmotor_controller #(
        .CLK_FREQ(CLK_FREQ),
        .MAX_DECEL_PER_CYCLE(MAX_DECEL_PER_CYCLE),       // 감속은 최대 변화량 전체
        .MAX_ACCEL_PER_CYCLE(MAX_DECEL_PER_CYCLE / 2)  // 가속은 최대 변화량의 절반
    ) U_DCMOTOR_CONTROLLER (
        .clk(clk),
        .reset_n(reset_n),
        .car_control(car_control),
        .stop(stop),
        .pwm_dc(pwm_dc),
        .dir_dc(dir_dc)
    );

    rc_car_mode_change u_rc_car_mode_change (
        .clk      (clk),
        .rst_n    (reset_n),
        .rx_done  (rx_done),
        .rx_data  (rx_data),
        .auto_mode(auto_mode)
    );

endmodule
