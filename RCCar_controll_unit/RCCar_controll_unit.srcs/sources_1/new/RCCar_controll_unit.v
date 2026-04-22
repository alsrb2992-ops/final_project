`timescale 1ns / 1ps

module RCCar_controll_unit (
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

    servo_motor_controller U_SERVO_MOTOR_CONTROLLER (
        .clk(clk),
        .reset_n(reset_n),
        .car_control(car_control),
        .pwm_servo(pwm_servo)
    );

    dcmotor_controller U_DCMOTOR_CONTROLLER (
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
