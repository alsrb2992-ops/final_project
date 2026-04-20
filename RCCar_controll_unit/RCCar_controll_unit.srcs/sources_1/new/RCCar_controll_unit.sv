`timescale 1ns / 1ps

module RCCar_controll_unit (
    input  logic       clk,
    input  logic       reset_n,
    input  logic       bluetooth_rx,
    input  logic       stop,
    input  logic       brake_signal,
    input  logic       warning_signal,
    input  logic [2:0] direction_degree,
    output logic       pwm_servo,
    output logic       pwm_dc,
    output logic [1:0] dir_dc
);

    logic [7:0] rx_data;
    logic rx_done;
    logic [3:0] car_control;
    logic auto_mode;
    logic [2:0] auto_direction;

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
        .auto_mode(auto_mode),
        .brake_signal(brake_signal),
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
