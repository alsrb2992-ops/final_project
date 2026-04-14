`timescale 1ns / 1ps

module RCCar_controll_unit(
        input clk,
        input reset_n,
        input bluetooth_rx,
        input stop,
        output pwm_servo,
        output pwm_dc,
        output [1:0] dir_dc
    );

    logic [7:0] rx_data;
    logic rx_done;
    logic [3:0] car_control;

    uart_rx BLUETOOTH_RX (
        .clk(clk),
        .reset_n(reset_n),
        .rx(bluetooth_rx),
        .data_out(rx_data),
        .rx_done(rx_done)
    );

    command_decoder U_COMMAND_DECODER(
        .clk(clk),
        .reset_n(reset_n),
        .rx_data(rx_data),
        .rx_done(rx_done),
        .car_control(car_control)
    );

    servo_motor_controller U_SERVO_MOTOR_CONTROLLER(
        .clk(clk),
        .reset_n(reset_n),
        .car_control(car_control),
        .pwm_servo(pwm_servo)
    );    

    dcmotor_controller U_DCMOTOR_CONTROLLER(
        .clk(clk),
        .reset_n(reset_n),
        .car_control(car_control),
        .stop(stop),
        .pwm_dc(pwm_dc),
        .dir_dc(dir_dc)
    );

endmodule
