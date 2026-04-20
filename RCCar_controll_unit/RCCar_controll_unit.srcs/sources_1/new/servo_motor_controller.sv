`timescale 1ns / 1ps
`include "RCcar_define.vh"

module servo_motor_controller (
    input clk,
    input reset_n,
    input [3:0] car_control,
    output logic pwm_servo
);

    // 1.5ms 가 중간

    logic [$clog2(125000000)-1:0] count;
    logic [$clog2(125000000)-1:0] period_set;

    localparam pwm_period = 50;  // Hz  
    localparam center_ms = 1.5;  // ms     
    localparam big_left_ms = 1.0;  // ms     
    localparam small_left_ms = 1.25;  // ms     

    localparam big_right_ms = 2.0;  // ms   
    localparam small_right_ms = 1.75;  // ms   

    localparam pwm_period_cnt = 125000000 / pwm_period;
    localparam center_cnt = center_ms * 1000000 / 8;

    localparam big_left_cnt = big_left_ms * 1000000 / 8;
    localparam small_left_cnt = small_left_ms * 1000000 / 8;

    localparam big_right_cnt = big_right_ms * 1000000 / 8;
    localparam small_right_cnt = small_right_ms * 1000000 / 8;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            count <= 0;
        end else begin
            if (count > pwm_period_cnt) begin
                count <= 0;
            end else begin
                count <= count + 1;
            end
        end
    end

    always_comb begin
        period_set = center_cnt;
        case (car_control)
            `RC_STOP: begin  // S
                period_set = center_cnt;
            end
            `RC_FORWARD: begin  // F
                period_set = center_cnt;
            end
            `RC_BACKWARD: begin  // B
                period_set = center_cnt;
            end
            `RC_TURN_RIGHT_BIG: begin  // R
                period_set = big_right_cnt;
            end
            `RC_TURN_RIGHT_SMALL: begin  // R
                period_set = small_right_cnt;
            end
            `RC_TURN_LEFT_BIG: begin  // L
                period_set = big_left_cnt;
            end
            `RC_TURN_LEFT_SMALL: begin  // L
                period_set = small_left_cnt;
            end

        endcase
    end

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pwm_servo <= 0;
        end else begin
            pwm_servo <= (count < period_set);
        end
    end

endmodule
