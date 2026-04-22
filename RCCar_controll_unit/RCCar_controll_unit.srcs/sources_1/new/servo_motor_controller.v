`timescale 1ns / 1ps
`include "RCcar_define.vh"

module servo_motor_controller (
    input clk,
    input reset_n,
    input [3:0] car_control,
    output reg pwm_servo
);

    // 1.5ms 가 중간

    reg [$clog2(125000000)-1:0] count;
    reg [$clog2(125000000)-1:0] period_set;

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

    always @(*) begin
        period_set = 0;
        case (car_control)
            `RC_STOP: begin  // 정지
                period_set = center_cnt;
            end
            `RC_FORWARD: begin  // 직진
                period_set = center_cnt;
            end
            `RC_BACKWARD: begin  // 후진
                period_set = center_cnt;
            end
            `RC_LEFT, `RC_TURN_LEFT_BIG, `RC_FORWARD_LEFT: begin  // 좌회전
                period_set = big_left_cnt;
            end
            `RC_RIGHT, `RC_FORWARD_RIGHT, `RC_TURN_RIGHT_BIG : begin // 우회전
                period_set = big_right_cnt;
            end

            `RC_TURN_LEFT_SMALL: begin  // 직진 + 좌회전
                period_set = small_left_cnt;
            end
            `RC_TURN_RIGHT_SMALL: begin  // 직진 + 우회전
                period_set = small_right_cnt;
            end
            `RC_BACKWARD_LEFT: begin  // 후진 + 좌회전
                period_set = big_left_cnt;
            end
            `RC_BACKWARD_RIGHT: begin  // 후진 + 우회전
                period_set = big_right_cnt;
            end
        endcase
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pwm_servo <= 0;
        end else begin
            pwm_servo <= (count < period_set);
        end
    end

endmodule
