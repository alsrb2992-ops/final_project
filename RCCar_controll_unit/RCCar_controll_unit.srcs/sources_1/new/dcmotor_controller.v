`timescale 1ns / 1ps
`include "RCcar_define.vh"

module dcmotor_controller (
    input              clk,
    input              reset_n,
    input        [3:0] car_control,
    input              stop,
    output reg       pwm_dc,
    output reg [1:0] dir_dc
);
    // 10MHz

    localparam pwm_period = 5000;  // Hz  
    localparam forward_back_ms = 100;  // %     
    localparam turn_ms = 80;  // %     

    localparam pwm_period_cnt = 125000000 / pwm_period;
    localparam forward_back_cnt = (pwm_period_cnt * forward_back_ms) / 100;
    localparam turn_cnt = (pwm_period_cnt * turn_ms) / 100;

    reg [$clog2(125000000)-1:0] count;
    reg [$clog2(125000000)-1:0] period_set;
    reg [1:0] dir_set;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            count <= 0;
        end
        else begin
            if (count > pwm_period_cnt) count <= 0;
            else count <= count + 1;
        end
    end

    always @(*) begin
        period_set = 0;
        dir_set = 2'b00;
        case (car_control)
            `RC_STOP: begin  // 정지
                period_set = 0;
                dir_set = 2'b00;
            end
            `RC_FORWARD: begin  // 직진
                period_set = forward_back_cnt;
                dir_set = 2'b01;
            end
            `RC_BACKWARD: begin  // 후진
                period_set = forward_back_cnt;
                dir_set = 2'b10;
            end
            `RC_LEFT: begin  // 좌회전
                period_set = 0;
                dir_set = 2'b00;
            end
            `RC_RIGHT: begin  // 우회전
                period_set = 0;
                dir_set = 2'b00;
            end
            `RC_FORWARD_LEFT, `RC_TURN_LEFT_BIG, `RC_TURN_LEFT_SMALL : begin // 직진 + 좌회전
                period_set = turn_cnt;
                dir_set = 2'b01;
            end
            `RC_FORWARD_RIGHT, `RC_TURN_RIGHT_BIG, `RC_TURN_RIGHT_SMALL : begin // 직진 + 우회전
                period_set = turn_cnt;
                dir_set = 2'b01;
            end
            `RC_BACKWARD_LEFT: begin  // 후진 + 좌회전
                period_set = turn_cnt;
                dir_set = 2'b10;
            end
            `RC_BACKWARD_RIGHT: begin  // 후진 + 우회전
                period_set = turn_cnt;
                dir_set = 2'b10;
            end
        endcase
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pwm_dc <= 0;
            dir_dc <= 0;
        end else begin
            pwm_dc <= (count < period_set);
            dir_dc <= dir_set;
        end
    end
endmodule
