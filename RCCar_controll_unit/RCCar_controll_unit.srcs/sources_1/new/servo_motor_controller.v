`timescale 1ns / 1ps
`include "RCcar_define.vh"

module servo_motor_controller #(
    parameter CLK_FREQ = 125_000_000,
    parameter MAX_CHANGE_PER_CYCLE = 125000000 * 20 / 1000 / 8 // 20ms마다 최대 변화량
) (
    input clk,
    input reset_n,
    input [3:0] car_control,
    output reg pwm_servo
);

    // 1.5ms 가 중간

    reg [ $clog2(CLK_FREQ)-1:0] current_period;
    reg [$clog2(125000000)-1:0] count;
    reg [$clog2(125000000)-1:0] period_set;

    localparam pwm_period = 50;  // Hz  
    localparam center_ms = 1.5;  // ms     
    localparam big_left_ms = 1.0;  // ms     
    localparam small_left_ms = 1.25;  // ms     

    localparam big_right_ms = 2.0;  // ms   
    localparam small_right_ms = 1.75;  // ms   

    localparam pwm_period_cnt = CLK_FREQ / pwm_period;
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
            current_period <= center_cnt;  // 초기값: 중앙
        end else begin
            // PWM 주기 시작 시점에만 업데이트 (20ms마다)
            if (count == 0) begin
                if (period_set > current_period) begin
                    // 목표가 현재보다 큼 → 증가 방향
                    if (period_set - current_period > MAX_CHANGE_PER_CYCLE) begin
                        current_period <= current_period + MAX_CHANGE_PER_CYCLE;
                    end else begin
                        current_period <= period_set;  // 거의 도달했으면 목표값으로
                    end
                end else if (period_set < current_period) begin
                    // 목표가 현재보다 작음 → 감소 방향
                    if (current_period - period_set > MAX_CHANGE_PER_CYCLE) begin
                        current_period <= current_period - MAX_CHANGE_PER_CYCLE;
                    end else begin
                        current_period <= period_set;  // 거의 도달했으면 목표값으로
                    end
                end
                // period_set == current_period 이면 변경 없음
            end
        end
    end



    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pwm_servo <= 0;
        end else begin
            pwm_servo <= (count < current_period);
        end
    end

endmodule
