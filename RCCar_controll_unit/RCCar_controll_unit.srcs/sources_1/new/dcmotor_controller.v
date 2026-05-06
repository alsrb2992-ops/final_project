`timescale 1ns / 1ps
`include "RCcar_define.vh"

module dcmotor_controller #(
    parameter CLK_FREQ = 125_000_000,
    // 가속: 5000Hz PWM 주기(0.2ms)당 최대 변화량
    // 값이 클수록 빠르게 가속/감속
    parameter MAX_DECEL_PER_CYCLE = 1000,  // 감속 시 (더 빠르게)
    parameter MAX_ACCEL_PER_CYCLE = MAX_DECEL_PER_CYCLE / 2  // 가속 시
) (
    input            clk,
    input            reset_n,
    input      [3:0] car_control,
    input            stop,
    output reg       pwm_dc,
    output reg [1:0] dir_dc
);
    // 125MHz 클럭 기준

    localparam pwm_period = 5000;  // Hz  
    localparam forward_back_ms = 80;  // %     
    localparam turn_ms = 60;  // %     
    localparam pwm_period_cnt = CLK_FREQ / pwm_period;
    localparam forward_back_cnt = (pwm_period_cnt * forward_back_ms) / 100;
    localparam turn_cnt = (pwm_period_cnt * turn_ms) / 100;

    reg [$clog2(
CLK_FREQ
)-1:0] current_period;  // 현재 속도 (부드럽게 변경)
    reg [1:0] dir_set;  // 목표 방향
    reg [1:0] current_dir;  // 현재 방향
    reg [$clog2(125000000)-1:0] count;
    reg [$clog2(125000000)-1:0] period_set;

    // PWM 카운터
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            count <= 0;
        end else begin
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
                period_set = turn_cnt;
                dir_set = 2'b01;
            end
            `RC_BACKWARD: begin  // 후진
                period_set = forward_back_cnt;
                dir_set = 2'b10;
            end
            `RC_LEFT: begin  // 좌회전 (제자리)
                period_set = 0;
                dir_set = 2'b00;
            end
            `RC_RIGHT: begin  // 우회전 (제자리)
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
            default: begin
                period_set = 0;
                dir_set = 2'b00;
            end
        endcase
    end

    // ===== 부드러운 가속/감속 로직 =====
    // PWM 주기(0.2ms)마다 current_period를 period_set으로 조금씩 이동
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            current_period <= 0;
            current_dir    <= 2'b00;
        end else begin
            // PWM 주기 시작 시점에만 업데이트 (0.2ms마다)
            if (count == 0) begin

                // ===== 방향 전환 안전 로직 =====
                // 전진 ↔ 후진 전환 시: 먼저 정지(00) 거쳐가기
                if (dir_set != current_dir && 
                    dir_set != 2'b00 && 
                    current_dir != 2'b00) begin
                    // 방향이 다르고 둘 다 움직이는 상태 → 먼저 정지
                    if (current_period > 0) begin
                        // 속도를 0으로 감속
                        if (current_period > MAX_DECEL_PER_CYCLE) begin
                            current_period <= current_period - MAX_DECEL_PER_CYCLE;
                        end else begin
                            current_period <= 0;
                        end
                    end else begin
                        // 속도가 0이 됨 → 방향 전환 가능
                        current_dir <= dir_set;
                    end
                end  // ===== 정상 가속/감속 =====
                else begin
                    // 방향 업데이트 (같은 방향이거나 정지 방향)
                    current_dir <= dir_set;

                    // 속도 업데이트
                    if (period_set > current_period) begin
                        // 가속: 목표가 현재보다 큼
                        if (period_set - current_period > MAX_ACCEL_PER_CYCLE) begin
                            current_period <= current_period + MAX_ACCEL_PER_CYCLE;
                        end else begin
                            current_period <= period_set;
                        end
                    end else if (period_set < current_period) begin
                        // 감속: 목표가 현재보다 작음
                        if (current_period - period_set > MAX_DECEL_PER_CYCLE) begin
                            current_period <= current_period - MAX_DECEL_PER_CYCLE;
                        end else begin
                            current_period <= period_set;
                        end
                    end
                    // period_set == current_period 이면 변경 없음
                end
            end
        end
    end

    // PWM 및 방향 출력
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pwm_dc <= 0;
            dir_dc <= 0;
        end else begin
            pwm_dc <= (count < current_period);
            dir_dc <= current_dir;  // 부드럽게 변경된 방향 사용
        end
    end

endmodule
