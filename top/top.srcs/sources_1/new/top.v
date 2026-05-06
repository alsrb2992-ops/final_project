`timescale 1ns / 1ps

module top #(
    parameter CLK_FREQ = 125_000_000,
    parameter BAUD_RATE = 128_000,  // uart baud_rate
    parameter FRONT_ANGLE_DEG       = 9'd30,                // lidar 0도 기준으로 부터 양방향으로 전방이라 인식하는 각도
    parameter BEHIND_ANGLE_DEG      = 9'd40,                // lidar 0도 기준으로 부터 양방향으로 후방이라 인식하는 각도
    parameter RIGHT_START_ANGLE_DEG = 9'd30,  // 오른쪽 시작 각도
    parameter RIGHT_END_ANGLE_DEG = 9'd90,  // 오른쪽 끝 각도
    parameter LEFT_START_ANGLE_DEG = 9'd270,  // 왼쪽 시작 각도
    parameter LEFT_END_ANGLE_DEG = 9'd330,  // 왼쪽 끝 각도
    parameter BRAKE_DIST_MM         = 14'd300,              // 브레이크 해야하는 인식 거리
    parameter WARN_DIST_MM = 14'd600,  // warning 을 알려주는 거리
    parameter HOLD_MS = 32'd200,  // brake 및 warning 유지 시간
    parameter SIDE_HOLD_MS = 32'd100,  // side warning 유지 시간
    parameter TURN_THRESHOLD_MM = 14'd800,  // 좌우, 최소 인식 거리
    parameter BIG_TURN_DIFF_MM      = 14'd500,       // 좌우 차이에 의한 크게 꺾는 방향 조정 수치        
    parameter SMALL_TURN_DIFF_MM    = 14'd200,       // 좌우 차이에 의한 작게 꺾는 방향 조정 수치 
    parameter MAX_CHANGE = 8,  //  20ms 마다 바뀌는 방향 수치
    parameter MAX_DECEL_PER_CYCLE   = 1000  ,                // 한번에 바뀔 수 있는 dc 수치
    parameter DIR_CHANGE_FREQUENCY  = 250_000             // 좌우 거리 유지하는 시간
) (
    input             sysclk,
    input             reset_n,
    input             lidar_rx,
    input             bluetooth_rx,
    output wire       wifi_tx,
    output wire       pwm_servo,
    output wire       pwm_dc,
    output wire [1:0] dir_dc
);


    wire [2:0] direction_degree_gpio;
    wire rst_n = reset_n; // 리셋 신호는 active low이므로 반전하여 사용
    wire clk = sysclk;
    wire brake_gpio;
    wire warning_led;
    
    lidar_top #(
        .CLK_FREQ             (CLK_FREQ),
        .BAUD_RATE            (BAUD_RATE),
        .FRONT_ANGLE_DEG      (FRONT_ANGLE_DEG),
        .BEHIND_ANGLE_DEG     (BEHIND_ANGLE_DEG),
        .RIGHT_START_ANGLE_DEG(RIGHT_START_ANGLE_DEG),
        .RIGHT_END_ANGLE_DEG  (RIGHT_END_ANGLE_DEG),
        .LEFT_START_ANGLE_DEG (LEFT_START_ANGLE_DEG),
        .LEFT_END_ANGLE_DEG   (LEFT_END_ANGLE_DEG),
        .BRAKE_DIST_MM        (BRAKE_DIST_MM),
        .WARN_DIST_MM         (WARN_DIST_MM),
        .HOLD_MS              (HOLD_MS),
        .SIDE_HOLD_MS         (SIDE_HOLD_MS),
        .TURN_THRESHOLD_MM    (TURN_THRESHOLD_MM),
        .BIG_TURN_DIFF_MM     (BIG_TURN_DIFF_MM),
        .SMALL_TURN_DIFF_MM   (SMALL_TURN_DIFF_MM),
        .DIR_CHANGE_FREQUENCY (DIR_CHANGE_FREQUENCY)
    ) u_lidar_top (
        .clk(clk),
        .rst_n(rst_n),
        .lidar_rx(lidar_rx),
        .wifi_tx(wifi_tx),
        .brake_gpio(brake_gpio),
        .warning_led(warning_led),
        .side_warning_signal_gpio(side_warning_signal_gpio),
        .direction_degree_gpio(direction_degree_gpio)
    );

    RCCar_controll_unit #(
        .CLK_FREQ(CLK_FREQ),
        .MAX_CHANGE(MAX_CHANGE),
        .MAX_DECEL_PER_CYCLE(MAX_DECEL_PER_CYCLE)       // 감속은 최대 변화량 전체
    ) u_RCCar_controll_unit (
        .clk             (clk),
        .reset_n         (rst_n),
        .bluetooth_rx    (bluetooth_rx),
        .stop            (stop),
        .brake_signal    (brake_gpio),
        .warning_signal  (warning_led),
        .direction_degree(direction_degree_gpio),
        .pwm_servo       (pwm_servo),
        .pwm_dc          (pwm_dc),
        .dir_dc          (dir_dc)
    );
endmodule
