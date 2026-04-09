// ============================================================
// lidar_passthrough_top.sv
//
// 구조:
//   lidar_rx ──→ lidar_top_debug (충돌방지)
//                    └──→ report_angle/dist/valid
//                               └──→ angle_distance_reporter
//                                          └──→ pc_tx (1초마다 ASCII)
// ============================================================
module lidar_passthrough_top #(
    parameter CLK_FREQ        = 100_000_000,
    parameter BAUD_RATE       = 128_000,
    parameter PC_BAUD_RATE    = 128_000,
    parameter FRONT_ANGLE_DEG = 9'd80,
    parameter BRAKE_DIST_MM   = 14'd300,
    parameter WARN_DIST_MM    = 14'd400,
    parameter HOLD_MS         = 32'd200
) (
    input logic clk,
    input logic rst,

    input logic lidar_rx,

    output logic pc_tx,  // 1초마다 ASCII 거리 리포트

    output logic brake_gpio,
    output logic warning_led
);

    wire         rst_n = ~rst;

    logic [ 8:0] report_angle;
    logic [13:0] report_dist;
    logic        report_valid;

    // 1. lidar_top_debug (충돌방지 + report 포트)
    lidar_top_debug #(
        .CLK_FREQ       (CLK_FREQ),
        .BAUD_RATE      (BAUD_RATE),
        .FRONT_ANGLE_DEG(FRONT_ANGLE_DEG),
        .BRAKE_DIST_MM  (BRAKE_DIST_MM),
        .WARN_DIST_MM   (WARN_DIST_MM),
        .HOLD_MS        (HOLD_MS)
    ) u_lidar_top (
        .clk         (clk),
        .rst_n       (rst_n),
        .lidar_rx    (lidar_rx),
        .brake_gpio  (brake_gpio),
        .warning_led (warning_led),
        .report_angle(report_angle),
        .report_dist (report_dist),
        .report_valid(report_valid)
    );

    // 2. angle_distance_reporter (1초마다 ASCII → pc_tx)
    angle_distance_reporter #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(PC_BAUD_RATE)
    ) u_reporter (
        .clk       (clk),
        .rst_n     (rst_n),
        .angle_in  (report_angle),
        .dist_in   (report_dist),
        .data_valid(report_valid),
        .tx        (pc_tx)
    );

endmodule
