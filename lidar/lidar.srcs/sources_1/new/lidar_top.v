// ============================================================
// lidar_top.sv
// YDLIDAR X4PRO 충돌방지 시스템 최상위 모듈
//
// 개선사항:
// 1. interference_filter - Median filter + Range validation
// 2. collision_detector - Hysteresis 추가
// ============================================================
module lidar_top #(
    parameter CLK_FREQ                     = 125_000_000,
    parameter BAUD_RATE                    = 128_000,
    parameter FRONT_ANGLE_DEG              = 9'd45,
    parameter FRONT_SIDE_1_START_ANGLE_DEG = 9'd35,
    parameter FRONT_SIDE_1_END_ANGLE_DEG   = 9'd55,
    parameter FRONT_SIDE_2_START_ANGLE_DEG = 9'd305,
    parameter FRONT_SIDE_2_END_ANGLE_DEG   = 9'd325,
    parameter BEHIND_ANGLE_DEG             = 9'd40,
    parameter RIGHT_START_ANGLE_DEG        = 9'd45,
    parameter RIGHT_END_ANGLE_DEG          = 9'd90,
    parameter LEFT_START_ANGLE_DEG         = 9'd270,
    parameter LEFT_END_ANGLE_DEG           = 9'd315,
    parameter BRAKE_DIST_MM                = 14'd300,
    parameter WARN_DIST_MM                 = 14'd600,
    parameter COUNT_DIST_MM                = 14'd500,
    parameter HYSTERESIS_MM                = 14'd100,
    parameter HOLD_MS                      = 32'd200,
    parameter SIDE_HOLD_MS                 = 32'd100,
    parameter TURN_THRESHOLD_MM            = 14'd800,
    parameter BIG_TURN_DIFF_MM             = 14'd500,
    parameter SMALL_TURN_DIFF_MM           = 14'd200,
    parameter DIR_CHANGE_FREQUENCY         = 2_500_000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       lidar_rx,
    output wire       wifi_tx,
    output wire       brake_gpio,
    output wire       warning_led,
    output wire       side_warning_signal_gpio,
    output wire [2:0] direction_degree_gpio
);

    wire [ 7:0] uart_data;
    wire        uart_valid;
    wire [ 7:0] sync_byte;
    wire        sync_byte_valid;
    wire        sync_pkt_start;
    wire        parser_ct_start;
    wire [ 7:0] parser_lsn;
    wire [15:0] parser_fsa;
    wire [15:0] parser_lsa;
    wire [15:0] parser_cs;
    wire [15:0] parser_si_raw;
    wire        parser_si_valid;
    wire        parser_pkt_done;
    wire        parser_cs_ok;
    wire        parser_fsa_lsa_valid;
    wire [13:0] dist_out;
    wire [ 1:0] dist_is;
    wire        dist_valid;
    wire [ 8:0] angle_out;
    wire        angle_valid;

    wire        side_warning_signal;
    wire [13:0] left_min_distance, right_min_distance;

    // Distance 2클럭 딜레이
    reg [13:0] dist_out_d1, dist_out_d2;
    reg [1:0] dist_is_d1, dist_is_d2;
    reg dist_valid_d1, dist_valid_d2;
    reg cs_ok_d1, cs_ok_d2;

    wire [13:0] filt_dist;
    wire [ 8:0] filt_angle;
    wire        filt_valid;
    wire        brake_signal;
    wire        warning_signal;
    wire        round_done_sig;

    wire [ 2:0] direction_degree;


    wire        is_left_over;
    wire        is_right_over;

    // wire [7:0] w_rx_data, w_rx_rdata, w_tx_rdata;
    // wire w_tx_full, w_rx_empty, w_tx_empty, w_tx_busy;
    // wire tx_ready;

    assign wifi_tx = lidar_rx;

    // ===== UART RX =====
    uart_rx_lidar #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart_rx (
        .clk(clk),
        .rst_n(rst_n),
        .rx(lidar_rx),
        .data(uart_data),
        .valid(uart_valid)
    );


    // ===== Packet Sync & Parser =====
    packet_sync u_sync (
        .clk(clk),
        .rst_n(rst_n),
        .rx_data(uart_data),
        .rx_valid(uart_valid),
        .byte_out(sync_byte),
        .byte_valid(sync_byte_valid),
        .pkt_start(sync_pkt_start)
    );

    packet_parser u_parser (
        .clk(clk),
        .rst_n(rst_n),
        .byte_in(sync_byte),
        .byte_valid(sync_byte_valid),
        .pkt_start(sync_pkt_start),
        .ct_start_bit(parser_ct_start),
        .lsn(parser_lsn),
        .fsa_raw(parser_fsa),
        .lsa_raw(parser_lsa),
        .cs_rx(parser_cs),
        .fsa_lsa_valid(parser_fsa_lsa_valid),
        .si_raw(parser_si_raw),
        .si_valid(parser_si_valid),
        .pkt_done(parser_pkt_done),
        .cs_ok(parser_cs_ok)
    );

    // ===== Distance & Angle Calculation =====
    distance_calc u_dist (
        .clk(clk),
        .rst_n(rst_n),
        .si_raw(parser_si_raw),
        .si_valid(parser_si_valid),
        .distance(dist_out),
        .is_flag(dist_is),
        .calc_valid(dist_valid)
    );

    angle_calc u_angle (
        .clk(clk),
        .rst_n(rst_n),
        .fsa_raw(parser_fsa),
        .lsa_raw(parser_lsa),
        .lsn(parser_lsn),
        .fsa_lsa_valid(parser_fsa_lsa_valid),
        .si_valid(parser_si_valid),
        .pkt_start(sync_pkt_start),
        .angle_deg(angle_out),
        .angle_valid(angle_valid)
    );

    // ===== Distance 2클럭 딜레이 (angle과 동기화) =====
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dist_out_d1   <= 0;
            dist_out_d2   <= 0;
            dist_is_d1    <= 0;
            dist_is_d2    <= 0;
            dist_valid_d1 <= 0;
            dist_valid_d2 <= 0;
            cs_ok_d1      <= 0;
            cs_ok_d2      <= 0;
        end else begin
            dist_out_d1   <= dist_out;
            dist_is_d1    <= dist_is;
            dist_valid_d1 <= dist_valid;
            cs_ok_d1      <= parser_cs_ok;

            dist_out_d2   <= dist_out_d1;
            dist_is_d2    <= dist_is_d1;
            dist_valid_d2 <= dist_valid_d1;
            cs_ok_d2      <= cs_ok_d1;
        end
    end

    // ===== Interference Filter (개선: Median + Range Validation) =====
    interference_filter u_filter (
        .clk(clk),
        .rst_n(rst_n),
        .distance_in(dist_out_d2),
        .is_flag(dist_is_d2),
        .angle_in(angle_out),
        .data_valid(angle_valid),
        .distance_out(filt_dist),
        .angle_out(filt_angle),
        .filtered_valid(filt_valid)
    );

    // ===== Round Detector =====
    round_detector u_round (
        .pkt_start(sync_pkt_start),
        .ct_start_bit(parser_ct_start),
        .round_done(round_done_sig)
    );

    // ===== Collision Detector (개선: 히스테리시스) =====
    collision_detector #(
        .FRONT_ANGLE_DEG             (FRONT_ANGLE_DEG),
        .FRONT_SIDE_1_START_ANGLE_DEG(FRONT_SIDE_1_START_ANGLE_DEG),
        .FRONT_SIDE_1_END_ANGLE_DEG  (FRONT_SIDE_1_END_ANGLE_DEG),
        .FRONT_SIDE_2_START_ANGLE_DEG(FRONT_SIDE_2_START_ANGLE_DEG),
        .FRONT_SIDE_2_END_ANGLE_DEG  (FRONT_SIDE_2_END_ANGLE_DEG),
        .BEHIND_ANGLE_DEG            (BEHIND_ANGLE_DEG),
        .RIGHT_START_ANGLE_DEG       (RIGHT_START_ANGLE_DEG),
        .RIGHT_END_ANGLE_DEG         (RIGHT_END_ANGLE_DEG),
        .LEFT_START_ANGLE_DEG        (LEFT_START_ANGLE_DEG),
        .LEFT_END_ANGLE_DEG          (LEFT_END_ANGLE_DEG),
        .BRAKE_DIST_MM               (BRAKE_DIST_MM),
        .WARN_DIST_MM                (WARN_DIST_MM),
        .COUNT_DIST_MM               (COUNT_DIST_MM),
        .HYSTERESIS_MM               (HYSTERESIS_MM)
    ) u_collision (
        .clk                (clk),
        .rst_n              (rst_n),
        .distance           (filt_dist),
        .angle              (filt_angle),
        .data_valid         (filt_valid),
        .round_done         (round_done_sig),
        .is_left_over       (is_left_over),
        .is_right_over      (is_right_over),
        .brake_signal       (brake_signal),
        .warning_signal     (warning_signal),
        .side_warning_signal(side_warning_signal),
        .left_min_distance  (left_min_distance),
        .right_min_distance (right_min_distance)
    );

    // ===== Left/Right Comparator =====
    left_right_comparator #(
        .CLK_FREQ(CLK_FREQ),
        .DIR_CHANGE_FREQUENCY(DIR_CHANGE_FREQUENCY),
        .TURN_THRESHOLD_MM(TURN_THRESHOLD_MM),
        .BIG_TURN_DIFF_MM(BIG_TURN_DIFF_MM),
        .SMALL_TURN_DIFF_MM(SMALL_TURN_DIFF_MM)
    ) u_left_right_comparator (
        .clk               (clk),
        .rst_n             (rst_n),
        .left_min_distance (left_min_distance),
        .right_min_distance(right_min_distance),
        .is_left_over      (is_left_over),
        .is_right_over     (is_right_over),
        .warning_signal    (warning_signal),
        .direction_degree  (direction_degree)
    );

    // ===== Brake Output =====
    brake_output #(
        .CLK_FREQ(CLK_FREQ),
        .HOLD_MS(HOLD_MS),
        .SIDE_HOLD_MS(SIDE_HOLD_MS)
    ) u_brake (
        .clk(clk),
        .rst_n(rst_n),
        .round_done(round_done_sig),
        .brake_signal(brake_signal),
        .warning_signal(warning_signal),
        .side_warning_signal(side_warning_signal),
        .direction_degree(direction_degree),
        .brake_gpio(brake_gpio),
        .warning_led(warning_led),
        .side_warning_signal_gpio(side_warning_signal_gpio),
        .direction_degree_gpio(direction_degree_gpio)
    );

endmodule
