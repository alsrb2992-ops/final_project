// ============================================================
// lidar_top.sv
// YDLIDAR X4PRO 충돌방지 시스템 최상위 모듈
//
// 개선사항:
// 1. interference_filter - Median filter + Range validation
// 2. collision_detector - Hysteresis 추가
// ============================================================
module lidar_top #(
    parameter CLK_FREQ              = 125_000_000,
    parameter BAUD_RATE             = 128_000,
    parameter FRONT_ANGLE_DEG       = 9'd45,
    parameter BEHIND_ANGLE_DEG      = 9'd40,
    parameter RIGHT_START_ANGLE_DEG = 9'd45,
    parameter RIGHT_END_ANGLE_DEG   = 9'd90,
    parameter LEFT_START_ANGLE_DEG  = 9'd270,
    parameter LEFT_END_ANGLE_DEG    = 9'd315,
    parameter BRAKE_DIST_MM         = 14'd300,
    parameter WARN_DIST_MM          = 14'd600,
    parameter HYSTERESIS_MM         = 14'd100,      // 히스테리시스
    parameter HOLD_MS               = 32'd200,
    parameter SIDE_HOLD_MS          = 32'd100,
    parameter TURN_THRESHOLD_MM     = 14'd800,
    parameter BIG_TURN_DIFF_MM      = 14'd100
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       lidar_rx,
    output logic       wifi_tx,
    output logic       brake_gpio,
    output logic       warning_led,
    output logic       side_warning_signal_gpio,
    output logic [2:0] direction_degree_gpio
);

    logic [ 7:0] uart_data;
    logic        uart_valid;
    logic [ 7:0] sync_byte;
    logic        sync_byte_valid;
    logic        sync_pkt_start;
    logic        parser_ct_start;
    logic [ 7:0] parser_lsn;
    logic [15:0] parser_fsa;
    logic [15:0] parser_lsa;
    logic [15:0] parser_cs;
    logic [15:0] parser_si_raw;
    logic        parser_si_valid;
    logic        parser_pkt_done;
    logic        parser_cs_ok;
    logic        parser_fsa_lsa_valid;
    logic [13:0] dist_out;
    logic [ 1:0] dist_is;
    logic        dist_valid;
    logic [ 8:0] angle_out;
    logic        angle_valid;

    logic        side_warning_signal;
    logic [13:0] left_min_distance, right_min_distance;

    // Distance 2클럭 딜레이
    logic [13:0] dist_out_d1, dist_out_d2;
    logic [1:0] dist_is_d1, dist_is_d2;
    logic dist_valid_d1, dist_valid_d2;
    logic cs_ok_d1, cs_ok_d2;

    logic [13:0] filt_dist;
    logic [ 8:0] filt_angle;
    logic        filt_valid;
    logic        brake_signal;
    logic        warning_signal;
    logic        round_done_sig;

    logic [ 2:0] direction_degree;

    logic [7:0] w_rx_data, w_rx_rdata, w_tx_rdata;
    logic w_tx_full, w_rx_empty, w_tx_empty, w_tx_busy;

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

    // ===== FIFO (WiFi 전송용) =====
    top_fifo U_fifo_rx (
        .clk(clk),
        .reset(rst_n),
        .wdata(uart_data),
        .rd(~w_tx_full),
        .wr(uart_valid),
        .rdata(w_rx_rdata),
        .full(),
        .empty(w_rx_empty)
    );

    top_fifo U_fifo_tx (
        .clk(clk),
        .reset(rst_n),
        .wdata(w_rx_rdata),
        .rd(~w_tx_busy),
        .wr(~w_rx_empty),
        .rdata(w_tx_rdata),
        .full(w_tx_full),
        .empty(w_tx_empty)
    );

    uart_tx_my U_UART_TX (
        .clk(clk),
        .reset(rst_n),
        .tx_data(w_tx_rdata),
        .tx_start(~w_tx_empty),
        .tx(wifi_tx),
        .tx_busy(w_tx_busy),
        .tx_done(tx_done)
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
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dist_out_d1   <= '0;
            dist_out_d2   <= '0;
            dist_is_d1    <= '0;
            dist_is_d2    <= '0;
            dist_valid_d1 <= '0;
            dist_valid_d2 <= '0;
            cs_ok_d1      <= '0;
            cs_ok_d2      <= '0;
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
        .FRONT_ANGLE_DEG      (FRONT_ANGLE_DEG),
        .BEHIND_ANGLE_DEG     (BEHIND_ANGLE_DEG),
        .RIGHT_START_ANGLE_DEG(RIGHT_START_ANGLE_DEG),
        .RIGHT_END_ANGLE_DEG  (RIGHT_END_ANGLE_DEG),
        .LEFT_START_ANGLE_DEG (LEFT_START_ANGLE_DEG),
        .LEFT_END_ANGLE_DEG   (LEFT_END_ANGLE_DEG),
        .BRAKE_DIST_MM        (BRAKE_DIST_MM),
        .WARN_DIST_MM         (WARN_DIST_MM),
        .HYSTERESIS_MM        (HYSTERESIS_MM)
    ) u_collision (
        .clk                (clk),
        .rst_n              (rst_n),
        .distance           (filt_dist),
        .angle              (filt_angle),
        .data_valid         (filt_valid),
        .round_done         (round_done_sig),
        .brake_signal       (brake_signal),
        .warning_signal     (warning_signal),
        .side_warning_signal(side_warning_signal),
        .left_min_distance  (left_min_distance),
        .right_min_distance (right_min_distance)
    );

    // ===== Left/Right Comparator =====
    left_right_comparator #(
        .TURN_THRESHOLD_MM(TURN_THRESHOLD_MM),
        .BIG_TURN_DIFF_MM (BIG_TURN_DIFF_MM)
    ) u_left_right_comparator (
        .left_min_distance (left_min_distance),
        .right_min_distance(right_min_distance),
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
