// ============================================================
// lidar_top_debug.sv
// lidar_top 과 동일하나 report 포트 3개 추가 (디버그용)
// ============================================================
module lidar_top_debug #(
    parameter CLK_FREQ        = 125_000_000,
    parameter BAUD_RATE       = 128_000,
    parameter FRONT_ANGLE_DEG = 9'd60,
    parameter BRAKE_DIST_MM   = 14'd300,
    parameter WARN_DIST_MM    = 14'd400,
    parameter HOLD_MS         = 32'd200
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        lidar_rx,
    output logic        brake_gpio,
    output logic        warning_led,
    // 디버그용 report 포트
    output logic [ 8:0] report_angle,
    output logic [13:0] report_dist,
    output logic        report_valid
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

    logic [13:0] dist_out_d1, dist_out_d2;
    logic [1:0] dist_is_d1, dist_is_d2;
    logic dist_valid_d1, dist_valid_d2;
    logic cs_ok_d1, cs_ok_d2;

    logic [13:0] filt_dist;
    logic [ 8:0] filt_angle;
    logic        filt_valid;
    logic        brake_sig;
    logic        warn_sig;
    logic        round_done_sig;

    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart_rx (
        .clk(clk),
        .rst_n(rst_n),
        .rx(lidar_rx),
        .data(uart_data),
        .valid(uart_valid)
    );

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

    // dist 2클럭 딜레이 (angle_calc +3클럭에 맞춤)
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

    interference_filter u_filter (
        .clk(clk),
        .rst_n(rst_n),
        .distance_in(dist_out_d2),
        .is_flag(dist_is_d2),
        .angle_in(angle_out),
        .data_valid(angle_valid  /* & cs_ok_d2 */),
        .distance_out(filt_dist),
        .angle_out(filt_angle),
        .filtered_valid(filt_valid)
    );

    round_detector u_round (
        .pkt_start(sync_pkt_start),
        .ct_start_bit(parser_ct_start),
        .round_done(round_done_sig)
    );

    collision_detector #(
        .FRONT_ANGLE_DEG(FRONT_ANGLE_DEG),
        .BRAKE_DIST_MM  (BRAKE_DIST_MM),
        .WARN_DIST_MM   (WARN_DIST_MM)
    ) u_collision (
        .clk(clk),
        .rst_n(rst_n),
        .distance(filt_dist),
        .angle(filt_angle),
        .data_valid(filt_valid),
        .round_done(round_done_sig),
        .brake_signal(brake_sig),
        .warning_signal(warn_sig)
    );

    brake_output #(
        .CLK_FREQ(CLK_FREQ),
        .HOLD_MS (HOLD_MS)
    ) u_brake (
        .clk(clk),
        .rst_n(rst_n),
        .brake_signal(brake_sig),
        .warning_signal(warn_sig),
        .brake_gpio(brake_gpio),
        .warning_led(warning_led)
    );

    assign report_angle = filt_angle;
    assign report_dist  = filt_dist;
    assign report_valid = filt_valid;

endmodule
