// ============================================================
// lidar_top.sv
// YDLIDAR X4PRO 충돌방지 시스템 최상위 모듈
//
// 모듈 연결 구조:
//
//  [LiDAR UART]
//       │
//  uart_rx         ← UART 수신
//       │
//  packet_sync     ← AA 55 헤더 탐색
//       │
//  packet_parser   ← CT/LSN/FSA/LSA/CS/Si 파싱
//       ├──────────────────────┐
//  distance_calc          angle_calc   ← 거리/각도 계산
//       │                     │
//       └──────┬──────────────┘
//      interference_filter    ← IS 간섭 필터링
//              │
//      collision_detector     ← 충돌 판단
//              │
//      brake_output           ← GPIO 제동 출력
// ============================================================
module lidar_top #(
    parameter CLK_FREQ        = 125_000_000,
    parameter BAUD_RATE       = 128_000,
    parameter FRONT_ANGLE_DEG = 9'd60,
    parameter BRAKE_DIST_MM   = 14'd300,
    parameter HOLD_MS         = 32'd200
) (
    input logic clk,
    input logic rst_n,

    // LiDAR UART
    input logic lidar_rx,

    // 출력
    output logic brake_gpio,
    output logic warning_led
);

    // ============================================================
    // 내부 신호 선언
    // ============================================================

    // uart_rx → packet_sync
    logic [ 7:0] uart_data;
    logic        uart_valid;

    // packet_sync → packet_parser
    logic [ 7:0] sync_byte;
    logic        sync_byte_valid;
    logic        sync_pkt_start;

    // packet_parser → distance_calc / angle_calc
    logic        parser_ct_start;
    logic [ 7:0] parser_lsn;
    logic [15:0] parser_fsa;
    logic [15:0] parser_lsa;
    logic [15:0] parser_cs;
    logic [15:0] parser_si_raw;
    logic        parser_si_valid;
    logic        parser_pkt_done;
    logic        parser_cs_ok;  // CS 검증 결과

    // distance_calc → interference_filter
    logic [13:0] dist_out;
    logic [ 1:0] dist_is;
    logic        dist_valid;

    // angle_calc → interference_filter
    logic [ 8:0] angle_out;
    logic        angle_valid;

    // interference_filter → collision_detector
    logic [13:0] filt_dist;
    logic [ 8:0] filt_angle;
    logic        filt_valid;

    // collision_detector → brake_output
    logic        brake_sig;
    logic        warn_sig;

    // round_detector → collision_detector
    logic        round_done_sig;

    // ============================================================
    // 모듈 인스턴스
    // ============================================================

    // 1. UART 수신
    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart_rx (
        .clk  (clk),
        .rst_n(rst_n),
        .rx   (lidar_rx),
        .data (uart_data),
        .valid(uart_valid)
    );

    // 2. AA 55 헤더 탐색
    packet_sync u_sync (
        .clk       (clk),
        .rst_n     (rst_n),
        .rx_data   (uart_data),
        .rx_valid  (uart_valid),
        .byte_out  (sync_byte),
        .byte_valid(sync_byte_valid),
        .pkt_start (sync_pkt_start)
    );

    // 3. 패킷 파싱
    packet_parser u_parser (
        .clk         (clk),
        .rst_n       (rst_n),
        .byte_in     (sync_byte),
        .byte_valid  (sync_byte_valid),
        .pkt_start   (sync_pkt_start),
        .ct_start_bit(parser_ct_start),
        .lsn         (parser_lsn),
        .fsa_raw     (parser_fsa),
        .lsa_raw     (parser_lsa),
        .cs_rx       (parser_cs),
        .si_raw      (parser_si_raw),
        .si_valid    (parser_si_valid),
        .pkt_done    (parser_pkt_done),
        .cs_ok       (parser_cs_ok)
    );

    // 4. 거리 계산
    distance_calc u_dist (
        .clk       (clk),
        .rst_n     (rst_n),
        .si_raw    (parser_si_raw),
        .si_valid  (parser_si_valid),
        .distance  (dist_out),
        .is_flag   (dist_is),
        .calc_valid(dist_valid)
    );

    // 5. 각도 계산 (정수부)
    angle_calc u_angle (
        .clk        (clk),
        .rst_n      (rst_n),
        .fsa_raw    (parser_fsa),
        .lsa_raw    (parser_lsa),
        .lsn        (parser_lsn),
        .si_valid   (parser_si_valid),
        .pkt_start  (sync_pkt_start),
        .angle_deg  (angle_out),
        .angle_valid(angle_valid)
    );

    // 6. 간섭 필터링
    // CS 불일치 패킷은 si_valid 차단하여 데이터 통과 막음
    // dist_valid 와 angle_valid 가 같은 Si에서 동시에 나옴
    interference_filter u_filter (
        .clk           (clk),
        .rst_n         (rst_n),
        .distance_in   (dist_out),
        .is_flag       (dist_is),
        .angle_in      (angle_out),
        .data_valid    (dist_valid & parser_cs_ok),
        .distance_out  (filt_dist),
        .angle_out     (filt_angle),
        .filtered_valid(filt_valid)
    );

    // 7. 1회전 완료 감지 (CL)
    round_detector u_round (
        .pkt_start   (sync_pkt_start),
        .ct_start_bit(parser_ct_start),
        .round_done  (round_done_sig)
    );

    // 8. 충돌 판단
    collision_detector #(
        .FRONT_ANGLE_DEG(FRONT_ANGLE_DEG),
        .BRAKE_DIST_MM  (BRAKE_DIST_MM)
    ) u_collision (
        .clk           (clk),
        .rst_n         (rst_n),
        .distance      (filt_dist),
        .angle         (filt_angle),
        .data_valid    (filt_valid),
        .round_done    (round_done_sig),
        .brake_signal  (brake_sig),
        .warning_signal(warn_sig)
    );

    // 9. 제동 GPIO 출력
    brake_output #(
        .CLK_FREQ(CLK_FREQ),
        .HOLD_MS (HOLD_MS)
    ) u_brake (
        .clk           (clk),
        .rst_n         (rst_n),
        .brake_signal  (brake_sig),
        .warning_signal(warn_sig),
        .brake_gpio    (brake_gpio),
        .warning_led   (warning_led)
    );

endmodule
