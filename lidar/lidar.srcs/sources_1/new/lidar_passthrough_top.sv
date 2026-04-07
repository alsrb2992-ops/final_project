// ============================================================
// lidar_passthrough_top.sv
// YDLIDAR X4PRO 수신 데이터를 PC 로 패스스루
//
// 구조:
//   lidar_rx ──┬──→ lidar_top (충돌방지)
//              └──→ FIFO → uart_tx → pc_tx (PC ComPort)
//
// LiDAR UART RX 신호를 lidar_top 과 FIFO 에 동시에 연결
// lidar_top 내부의 uart_rx 가 파싱하는 동시에
// 수신 바이트를 FIFO 에 저장 후 PC 로 송신
// ============================================================
module lidar_passthrough_top #(
    parameter CLK_FREQ        = 100_000_000,  // Zybo Z7: 125MHz
    parameter BAUD_RATE       = 128_000,
    parameter PC_BAUD_RATE    = 128_000,
    parameter FRONT_ANGLE_DEG = 9'd60,        // 전방 ±60° = 총 120°
    parameter BRAKE_DIST_MM   = 14'd300,      // 제동 거리 300mm
    parameter HOLD_MS         = 32'd200,
    parameter FIFO_DEPTH      = 512
) (
    input logic clk,
    // input logic rst_n,
    input logic rst,

    // LiDAR
    input logic lidar_rx,

    // PC UART
    output logic pc_tx,

    // 충돌방지 출력
    output logic brake_gpio,
    output logic warning_led,

    // 상태 표시
    output logic fifo_full_led  // FIFO full 경고
);

    logic rst_n;
    assign rst_n = ~rst;  // active low reset

    // ============================================================
    // 내부 신호
    // ============================================================

    // lidar_top 내부 uart_rx 와 동일하게 수신
    // → uart_rx 를 lidar_top 외부로 꺼내서 공유
    logic [7:0] rx_data;
    logic       rx_valid;

    // FIFO
    logic       fifo_full;
    logic       fifo_empty;
    logic [7:0] fifo_rd_data;
    logic       fifo_rd_en;

    // UART TX
    logic       tx_ready;

    // ============================================================
    // 1. UART RX (LiDAR 수신 공유)
    // ============================================================
    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_lidar_rx (
        .clk  (clk),
        .rst_n(rst_n),
        .rx   (lidar_rx),
        .data (rx_data),
        .valid(rx_valid)
    );

    // ============================================================
    // 2. lidar_top (충돌방지)
    // uart_rx 를 공유하기 위해 lidar_rx 신호 직접 연결
    // lidar_top 내부에서 uart_rx 를 별도로 인스턴스하지만
    // 같은 lidar_rx 를 바라보므로 동일하게 동작
    // ============================================================
    lidar_top #(
        .CLK_FREQ       (CLK_FREQ),
        .BAUD_RATE      (BAUD_RATE),
        .FRONT_ANGLE_DEG(FRONT_ANGLE_DEG),
        .BRAKE_DIST_MM  (BRAKE_DIST_MM),
        .HOLD_MS        (HOLD_MS)
    ) u_lidar_top (
        .clk        (clk),
        .rst_n      (rst_n),
        .lidar_rx   (lidar_rx),
        .brake_gpio (brake_gpio),
        .warning_led(warning_led)
    );

    // ============================================================
    // 3. FIFO (수신 바이트 버퍼링)
    // rx_valid 시 wr_en → FIFO 에 저장
    // full 이면 데이터 드롭 (LiDAR 데이터는 연속 스트림이므로
    //   FIFO full 은 PC 측 baud rate 가 너무 느릴 때 발생)
    // ============================================================
    fifo_sync #(
        .DATA_WIDTH(8),
        .DEPTH     (FIFO_DEPTH)
    ) u_fifo (
        .clk    (clk),
        .rst_n  (rst_n),
        .wr_data(rx_data),
        .wr_en  (rx_valid & ~fifo_full),
        .full   (fifo_full),
        .rd_data(fifo_rd_data),
        .rd_en  (fifo_rd_en),
        .empty  (fifo_empty)
    );

    // ============================================================
    // 4. FIFO → UART TX 컨트롤러
    // FIFO 에 데이터 있고 TX 가 ready 면 읽어서 전송
    // ============================================================
    typedef enum logic [1:0] {
        TX_IDLE = 2'b00,
        TX_READ = 2'b01,
        TX_SEND = 2'b10
    } tx_state_t;

    tx_state_t       tx_state;
    logic      [7:0] tx_data;
    logic            tx_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE;
            tx_data <= '0;
            tx_valid <= '0;
            fifo_rd_en <= '0;
        end else begin
            tx_valid   <= '0;
            fifo_rd_en <= '0;

            case (tx_state)
                TX_IDLE: begin
                    if (!fifo_empty && tx_ready) begin
                        fifo_rd_en <= 1'b1;
                        tx_state   <= TX_READ;
                    end
                end

                TX_READ: begin
                    // FIFO read latency 1클럭 대기
                    tx_data  <= fifo_rd_data;
                    tx_valid <= 1'b1;
                    tx_state <= TX_SEND;
                end

                TX_SEND: begin
                    // TX 가 busy 해질 때까지 대기
                    if (!tx_ready) begin
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // ============================================================
    // 5. UART TX (PC 전송)
    // ============================================================
    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(PC_BAUD_RATE)
    ) u_pc_tx (
        .clk  (clk),
        .rst_n(rst_n),
        .data (tx_data),
        .valid(tx_valid),
        .ready(tx_ready),
        .tx   (pc_tx)
    );

    // ============================================================
    // FIFO full LED
    // ============================================================
    assign fifo_full_led = fifo_full;

endmodule
