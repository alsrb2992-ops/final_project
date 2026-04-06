// ============================================================
// packet_sync.sv
// AA 55 헤더 탐색 및 패킷 동기화 모듈
// ============================================================
module packet_sync (
    input  logic        clk,
    input  logic        rst_n,

    // uart_rx 로부터
    input  logic [7:0]  rx_data,
    input  logic        rx_valid,

    // 하위 모듈로
    output logic [7:0]  byte_out,
    output logic        byte_valid,    // 헤더 이후 바이트만 통과
    output logic        pkt_start      // AA 55 감지 시 1클럭 펄스
);

// AA 55 순서로 들어옴 (little-endian: AA 먼저, 55 나중)
typedef enum logic [1:0] {
    WAIT_AA = 2'b00,
    WAIT_55 = 2'b01,
    PASS    = 2'b10
} sync_state_t;

sync_state_t state;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= WAIT_AA;
        byte_out    <= '0;
        byte_valid  <= '0;
        pkt_start   <= '0;
    end else begin
        byte_valid <= '0;
        pkt_start  <= '0;

        if (rx_valid) begin
            case (state)
                // ----------------------------------------------
                WAIT_AA: begin
                    if (rx_data == 8'hAA)
                        state <= WAIT_55;
                end

                // ----------------------------------------------
                WAIT_55: begin
                    if (rx_data == 8'h55) begin
                        state     <= PASS;
                        pkt_start <= 1'b1;   // 헤더 감지 완료
                    end else if (rx_data == 8'hAA) begin
                        state <= WAIT_55;    // AA AA 55 케이스 처리
                    end else begin
                        state <= WAIT_AA;    // 헤더 아님, 재탐색
                    end
                end

                // ----------------------------------------------
                // 헤더 이후 바이트 통과
                // 다음 AA 가 오면 새 패킷 시작
                PASS: begin
                    if (rx_data == 8'hAA) begin
                        state <= WAIT_55;    // 다음 패킷 헤더
                    end else begin
                        byte_out   <= rx_data;
                        byte_valid <= 1'b1;
                    end
                end
            endcase
        end
    end
end

endmodule
