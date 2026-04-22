// ============================================================
// packet_sync.sv
// AA 55 헤더 탐색 및 패킷 동기화 모듈
//
// 수정: PASS 중 0xAA 가 데이터로 나올 수 있음
//   기존: 0xAA 오면 무조건 헤더로 판단 → 데이터 유실
//   수정: 0xAA 뒤에 0x55 가 와야만 새 패킷으로 인식
//         0xAA 뒤에 다른 값 오면 0xAA 도 데이터로 통과
// ============================================================
module packet_sync (
    input wire clk,
    input wire rst_n,

    input wire [7:0] rx_data,
    input wire       rx_valid,

    output reg [7:0] byte_out,
    output reg       byte_valid,
    output reg       pkt_start
);

    localparam WAIT_AA = 3'b000;
    localparam WAIT_55 = 3'b001;
    localparam PASS = 3'b010;
    localparam   PASS_GOT_AA = 3'b011;  // PASS 중 0xAA 받고 다음 바이트 대기

    reg [2:0] state;
    reg [7:0] pending_data;  // 0xAA 뒤에 온 바이트 임시 저장
    reg has_pending;  // pending 유효 플래그

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= WAIT_AA;
            byte_out     <= 0;
            byte_valid   <= 0;
            pkt_start    <= 0;
            pending_data <= 0;
            has_pending  <= 0;
        end else begin
            byte_valid <= 0;
            pkt_start  <= 0;

            // pending 데이터 출력 (rx_valid 없는 클럭에 처리)
            if (has_pending && !rx_valid) begin
                byte_out    <= pending_data;
                byte_valid  <= 1'b1;
                has_pending <= 1'b0;
            end

            if (rx_valid) begin
                case (state)
                    // ------------------------------------------
                    WAIT_AA: begin
                        if (rx_data == 8'hAA) state <= WAIT_55;
                    end

                    // ------------------------------------------
                    WAIT_55: begin
                        if (rx_data == 8'h55) begin
                            state     <= PASS;
                            pkt_start <= 1'b1;
                        end else if (rx_data == 8'hAA) begin
                            state <= WAIT_55;  // AA AA 55 케이스
                        end else begin
                            state <= WAIT_AA;
                        end
                    end

                    // ------------------------------------------
                    // 데이터 통과 중
                    PASS: begin
                        if (rx_data == 8'hAA) begin
                            // 다음 바이트가 0x55 인지 확인 필요
                            state <= PASS_GOT_AA;
                        end else begin
                            byte_out   <= rx_data;
                            byte_valid <= 1'b1;
                        end
                    end

                    // ------------------------------------------
                    // PASS 중 0xAA 를 받은 후 다음 바이트 확인
                    PASS_GOT_AA: begin
                        if (rx_data == 8'h55) begin
                            // AA 55 → 진짜 새 패킷 헤더
                            state     <= PASS;
                            pkt_start <= 1'b1;
                        end else if (rx_data == 8'hAA) begin
                            // AA AA → 앞의 AA 는 데이터로 출력
                            //         뒤의 AA 는 계속 대기
                            byte_out   <= 8'hAA;
                            byte_valid <= 1'b1;
                            state      <= PASS_GOT_AA;
                        end else begin
                            // AA + 일반 데이터 → 둘 다 데이터
                            // AA 먼저 출력, 현재 바이트는 pending
                            byte_out     <= 8'hAA;
                            byte_valid   <= 1'b1;
                            pending_data <= rx_data;
                            has_pending  <= 1'b1;
                            state        <= PASS;
                        end
                    end

                    default: state <= WAIT_AA;
                endcase
            end
        end
    end

endmodule
