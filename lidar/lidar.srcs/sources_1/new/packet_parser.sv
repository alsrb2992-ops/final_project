// ============================================================
// packet_parser.sv
// 패킷 필드 파싱 모듈
// AA 55 이후 바이트를 순서대로 파싱
// CT(1) LSN(1) FSA(2) LSA(2) CS(2) Si(2*LSN)
// ============================================================
module packet_parser (
    input logic clk,
    input logic rst_n,

    // packet_sync 로부터
    input logic [7:0] byte_in,
    input logic       byte_valid,
    input logic       pkt_start,   // AA55 감지 펄스

    // 파싱 결과 출력
    output logic        ct_start_bit,  // CT[bit0]: 1=시작패킷
    output logic [ 7:0] lsn,           // 샘플 수
    output logic [15:0] fsa_raw,       // FSA raw (각도 계산용)
    output logic [15:0] lsa_raw,       // LSA raw
    output logic [15:0] cs_rx,         // 수신된 체크섬

    // Si 출력 (1포인트씩)
    output logic [15:0] si_raw,   // 현재 Si raw
    output logic        si_valid, // Si 1개 완성 펄스

    // 패킷 완료
    output logic pkt_done  // 패킷 1개 파싱 완료
);

    // 파싱 상태
    typedef enum logic [3:0] {
        S_CT    = 4'd0,
        S_LSN   = 4'd1,
        S_FSA_L = 4'd2,
        S_FSA_H = 4'd3,
        S_LSA_L = 4'd4,
        S_LSA_H = 4'd5,
        S_CS_L  = 4'd6,
        S_CS_H  = 4'd7,
        S_SI_L  = 4'd8,
        S_SI_H  = 4'd9
    } parse_state_t;

    parse_state_t       state;
    logic         [7:0] si_cnt;  // 현재까지 처리한 Si 개수
    logic         [7:0] si_byte_l;  // Si 하위 바이트 임시 저장

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_CT;
            ct_start_bit <= '0;
            lsn          <= '0;
            fsa_raw      <= '0;
            lsa_raw      <= '0;
            cs_rx        <= '0;
            si_raw       <= '0;
            si_valid     <= '0;
            pkt_done     <= '0;
            si_cnt       <= '0;
            si_byte_l    <= '0;
        end else begin
            si_valid <= '0;
            pkt_done <= '0;

            // 새 패킷 시작 시 상태 리셋
            if (pkt_start) begin
                state  <= S_CT;
                si_cnt <= '0;
            end else if (byte_valid) begin
                case (state)
                    // ------------------------------------------
                    S_CT: begin
                        ct_start_bit <= byte_in[0];  // bit0 만 사용
                        state        <= S_LSN;
                    end

                    // ------------------------------------------
                    S_LSN: begin
                        lsn   <= byte_in;
                        state <= S_FSA_L;
                    end

                    // ------------------------------------------
                    S_FSA_L: begin
                        fsa_raw[7:0] <= byte_in;  // LSB 먼저
                        state        <= S_FSA_H;
                    end

                    S_FSA_H: begin
                        fsa_raw[15:8] <= byte_in;  // MSB
                        state         <= S_LSA_L;
                    end

                    // ------------------------------------------
                    S_LSA_L: begin
                        lsa_raw[7:0] <= byte_in;
                        state        <= S_LSA_H;
                    end

                    S_LSA_H: begin
                        lsa_raw[15:8] <= byte_in;
                        state         <= S_CS_L;
                    end

                    // ------------------------------------------
                    S_CS_L: begin
                        cs_rx[7:0] <= byte_in;
                        state      <= S_CS_H;
                    end

                    S_CS_H: begin
                        cs_rx[15:8] <= byte_in;
                        state       <= S_SI_L;
                    end

                    // ------------------------------------------
                    // Si: 2바이트씩 LSN 개 수신
                    S_SI_L: begin
                        si_byte_l <= byte_in;  // Si LSB
                        state     <= S_SI_H;
                    end

                    S_SI_H: begin
                        si_raw   <= {byte_in, si_byte_l};  // {MSB, LSB}
                        si_valid <= 1'b1;
                        si_cnt   <= si_cnt + 1;

                        if (si_cnt + 1 >= lsn) begin
                            // 모든 Si 수신 완료
                            pkt_done <= 1'b1;
                            state    <= S_CT;  // 다음 패킷 대기
                            si_cnt   <= '0;
                        end else begin
                            state <= S_SI_L;  // 다음 Si
                        end
                    end

                    default: state <= S_CT;
                endcase
            end
        end
    end

endmodule
