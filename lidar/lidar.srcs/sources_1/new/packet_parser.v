// ============================================================
// packet_parser.sv
// 패킷 필드 파싱 모듈
// AA 55 이후 바이트를 순서대로 파싱
// CT(1) LSN(1) FSA(2) LSA(2) CS(2) Si(2*LSN)
//
// CS 검증:
//   CS = PH ^ FSA ^ {LSN,CT} ^ LSA ^ S1 ^ S2 ^ ... ^ Sn
//   pkt_done 시점에 cs_ok 출력
// ============================================================
module packet_parser (
    input wire clk,
    input wire rst_n,

    input wire [7:0] byte_in,
    input wire       byte_valid,
    input wire       pkt_start,

    output reg ct_start_bit,
    output reg [7:0] lsn,
    output reg [15:0] fsa_raw,
    output reg [15:0] lsa_raw,
    output reg [15:0] cs_rx,
    output reg        fsa_lsa_valid, // FSA+LSA 파싱 완료 펄스 (S1 전에 출력)

    output reg [15:0] si_raw,
    output reg        si_valid,

    output reg pkt_done,
    output reg cs_ok      // CS 검증 결과 (pkt_done 과 동시)
);

    localparam [15:0] PH = 16'h55AA;

    localparam S_CT = 4'd0;
    localparam S_LSN = 4'd1;
    localparam S_FSA_L = 4'd2;
    localparam S_FSA_H = 4'd3;
    localparam S_LSA_L = 4'd4;
    localparam S_LSA_H = 4'd5;
    localparam S_CS_L = 4'd6;
    localparam S_CS_H = 4'd7;
    localparam S_SI_L = 4'd8;
    localparam S_SI_H = 4'd9;

    reg [ 3:0] state;
    reg [ 7:0] si_cnt;
    reg [ 7:0] si_byte_l;
    reg [ 7:0] ct_byte;  // CT 원본 저장 (LSN 수신 시 XOR 용)
    reg [15:0] cs_calc;  // CS 누적 계산값

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_CT;
            ct_start_bit  <= 0;
            ct_byte       <= 0;
            lsn           <= 0;
            fsa_raw       <= 0;
            lsa_raw       <= 0;
            cs_rx         <= 0;
            fsa_lsa_valid <= 0;
            si_raw        <= 0;
            si_valid      <= 0;
            pkt_done      <= 0;
            cs_ok         <= 0;
            si_cnt        <= 0;
            si_byte_l     <= 0;
            cs_calc       <= PH;
        end else begin
            si_valid      <= 0;
            pkt_done      <= 0;
            cs_ok         <= 0;
            fsa_lsa_valid <= 0;

            if (pkt_start) begin
                state   <= S_CT;
                si_cnt  <= 0;
                cs_calc <= PH;  // 새 패킷마다 PH 로 초기화
            end else if (byte_valid) begin
                case (state)
                    // ------------------------------------------
                    // CT: 원본 바이트 저장
                    S_CT: begin
                        ct_start_bit <= byte_in[0];
                        ct_byte      <= byte_in;
                        state        <= S_LSN;
                    end

                    // ------------------------------------------
                    // LSN: {LSN, CT} 2바이트 XOR
                    S_LSN: begin
                        lsn     <= byte_in;
                        state   <= S_FSA_L;
                        cs_calc <= cs_calc ^ {byte_in, ct_byte};
                    end

                    // ------------------------------------------
                    // FSA: LSB 먼저 저장, MSB 수신 시 XOR
                    S_FSA_L: begin
                        fsa_raw[7:0] <= byte_in;
                        state        <= S_FSA_H;
                    end

                    S_FSA_H: begin
                        fsa_raw[15:8] <= byte_in;
                        state         <= S_LSA_L;
                        cs_calc       <= cs_calc ^ {byte_in, fsa_raw[7:0]};
                    end

                    // ------------------------------------------
                    // LSA: LSB 먼저 저장, MSB 수신 시 XOR
                    S_LSA_L: begin
                        lsa_raw[7:0] <= byte_in;
                        state        <= S_LSA_H;
                    end

                    S_LSA_H: begin
                        lsa_raw[15:8] <= byte_in;
                        state <= S_CS_L;
                        cs_calc <= cs_calc ^ {byte_in, lsa_raw[7:0]};
                        fsa_lsa_valid <= 1'b1;  // S1 수신 전 각도 계산 시작 신호
                    end

                    // ------------------------------------------
                    // CS: 저장만, XOR 참여 안함
                    S_CS_L: begin
                        cs_rx[7:0] <= byte_in;
                        state      <= S_CS_H;
                    end

                    S_CS_H: begin
                        cs_rx[15:8] <= byte_in;
                        state       <= S_SI_L;
                    end

                    // ------------------------------------------
                    // Si: 2바이트 수신 후 XOR 누적
                    S_SI_L: begin
                        si_byte_l <= byte_in;
                        state     <= S_SI_H;
                    end

                    S_SI_H: begin
                        si_raw   <= {byte_in, si_byte_l};
                        si_valid <= 1'b1;
                        si_cnt   <= si_cnt + 1;

                        // Si XOR 누적
                        cs_calc  <= cs_calc ^ {byte_in, si_byte_l};

                        if (si_cnt + 1 >= lsn) begin
                            pkt_done <= 1'b1;
                            // cs_calc 는 비블로킹이므로
                            // 현재 Si 까지 포함한 최종값을 조합논리로 계산
                            cs_ok  <= ((cs_calc ^ {byte_in, si_byte_l})
                                        == cs_rx);
                            state <= S_CT;
                            si_cnt <= 0;
                        end else begin
                            state <= S_SI_L;
                        end
                    end

                    default: state <= S_CT;
                endcase
            end
        end
    end

endmodule
