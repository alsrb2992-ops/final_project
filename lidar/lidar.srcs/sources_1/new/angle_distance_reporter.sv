// ============================================================
// angle_distance_reporter.sv
//
// 각도 라벨은 parameter 로 완전 고정
// 거리값은 data_valid 마다 4단계 파이프라인으로 변환
// 1초마다 미리 변환된 ASCII 버퍼를 바로 UART 전송
// ============================================================
module angle_distance_reporter #(
    parameter CLK_FREQ  = 125_000_000,
    parameter BAUD_RATE = 128_000
) (
    input logic clk,
    input logic rst_n,

    input logic [ 8:0] angle_in,
    input logic [13:0] dist_in,
    input logic        data_valid,

    output logic tx
);

    localparam NUM_ANG = 6;

    // 감시 각도
    localparam [8:0] ANG[0:NUM_ANG-1] = '{
        9'd0,
        9'd45,
        9'd90,
        9'd180,
        9'd270,
        9'd315
    };

    // ============================================================
    // 각도 라벨 ROM: parameter 로 완전 고정
    // [각도인덱스][바이트위치] 5바이트: 'D' aaa ':'
    // ============================================================
    localparam [7:0] ANG_LABEL[0:NUM_ANG-1][0:4] = '{
        '{8'h44, 8'h30, 8'h30, 8'h30, 8'h3A},  // "D000:"
        '{8'h44, 8'h30, 8'h34, 8'h35, 8'h3A},  // "D045:"
        '{8'h44, 8'h30, 8'h39, 8'h30, 8'h3A},  // "D090:"
        '{8'h44, 8'h31, 8'h38, 8'h30, 8'h3A},  // "D180:"
        '{8'h44, 8'h32, 8'h37, 8'h30, 8'h3A},  // "D270:"
        '{8'h44, 8'h33, 8'h31, 8'h35, 8'h3A}  // "D315:"
    };

    // ============================================================
    // 거리 ASCII 버퍼 (각도별 4바이트, 파이프라인으로 갱신)
    // ============================================================
    logic [ 7:0] dist_ascii[0:NUM_ANG-1][0:3];

    // ============================================================
    // 파이프라인: data_valid → 4클럭 후 dist_ascii 갱신
    //
    // Stage 0: 각도 매칭 → idx/dist 래치
    // Stage 1: dist / 1000  → 천 ASCII
    // Stage 2: rem / 100    → 백 ASCII
    // Stage 3: rem / 10     → 십/일 ASCII → 저장
    // ============================================================

    // Stage 0
    logic [13:0] s0_dist;
    logic [ 2:0] s0_idx;
    logic        s0_valid;

    // Stage 1
    logic [13:0] s1_rem;
    logic [ 7:0] s1_d3;
    logic [ 2:0] s1_idx;
    logic        s1_valid;

    // Stage 2
    logic [13:0] s2_rem;
    logic [7:0] s2_d3, s2_d2;
    logic [2:0] s2_idx;
    logic       s2_valid;

    // Stage 3
    logic [7:0] s3_d3, s3_d2, s3_d1, s3_d0;
    logic [2:0] s3_idx;
    logic       s3_valid;

    // Stage 0: 각도 매칭
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_dist  <= '0;
            s0_idx   <= '0;
            s0_valid <= '0;
        end else begin
            s0_valid <= '0;
            if (data_valid) begin
                for (int i = 0; i < NUM_ANG; i++) begin
                    if (ANG[i] == 9'd0) begin
                        if (angle_in <= 9'd1 || angle_in >= 9'd359) begin
                            s0_dist  <= dist_in;
                            s0_idx   <= 3'(i);
                            s0_valid <= 1'b1;
                        end
                    end else begin
                        if (angle_in >= ANG[i] - 9'd1 &&
                            angle_in <= ANG[i] + 9'd1) begin
                            s0_dist  <= dist_in;
                            s0_idx   <= 3'(i);
                            s0_valid <= 1'b1;
                        end
                    end
                end
            end
        end
    end

    // Stage 1: 천 자리
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_rem <= '0;
            s1_d3 <= 8'd48;
            s1_idx <= '0;
            s1_valid <= '0;
        end else begin
            s1_valid <= s0_valid;
            s1_idx   <= s0_idx;
            s1_d3    <= 8'(s0_dist / 14'd1000) + 8'd48;
            s1_rem   <= s0_dist % 14'd1000;
        end
    end

    // Stage 2: 백 자리
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_rem <= '0;
            s2_d3 <= 8'd48;
            s2_d2 <= 8'd48;
            s2_idx <= '0;
            s2_valid <= '0;
        end else begin
            s2_valid <= s1_valid;
            s2_idx   <= s1_idx;
            s2_d3    <= s1_d3;
            s2_d2    <= 8'(s1_rem / 14'd100) + 8'd48;
            s2_rem   <= s1_rem % 14'd100;
        end
    end

    // Stage 3: 십/일 자리 → dist_ascii 저장
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_d3 <= 8'd48;
            s3_d2 <= 8'd48;
            s3_d1 <= 8'd48;
            s3_d0 <= 8'd48;
            s3_idx <= '0;
            s3_valid <= '0;
            for (int i = 0; i < NUM_ANG; i++) begin
                dist_ascii[i][0] <= 8'd48;
                dist_ascii[i][1] <= 8'd48;
                dist_ascii[i][2] <= 8'd48;
                dist_ascii[i][3] <= 8'd48;
            end
        end else begin
            s3_valid <= s2_valid;
            s3_idx   <= s2_idx;
            s3_d3    <= s2_d3;
            s3_d2    <= s2_d2;
            s3_d1    <= 8'(s2_rem / 14'd10) + 8'd48;
            s3_d0    <= 8'(s2_rem % 14'd10) + 8'd48;

            // 변환 완료 → 해당 슬롯에 저장
            if (s2_valid) begin
                dist_ascii[s2_idx][0] <= s2_d3;
                dist_ascii[s2_idx][1] <= s2_d2;
                dist_ascii[s2_idx][2] <= 8'(s2_rem / 14'd10) + 8'd48;
                dist_ascii[s2_idx][3] <= 8'(s2_rem % 14'd10) + 8'd48;
            end
        end
    end

    // ============================================================
    // 1초 타이머
    // ============================================================
    logic [26:0] sec_cnt;
    logic        sec_tick;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sec_cnt  <= '0;
            sec_tick <= '0;
        end else begin
            sec_tick <= '0;
            if (sec_cnt == CLK_FREQ - 1) begin
                sec_cnt  <= '0;
                sec_tick <= 1'b1;
            end else sec_cnt <= sec_cnt + 1;
        end
    end

    // ============================================================
    // 전송 버퍼 + FSM
    // 61바이트: 10바이트×6 + \n
    // ============================================================
    localparam BUF_LEN = 61;

    logic [7:0] send_buf  [0:BUF_LEN-1];
    logic [5:0] send_idx;
    logic       sending;

    logic       tx_valid;
    logic       tx_ready;
    logic [7:0] tx_data_r;

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) u_tx (
        .clk  (clk),
        .rst_n(rst_n),
        .data (tx_data_r),
        .valid(tx_valid),
        .ready(tx_ready),
        .tx   (tx)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            send_idx  <= '0;
            sending   <= '0;
            tx_valid  <= '0;
            tx_data_r <= '0;
            for (int i = 0; i < BUF_LEN; i++) send_buf[i] <= 8'd32;
        end else begin
            tx_valid <= '0;

            // sec_tick: 미리 변환된 값 버퍼에 복사
            if (sec_tick && !sending) begin
                for (int i = 0; i < NUM_ANG; i++) begin
                    automatic int b = i * 10;
                    send_buf[b+0] <= ANG_LABEL[i][0];
                    send_buf[b+1] <= ANG_LABEL[i][1];
                    send_buf[b+2] <= ANG_LABEL[i][2];
                    send_buf[b+3] <= ANG_LABEL[i][3];
                    send_buf[b+4] <= ANG_LABEL[i][4];
                    send_buf[b+5] <= dist_ascii[i][0];
                    send_buf[b+6] <= dist_ascii[i][1];
                    send_buf[b+7] <= dist_ascii[i][2];
                    send_buf[b+8] <= dist_ascii[i][3];
                    send_buf[b+9] <= (i == NUM_ANG - 1) ? 8'h0D : 8'h20;
                end
                send_buf[60] <= 8'h0A;
                send_idx <= '0;
                sending <= 1'b1;
            end

            // 순차 전송
            if (sending && tx_ready && !tx_valid) begin
                tx_data_r <= send_buf[send_idx];
                tx_valid  <= 1'b1;
                if (send_idx == BUF_LEN - 1) begin
                    sending  <= 1'b0;
                    send_idx <= '0;
                end else send_idx <= send_idx + 1;
            end
        end
    end

endmodule
