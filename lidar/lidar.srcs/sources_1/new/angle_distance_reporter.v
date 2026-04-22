// ============================================================
// angle_distance_reporter.v
//
// 각도 라벨은 parameter 로 완전 고정
// 거리값은 data_valid 마다 4단계 파이프라인으로 변환
// 1초마다 미리 변환된 ASCII 버퍼를 바로 UART 전송
// ============================================================
module angle_distance_reporter #(
    parameter CLK_FREQ  = 125_000_000,
    parameter BAUD_RATE = 128_000
) (
    input wire clk,
    input wire rst_n,

    input wire [ 8:0] angle_in,
    input wire [13:0] dist_in,
    input wire        data_valid,

    output wire tx
);

    localparam NUM_ANG = 6;

    // ============================================================
    // 감시 각도 (initial 블록으로 초기화)
    // ============================================================
    reg [8:0] ANG[0:NUM_ANG-1];
    initial begin
        ANG[0] = 9'd0;
        ANG[1] = 9'd45;
        ANG[2] = 9'd90;
        ANG[3] = 9'd180;
        ANG[4] = 9'd270;
        ANG[5] = 9'd315;
    end

    // ============================================================
    // 각도 라벨 ROM: "D000:" "D045:" 등
    // 2D localparam 배열 → 개별 1D 배열 + function 으로 대체
    // ============================================================
    reg [7:0] ANG_LABEL_0[0:4];
    reg [7:0] ANG_LABEL_1[0:4];
    reg [7:0] ANG_LABEL_2[0:4];
    reg [7:0] ANG_LABEL_3[0:4];
    reg [7:0] ANG_LABEL_4[0:4];
    reg [7:0] ANG_LABEL_5[0:4];

    initial begin
        // "D000:"
        ANG_LABEL_0[0] = 8'h44;
        ANG_LABEL_0[1] = 8'h30;
        ANG_LABEL_0[2] = 8'h30;
        ANG_LABEL_0[3] = 8'h30;
        ANG_LABEL_0[4] = 8'h3A;
        // "D045:"
        ANG_LABEL_1[0] = 8'h44;
        ANG_LABEL_1[1] = 8'h30;
        ANG_LABEL_1[2] = 8'h34;
        ANG_LABEL_1[3] = 8'h35;
        ANG_LABEL_1[4] = 8'h3A;
        // "D090:"
        ANG_LABEL_2[0] = 8'h44;
        ANG_LABEL_2[1] = 8'h30;
        ANG_LABEL_2[2] = 8'h39;
        ANG_LABEL_2[3] = 8'h30;
        ANG_LABEL_2[4] = 8'h3A;
        // "D180:"
        ANG_LABEL_3[0] = 8'h44;
        ANG_LABEL_3[1] = 8'h31;
        ANG_LABEL_3[2] = 8'h38;
        ANG_LABEL_3[3] = 8'h30;
        ANG_LABEL_3[4] = 8'h3A;
        // "D270:"
        ANG_LABEL_4[0] = 8'h44;
        ANG_LABEL_4[1] = 8'h32;
        ANG_LABEL_4[2] = 8'h37;
        ANG_LABEL_4[3] = 8'h30;
        ANG_LABEL_4[4] = 8'h3A;
        // "D315:"
        ANG_LABEL_5[0] = 8'h44;
        ANG_LABEL_5[1] = 8'h33;
        ANG_LABEL_5[2] = 8'h31;
        ANG_LABEL_5[3] = 8'h35;
        ANG_LABEL_5[4] = 8'h3A;
    end

    // ANG_LABEL 접근 함수 (2D localparam 배열 대체)
    function [7:0] get_ang_label;
        input [2:0] ang_idx;
        input [2:0] byte_idx;
        begin
            case (ang_idx)
                3'd0: get_ang_label = ANG_LABEL_0[byte_idx];
                3'd1: get_ang_label = ANG_LABEL_1[byte_idx];
                3'd2: get_ang_label = ANG_LABEL_2[byte_idx];
                3'd3: get_ang_label = ANG_LABEL_3[byte_idx];
                3'd4: get_ang_label = ANG_LABEL_4[byte_idx];
                3'd5: get_ang_label = ANG_LABEL_5[byte_idx];
                default: get_ang_label = 8'h20;
            endcase
        end
    endfunction

    // ============================================================
    // 거리 ASCII 버퍼 (각도별 4바이트, 파이프라인으로 갱신)
    // dist_ascii[ang_idx][byte_idx] → 1D로 펼침
    // dist_ascii_flat[ang_idx*4 + byte_idx]
    // ============================================================
    reg [ 7:0] dist_ascii_flat[0:NUM_ANG*4-1];  // 24 entries

    // ============================================================
    // 파이프라인: data_valid → 4클럭 후 dist_ascii 갱신
    //
    // Stage 0: 각도 매칭 → idx/dist 래치
    // Stage 1: dist / 1000  → 천 ASCII
    // Stage 2: rem / 100    → 백 ASCII
    // Stage 3: rem / 10     → 십/일 ASCII → 저장
    // ============================================================

    // Stage 0
    reg [13:0] s0_dist;
    reg [ 2:0] s0_idx;
    reg        s0_valid;

    // Stage 1
    reg [13:0] s1_rem;
    reg [ 7:0] s1_d3;
    reg [ 2:0] s1_idx;
    reg        s1_valid;

    // Stage 2
    reg [13:0] s2_rem;
    reg [7:0] s2_d3, s2_d2;
    reg [2:0] s2_idx;
    reg       s2_valid;

    // Stage 3
    reg [7:0] s3_d3, s3_d2, s3_d1, s3_d0;
    reg [2:0] s3_idx;
    reg       s3_valid;

    // Stage 0: 각도 매칭
    // (for + automatic 제거 → 수동 전개)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_dist  <= 14'd0;
            s0_idx   <= 3'd0;
            s0_valid <= 1'b0;
        end else begin
            s0_valid <= 1'b0;
            if (data_valid && dist_in != 14'd0) begin
                // ANG[0] = 0도: wrap-around 특수 처리
                if (angle_in <= 9'd1 || angle_in >= 9'd359) begin
                    s0_dist  <= dist_in;
                    s0_idx   <= 3'd0;
                    s0_valid <= 1'b1;
                end
                // ANG[1] = 45도
                if (angle_in >= 9'd42 && angle_in <= 9'd48) begin
                    s0_dist  <= dist_in;
                    s0_idx   <= 3'd1;
                    s0_valid <= 1'b1;
                end
                // ANG[2] = 90도
                if (angle_in >= 9'd87 && angle_in <= 9'd93) begin
                    s0_dist  <= dist_in;
                    s0_idx   <= 3'd2;
                    s0_valid <= 1'b1;
                end
                // ANG[3] = 180도
                if (angle_in >= 9'd177 && angle_in <= 9'd183) begin
                    s0_dist  <= dist_in;
                    s0_idx   <= 3'd3;
                    s0_valid <= 1'b1;
                end
                // ANG[4] = 270도
                if (angle_in >= 9'd267 && angle_in <= 9'd273) begin
                    s0_dist  <= dist_in;
                    s0_idx   <= 3'd4;
                    s0_valid <= 1'b1;
                end
                // ANG[5] = 315도
                if (angle_in >= 9'd312 && angle_in <= 9'd318) begin
                    s0_dist  <= dist_in;
                    s0_idx   <= 3'd5;
                    s0_valid <= 1'b1;
                end
            end
        end
    end

    // Stage 1: 천 자리
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_rem   <= 14'd0;
            s1_d3    <= 8'd48;
            s1_idx   <= 3'd0;
            s1_valid <= 1'b0;
        end else begin
            s1_valid <= s0_valid;
            s1_idx   <= s0_idx;
            s1_d3    <= (s0_dist / 14'd1000) + 8'd48;
            s1_rem   <= s0_dist % 14'd1000;
        end
    end

    // Stage 2: 백 자리
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_rem   <= 14'd0;
            s2_d3    <= 8'd48;
            s2_d2    <= 8'd48;
            s2_idx   <= 3'd0;
            s2_valid <= 1'b0;
        end else begin
            s2_valid <= s1_valid;
            s2_idx   <= s1_idx;
            s2_d3    <= s1_d3;
            s2_d2    <= (s1_rem / 14'd100) + 8'd48;
            s2_rem   <= s1_rem % 14'd100;
        end
    end

    // Stage 3: 십/일 자리 → dist_ascii 저장
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_d3    <= 8'd48;
            s3_d2    <= 8'd48;
            s3_d1    <= 8'd48;
            s3_d0    <= 8'd48;
            s3_idx   <= 3'd0;
            s3_valid <= 1'b0;
            for (k = 0; k < NUM_ANG * 4; k = k + 1) begin
                dist_ascii_flat[k] <= 8'd48;
            end
        end else begin
            s3_valid <= s2_valid;
            s3_idx   <= s2_idx;
            s3_d3    <= s2_d3;
            s3_d2    <= s2_d2;
            s3_d1    <= (s2_rem / 14'd10) + 8'd48;
            s3_d0    <= (s2_rem % 14'd10) + 8'd48;

            // 변환 완료 → 해당 슬롯에 저장
            if (s2_valid) begin
                dist_ascii_flat[s2_idx*4+0] <= s2_d3;
                dist_ascii_flat[s2_idx*4+1] <= s2_d2;
                dist_ascii_flat[s2_idx*4+2] <= (s2_rem / 14'd10) + 8'd48;
                dist_ascii_flat[s2_idx*4+3] <= (s2_rem % 14'd10) + 8'd48;
            end
        end
    end

    // ============================================================
    // 1초 타이머
    // ============================================================
    reg [26:0] sec_cnt;
    reg        sec_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sec_cnt  <= 27'd0;
            sec_tick <= 1'b0;
        end else begin
            sec_tick <= 1'b0;
            if (sec_cnt == CLK_FREQ - 1) begin
                sec_cnt  <= 27'd0;
                sec_tick <= 1'b1;
            end else begin
                sec_cnt <= sec_cnt + 27'd1;
            end
        end
    end

    // ============================================================
    // 전송 버퍼 + FSM
    // 61바이트: 10바이트×6 + \n
    // ============================================================
    localparam BUF_LEN = 61;

    reg  [7:0] send_buf  [0:BUF_LEN-1];
    reg  [5:0] send_idx;
    reg        sending;

    reg        tx_valid;
    wire       tx_ready;
    reg  [7:0] tx_data_r;

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

    integer m;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            send_idx  <= 6'd0;
            sending   <= 1'b0;
            tx_valid  <= 1'b0;
            tx_data_r <= 8'd0;
            for (m = 0; m < BUF_LEN; m = m + 1) begin
                send_buf[m] <= 8'd32;
            end
        end else begin
            tx_valid <= 1'b0;

            // sec_tick: 미리 변환된 값 버퍼에 복사
            // (automatic int + for 루프 → 수동 전개)
            if (sec_tick && !sending) begin
                // ANG 0 (offset 0)
                send_buf[0] <= get_ang_label(3'd0, 3'd0);
                send_buf[1] <= get_ang_label(3'd0, 3'd1);
                send_buf[2] <= get_ang_label(3'd0, 3'd2);
                send_buf[3] <= get_ang_label(3'd0, 3'd3);
                send_buf[4] <= get_ang_label(3'd0, 3'd4);
                send_buf[5] <= dist_ascii_flat[0];
                send_buf[6] <= dist_ascii_flat[1];
                send_buf[7] <= dist_ascii_flat[2];
                send_buf[8] <= dist_ascii_flat[3];
                send_buf[9] <= 8'h20;  // space

                // ANG 1 (offset 10)
                send_buf[10] <= get_ang_label(3'd1, 3'd0);
                send_buf[11] <= get_ang_label(3'd1, 3'd1);
                send_buf[12] <= get_ang_label(3'd1, 3'd2);
                send_buf[13] <= get_ang_label(3'd1, 3'd3);
                send_buf[14] <= get_ang_label(3'd1, 3'd4);
                send_buf[15] <= dist_ascii_flat[4];
                send_buf[16] <= dist_ascii_flat[5];
                send_buf[17] <= dist_ascii_flat[6];
                send_buf[18] <= dist_ascii_flat[7];
                send_buf[19] <= 8'h20;

                // ANG 2 (offset 20)
                send_buf[20] <= get_ang_label(3'd2, 3'd0);
                send_buf[21] <= get_ang_label(3'd2, 3'd1);
                send_buf[22] <= get_ang_label(3'd2, 3'd2);
                send_buf[23] <= get_ang_label(3'd2, 3'd3);
                send_buf[24] <= get_ang_label(3'd2, 3'd4);
                send_buf[25] <= dist_ascii_flat[8];
                send_buf[26] <= dist_ascii_flat[9];
                send_buf[27] <= dist_ascii_flat[10];
                send_buf[28] <= dist_ascii_flat[11];
                send_buf[29] <= 8'h20;

                // ANG 3 (offset 30)
                send_buf[30] <= get_ang_label(3'd3, 3'd0);
                send_buf[31] <= get_ang_label(3'd3, 3'd1);
                send_buf[32] <= get_ang_label(3'd3, 3'd2);
                send_buf[33] <= get_ang_label(3'd3, 3'd3);
                send_buf[34] <= get_ang_label(3'd3, 3'd4);
                send_buf[35] <= dist_ascii_flat[12];
                send_buf[36] <= dist_ascii_flat[13];
                send_buf[37] <= dist_ascii_flat[14];
                send_buf[38] <= dist_ascii_flat[15];
                send_buf[39] <= 8'h20;

                // ANG 4 (offset 40)
                send_buf[40] <= get_ang_label(3'd4, 3'd0);
                send_buf[41] <= get_ang_label(3'd4, 3'd1);
                send_buf[42] <= get_ang_label(3'd4, 3'd2);
                send_buf[43] <= get_ang_label(3'd4, 3'd3);
                send_buf[44] <= get_ang_label(3'd4, 3'd4);
                send_buf[45] <= dist_ascii_flat[16];
                send_buf[46] <= dist_ascii_flat[17];
                send_buf[47] <= dist_ascii_flat[18];
                send_buf[48] <= dist_ascii_flat[19];
                send_buf[49] <= 8'h20;

                // ANG 5 (offset 50) - 마지막은 CR
                send_buf[50] <= get_ang_label(3'd5, 3'd0);
                send_buf[51] <= get_ang_label(3'd5, 3'd1);
                send_buf[52] <= get_ang_label(3'd5, 3'd2);
                send_buf[53] <= get_ang_label(3'd5, 3'd3);
                send_buf[54] <= get_ang_label(3'd5, 3'd4);
                send_buf[55] <= dist_ascii_flat[20];
                send_buf[56] <= dist_ascii_flat[21];
                send_buf[57] <= dist_ascii_flat[22];
                send_buf[58] <= dist_ascii_flat[23];
                send_buf[59] <= 8'h0D;  // CR

                // LF
                send_buf[60] <= 8'h0A;

                send_idx <= 6'd0;
                sending <= 1'b1;
            end

            // 순차 전송
            if (sending && tx_ready && !tx_valid) begin
                tx_data_r <= send_buf[send_idx];
                tx_valid  <= 1'b1;
                if (send_idx == BUF_LEN - 1) begin
                    sending  <= 1'b0;
                    send_idx <= 6'd0;
                end else begin
                    send_idx <= send_idx + 6'd1;
                end
            end
        end
    end

endmodule
