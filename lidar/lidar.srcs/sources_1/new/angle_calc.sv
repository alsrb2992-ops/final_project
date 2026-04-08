module angle_calc (
    input logic clk,
    input logic rst_n,

    input logic [15:0] fsa_raw,
    input logic [15:0] lsa_raw,
    input logic [ 7:0] lsn,
    input logic        fsa_lsa_valid,
    input logic        si_valid,
    input logic        pkt_start,

    output logic [8:0] angle_deg,
    output logic       angle_valid
);

    // --------------------------------------------------
    // 기본 각도 계산
    // --------------------------------------------------
    wire [8:0] angle_fsa = fsa_raw[15:7];
    wire [8:0] angle_lsa = lsa_raw[15:7];

    wire [8:0] raw_diff = angle_lsa - angle_fsa;
    wire [8:0] angle_diff = (angle_lsa >= angle_fsa) ?
                             raw_diff :
                             (9'd360 - angle_fsa + angle_lsa);

    logic [8:0] angle_diff_reg;
    // --------------------------------------------------
    // LUT 출력
    // --------------------------------------------------
    logic [15:0] recip_q16;

    always_comb begin
        case (lsn - 1)
            8'd1: recip_q16 = 16'd65535;
            8'd2: recip_q16 = 16'd32768;
            8'd3: recip_q16 = 16'd21845;
            8'd4: recip_q16 = 16'd16384;
            8'd5: recip_q16 = 16'd13107;
            // ...
            8'd254: recip_q16 = 16'd258;
            default: recip_q16 = 16'd0;
        endcase
    end

    // --------------------------------------------------
    // 내부 레지스터
    // --------------------------------------------------
    logic [14:0] step_q6;
    logic [14:0] accum_q6;

    logic lat_lsn1;
    logic [8:0] lat_fsa;

    // 곱셈 파이프라인용
    logic [30:0] mult_tmp;   // (diff<<6)=15bit × recip(16bit) → 31bit 필요하지만 최적화 가능

    logic fsa_lsa_valid_1;
    logic [8:0] lsn_reg;
    // --------------------------------------------------
    // 메인 로직
    // --------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step_q6        <= '0;
            accum_q6       <= '0;
            lat_lsn1       <= '0;
            lat_fsa        <= '0;
            angle_deg      <= '0;
            angle_valid    <= '0;
            mult_tmp       <= '0;
            angle_diff_reg <= 0;
            lsn_reg        <= 0;
        end else begin
            angle_valid <= 1'b0;


            // ------------------------------------------
            // STEP 계산 (division 제거!)
            // ------------------------------------------
            if (fsa_lsa_valid) begin
                lat_fsa <= angle_fsa;
                lsn_reg <= lsn;
                angle_diff_reg <= angle_diff;

                fsa_lsa_valid_1 <= 1;


                if (lsn == 8'd1) begin
                    lat_lsn1 <= 1;
                end else begin
                    // 1-cycle pipeline (timing 안정)
                    lat_lsn1 <= 0;
                end

                accum_q6 <= {angle_fsa, 6'b0};
            end

            if (fsa_lsa_valid_1) begin

                fsa_lsa_valid_1 <= 0;

                if (lsn_reg == 8'd1) begin
                    step_q6 <= 0;
                end else begin
                    // 1-cycle pipeline (timing 안정)
                    mult_tmp <= {angle_diff_reg, 6'b0} / (lsn_reg - 1);
                    step_q6  <= mult_tmp[30:16];  // >>16
                end
            end

            // ------------------------------------------
            // 각도 출력
            // ------------------------------------------
            if (si_valid) begin
                if (lat_lsn1) begin
                    angle_deg <= lat_fsa;
                end else begin
                    angle_deg <= accum_q6[14:6];
                    accum_q6  <= accum_q6 + step_q6;
                end

                angle_valid <= 1'b1;
            end
        end
    end

endmodule


