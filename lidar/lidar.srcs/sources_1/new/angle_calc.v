module angle_calc (
    input wire clk,
    input wire rst_n,

    input wire [16:0] fsa_raw,
    input wire [16:0] lsa_raw,
    input wire [ 8:0] lsn,
    input wire        fsa_lsa_valid,
    input wire        si_valid,
    input wire        pkt_start,

    output reg [8:0] angle_deg,
    output reg       angle_valid
);

    wire [8:0] angle_fsa = fsa_raw[15:7];
    wire [8:0] angle_lsa = lsa_raw[15:7];

    wire [8:0] raw_diff = angle_lsa - angle_fsa;
    wire [8:0] angle_diff = (angle_lsa >= angle_fsa) ?
                             raw_diff :
                             (9'd360 - angle_fsa + angle_lsa);

    reg [8:0] angle_diff_reg;

    reg [14:0] step_q6;
    reg [14:0] accum_q6;
    reg lat_lsn1;
    reg [8:0] lat_fsa;

    // ↓ 15비트로 축소 (359<<6 = 22976, 15비트면 충분)
    reg [14:0] mult_tmp;

    reg fsa_lsa_valid_1;  // fsa_lsa_valid 다음 클럭 (나눗셈)
    reg fsa_lsa_valid_2;  // 그 다음 클럭 (step_q6 확정)
    reg fsa_lsa_valid_3;  // 그 다음 클럭 (step_q6 확정)
    reg [8:0] lsn_reg;
    reg [8:0] lsn_reg_2;

    reg [31:0] lut_val;
    reg [31:0] lut_val_2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step_q6         <= 0;
            accum_q6        <= 0;
            lat_lsn1        <= 0;
            lat_fsa         <= 0;
            angle_deg       <= 0;
            angle_valid     <= 0;
            mult_tmp        <= 0;
            angle_diff_reg  <= 0;
            lsn_reg         <= 0;
            lsn_reg_2       <= 0;
            fsa_lsa_valid_1 <= 0;
            fsa_lsa_valid_2 <= 0;
            fsa_lsa_valid_3 <= 0;
        end else begin
            angle_valid <= 1'b0;

            // Stage 0: fsa_lsa_valid → 파라미터 래치
            if (fsa_lsa_valid) begin
                lat_fsa        <= angle_fsa;
                lsn_reg        <= lsn;
                angle_diff_reg <= angle_diff;
                lat_lsn1       <= (lsn == 8'd1);
                accum_q6       <= {angle_fsa, 6'b0};
            end

            // 플래그 파이프라인
            fsa_lsa_valid_1 <= fsa_lsa_valid;
            fsa_lsa_valid_2 <= fsa_lsa_valid_1;
            fsa_lsa_valid_3 <= fsa_lsa_valid_2;

            if (fsa_lsa_valid_1) begin
                lut_val_2 <= lut_val;
                lsn_reg_2 <= lsn_reg;
            end

            // Stage 1: 나눗셈 → mult_tmp (15비트)
            // Stage 1: LUT 곱셈 (나눗셈 제거)
            if (fsa_lsa_valid_2) begin
                if (lsn_reg_2 == 8'd1) begin
                    mult_tmp <= 0;
                end else begin
                    mult_tmp <= (angle_diff_reg * lut_val_2) >> 16;
                end
            end

            // Stage 2: mult_tmp 확정 후 step_q6 복사
            // (비블로킹이므로 한 클럭 뒤에야 mult_tmp 값이 확정됨)
            if (fsa_lsa_valid_3) begin
                step_q6 <= mult_tmp;
            end

            // 각도 출력
            if (si_valid) begin
                if (lat_lsn1) begin
                    angle_deg <= lat_fsa;
                end else begin
                    angle_deg <= accum_q6[14:6] > 360 ? accum_q6[14:6] - 360 : accum_q6[14:6];  // 360 도 
                    accum_q6 <= accum_q6 + step_q6;
                end
                angle_valid <= 1'b1;
            end
        end
    end


    always @(*) begin
        case (lsn_reg)
            8'd0, 8'd1: lut_val = 32'd0;

            8'd2: lut_val = 32'd4194304;
            8'd3: lut_val = 32'd2097152;
            8'd4: lut_val = 32'd1398101;
            8'd5: lut_val = 32'd1048576;
            8'd6: lut_val = 32'd838860;
            8'd7: lut_val = 32'd699050;
            8'd8: lut_val = 32'd599186;
            8'd9: lut_val = 32'd524288;
            8'd10: lut_val = 32'd466033;
            8'd11: lut_val = 32'd419430;
            8'd12: lut_val = 32'd381300;
            8'd13: lut_val = 32'd349525;
            8'd14: lut_val = 32'd322638;
            8'd15: lut_val = 32'd299593;
            8'd16: lut_val = 32'd279620;
            8'd17: lut_val = 32'd262144;
            8'd18: lut_val = 32'd246723;
            8'd19: lut_val = 32'd233016;
            8'd20: lut_val = 32'd220752;
            8'd21: lut_val = 32'd209715;
            8'd22: lut_val = 32'd199728;
            8'd23: lut_val = 32'd190650;
            8'd24: lut_val = 32'd182361;
            8'd25: lut_val = 32'd174762;
            8'd26: lut_val = 32'd167772;
            8'd27: lut_val = 32'd161319;
            8'd28: lut_val = 32'd155344;
            8'd29: lut_val = 32'd149796;
            8'd30: lut_val = 32'd144631;
            8'd31: lut_val = 32'd139810;
            8'd32: lut_val = 32'd135300;
            8'd33: lut_val = 32'd131072;
            8'd34: lut_val = 32'd127101;
            8'd35: lut_val = 32'd123361;
            8'd36: lut_val = 32'd119837;
            8'd37: lut_val = 32'd116508;
            8'd38: lut_val = 32'd113359;
            8'd39: lut_val = 32'd110376;
            8'd40: lut_val = 32'd107546;
            8'd41: lut_val = 32'd104857;
            default: lut_val = 32'd0;
        endcase
    end

endmodule
