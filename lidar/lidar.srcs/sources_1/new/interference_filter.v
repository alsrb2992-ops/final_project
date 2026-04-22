module interference_filter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [13:0] distance_in,
    input  wire [ 1:0] is_flag,
    input  wire [ 8:0] angle_in,
    input  wire        data_valid,
    output reg  [13:0] distance_out,
    output reg  [ 8:0] angle_out,
    output reg         filtered_valid
);
    // ===== 파라미터 =====
    localparam MIN_VALID_DIST = 14'd50;
    localparam MAX_VALID_DIST = 14'd5000;
    localparam MAX_JUMP = 14'd1000;

    // ===== 1단계: Range & IS Validation =====
    wire range_valid = (distance_in >= MIN_VALID_DIST) && 
                       (distance_in <= MAX_VALID_DIST);
    wire is_valid = (is_flag == 2'b00);

    // ===== 버퍼 =====
    reg [13:0] buffer[0:2];
    reg [8:0] angle_buffer[0:2];
    reg [1:0] valid_count;

    // ===== 파이프라인 Stage 1: Median 계산 (레지스터화) =====
    reg [13:0] median_dist_comb;
    reg [13:0] median_dist_reg;
    reg [8:0] median_angle_reg;
    reg median_valid_reg;
    reg [13:0] min_val, mid_val, max_val;

    // Median 계산 (최적화된 sorting network)
    always @(*) begin

        // 3개 값 정렬 (최적화된 비교)
        if (buffer[0] <= buffer[1]) begin
            if (buffer[1] <= buffer[2]) begin
                // buffer[0] <= buffer[1] <= buffer[2]
                min_val = buffer[0];
                mid_val = buffer[1];
                max_val = buffer[2];
            end else if (buffer[0] <= buffer[2]) begin
                // buffer[0] <= buffer[2] < buffer[1]
                min_val = buffer[0];
                mid_val = buffer[2];
                max_val = buffer[1];
            end else begin
                // buffer[2] < buffer[0] <= buffer[1]
                min_val = buffer[2];
                mid_val = buffer[0];
                max_val = buffer[1];
            end
        end else begin  // buffer[1] < buffer[0]
            if (buffer[0] <= buffer[2]) begin
                // buffer[1] < buffer[0] <= buffer[2]
                min_val = buffer[1];
                mid_val = buffer[0];
                max_val = buffer[2];
            end else if (buffer[1] <= buffer[2]) begin
                // buffer[1] <= buffer[2] < buffer[0]
                min_val = buffer[1];
                mid_val = buffer[2];
                max_val = buffer[0];
            end else begin
                // buffer[2] < buffer[1] < buffer[0]
                min_val = buffer[2];
                mid_val = buffer[1];
                max_val = buffer[0];
            end
        end

        median_dist_comb = mid_val;
    end

    // Stage 1 레지스터
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            median_dist_reg  <= 0;
            median_angle_reg <= 0;
            median_valid_reg <= 0;
        end else begin
            if (valid_count >= 2'd2) begin
                median_dist_reg  <= median_dist_comb;
                median_angle_reg <= angle_buffer[1];
                median_valid_reg <= 1'b1;
            end else begin
                median_valid_reg <= 1'b0;
            end
        end
    end

    // ===== 파이프라인 Stage 2: Outlier Detection =====
    reg [13:0] prev_output;
    reg [13:0] diff;
    reg outlier_detected;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            diff             <= 0;
            outlier_detected <= 0;
        end else begin
            if (median_valid_reg) begin
                // 절댓값 계산 (파이프라인 Stage 2)
                diff <= (median_dist_reg > prev_output) ? 
                        (median_dist_reg - prev_output) : 
                        (prev_output - median_dist_reg);
                outlier_detected <= (diff > MAX_JUMP);
            end else begin
                outlier_detected <= 1'b0;
            end
        end
    end

    // ===== Stage 3: 최종 출력 =====
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            distance_out    <= 0;
            angle_out       <= 0;
            filtered_valid  <= 0;
            buffer[0]       <= 0;
            buffer[1]       <= 0;
            buffer[2]       <= 0;
            angle_buffer[0] <= 0;
            angle_buffer[1] <= 0;
            angle_buffer[2] <= 0;
            valid_count     <= 0;
            prev_output     <= 0;
        end else begin
            filtered_valid <= 0;

            // 입력 버퍼 업데이트 (Stage 0)
            if (data_valid && is_valid && range_valid) begin
                buffer[2] <= buffer[1];
                buffer[1] <= buffer[0];
                buffer[0] <= distance_in;
                angle_buffer[2] <= angle_buffer[1];
                angle_buffer[1] <= angle_buffer[0];
                angle_buffer[0] <= angle_in;

                if (valid_count < 2'd3) valid_count <= valid_count + 1;
            end

            // 최종 출력 (Stage 3)
            if (median_valid_reg && !outlier_detected) begin
                distance_out <= median_dist_reg;
                angle_out <= median_angle_reg;
                filtered_valid <= 1'b1;
                prev_output <= median_dist_reg;
            end
        end
    end

endmodule
