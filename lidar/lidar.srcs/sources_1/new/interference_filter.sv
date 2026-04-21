// ============================================================
// interference_filter.sv
// IS 플래그 기반 간섭 포인트 필터링 + 노이즈 제거 강화
//
// 개선사항:
// 1. IS flag 검증 (기존)
// 2. Range validation (50mm~5000mm)
// 3. 3-point median filter (노이즈 제거)
// 4. Outlier rejection (급격한 변화 제거)
// ============================================================
module interference_filter (
    input logic clk,
    input logic rst_n,

    input logic [13:0] distance_in,
    input logic [ 1:0] is_flag,
    input logic [ 8:0] angle_in,
    input logic        data_valid,

    output logic [13:0] distance_out,
    output logic [ 8:0] angle_out,
    output logic        filtered_valid
);

    // ===== 파라미터 =====
    localparam MIN_VALID_DIST = 14'd50;  // 5cm 미만은 오류
    localparam MAX_VALID_DIST = 14'd5000;  // 5m 초과는 오류
    localparam MAX_JUMP = 14'd1000;         // 한 샘플에서 1m 이상 차이나면 의심

    // ===== 1단계: Range Validation =====
    wire range_valid = (distance_in >= MIN_VALID_DIST) && 
                       (distance_in <= MAX_VALID_DIST);

    // ===== 2단계: IS Flag Check =====
    wire is_valid = (is_flag == 2'b00);  // IS == 0만 통과

    // ===== 3단계: Median Filter (3-point) =====
    logic [13:0] buffer[0:2];  // 최근 3개 샘플 저장
    logic [8:0] angle_buffer[0:2];
    logic [1:0] valid_count;  // 유효한 샘플 개수

    // Median 계산 (3개 중 중간값)
    logic [13:0] median_dist;
    always_comb begin
        // 정렬하여 중간값 찾기
        if ((buffer[0] >= buffer[1] && buffer[0] <= buffer[2]) ||
            (buffer[0] >= buffer[2] && buffer[0] <= buffer[1]))
            median_dist = buffer[0];
        else if ((buffer[1] >= buffer[0] && buffer[1] <= buffer[2]) ||
                 (buffer[1] >= buffer[2] && buffer[1] <= buffer[0]))
            median_dist = buffer[1];
        else median_dist = buffer[2];
    end

    // ===== 4단계: Outlier Detection (급격한 변화 제거) =====
    logic [13:0] prev_output;
    wire [13:0] diff = (median_dist > prev_output) ? 
                       (median_dist - prev_output) : 
                       (prev_output - median_dist);
    wire outlier_detected = (diff > MAX_JUMP) && (valid_count >= 2);

    // ===== 메인 로직 =====
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            distance_out    <= '0;
            angle_out       <= '0;
            filtered_valid  <= '0;
            buffer[0]       <= '0;
            buffer[1]       <= '0;
            buffer[2]       <= '0;
            angle_buffer[0] <= '0;
            angle_buffer[1] <= '0;
            angle_buffer[2] <= '0;
            valid_count     <= '0;
            prev_output     <= '0;
        end else begin
            filtered_valid <= '0;

            if (data_valid && is_valid && range_valid) begin
                // 버퍼 시프트 (FIFO)
                buffer[2] <= buffer[1];
                buffer[1] <= buffer[0];
                buffer[0] <= distance_in;

                angle_buffer[2] <= angle_buffer[1];
                angle_buffer[1] <= angle_buffer[0];
                angle_buffer[0] <= angle_in;

                // 유효 카운트 증가 (최대 3)
                if (valid_count < 2'd3) valid_count <= valid_count + 1;

                // 3개 샘플 모였고, outlier가 아니면 출력
                if (valid_count >= 2'd2 && !outlier_detected) begin
                    distance_out <= median_dist;
                    angle_out <= angle_buffer[1];  // 중간 샘플의 각도
                    filtered_valid <= 1'b1;
                    prev_output <= median_dist;
                end
            end
        end
    end

endmodule
