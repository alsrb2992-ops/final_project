// ============================================================
// interference_filter.sv
// IS 플래그 기반 간섭 포인트 필터링
//
// IS == 0 : 정상 → 통과
// IS == 2 : 경면반사 → 필터링
// IS == 3 : 주변광 간섭 → 필터링
// Distance == 0 : 측정불가 → 필터링
// ============================================================
module interference_filter (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [13:0] distance_in,
    input  logic [1:0]  is_flag,
    input  logic [8:0]  angle_in,
    input  logic        data_valid,    // distance_calc 의 calc_valid

    output logic [13:0] distance_out,
    output logic [8:0]  angle_out,
    output logic        filtered_valid // 유효 포인트만 통과
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        distance_out   <= '0;
        angle_out      <= '0;
        filtered_valid <= '0;
    end else begin
        filtered_valid <= '0;

        if (data_valid) begin
            // IS == 0 이고 Distance != 0 인 경우만 통과
            if ((is_flag == 2'b00) && (distance_in != 14'd0)) begin
                distance_out   <= distance_in;
                angle_out      <= angle_in;
                filtered_valid <= 1'b1;
            end
            // 나머지는 버림
        end
    end
end

endmodule
