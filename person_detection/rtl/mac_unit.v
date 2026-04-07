// =============================================================================
// mac_unit.v  —  Q4.12 Multiply-Accumulate Unit
// =============================================================================
// Q4.12: signed 16bit (1s+3int+12frac), 범위 -8.0 ~ +7.999755
// a(Q4.12) × w(Q4.12) = product(Q8.24, 32bit) → >>12 → 누산(40bit)
// =============================================================================

module mac_unit (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,

    input  wire signed [15:0] a,
    input  wire signed [15:0] w,

    input  wire signed [39:0] acc_in,
    output reg  signed [39:0] acc_out,
    output wire signed [39:0] acc_comb
);

    wire signed [31:0] product = a * w;
    wire signed [39:0] product_scaled = {{20{product[31]}}, product[31:12]};

    assign acc_comb = acc_in + product_scaled;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  acc_out <= 40'sd0;
        else if (en) acc_out <= acc_comb;
    end

endmodule
