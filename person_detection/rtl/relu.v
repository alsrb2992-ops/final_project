// =============================================================================
// relu.v  —  Q4.12 ReLU + 클리핑
// =============================================================================
// 40bit 누산 → 음수면 0, Q4.12 최대(0x7FFF) 초과면 포화, 그 외 하위 16bit
// =============================================================================

module relu #(
    parameter ACC_WIDTH = 40,
    parameter OUT_WIDTH = 16
) (
    input  wire signed [ACC_WIDTH-1:0] in_acc,
    output wire signed [OUT_WIDTH-1:0] out_data
);

    localparam signed [OUT_WIDTH-1:0] MAX_VAL = {1'b0, {(OUT_WIDTH-1){1'b1}}};

    assign out_data =
        (in_acc[ACC_WIDTH-1])                                       ? {OUT_WIDTH{1'b0}} :
        (in_acc > {{(ACC_WIDTH-OUT_WIDTH){1'b0}}, MAX_VAL})         ? MAX_VAL :
        in_acc[OUT_WIDTH-1:0];

endmodule
