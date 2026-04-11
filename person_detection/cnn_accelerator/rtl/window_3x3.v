// =============================================================================
// window_3x3.v  —  3×3 슬라이딩 윈도우 레지스터 (Q4.12, 16bit)
// =============================================================================

module window_3x3 #(
    parameter DATA_WIDTH = 16
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  in_valid,
    input  wire [DATA_WIDTH-1:0] row0,
    input  wire [DATA_WIDTH-1:0] row1,
    input  wire [DATA_WIDTH-1:0] row2,

    output wire [DATA_WIDTH*9-1:0] window_flat,
    output reg                     out_valid
);

    reg [DATA_WIDTH-1:0] sr0 [0:2];
    reg [DATA_WIDTH-1:0] sr1 [0:2];
    reg [DATA_WIDTH-1:0] sr2 [0:2];
    reg [1:0] col_fill;

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            col_fill  <= 2'd0;
            for (i = 0; i < 3; i = i + 1) begin
                sr0[i] <= 0; sr1[i] <= 0; sr2[i] <= 0;
            end
        end
        else if (in_valid) begin
            sr0[0] <= sr0[1]; sr0[1] <= sr0[2]; sr0[2] <= row0;
            sr1[0] <= sr1[1]; sr1[1] <= sr1[2]; sr1[2] <= row1;
            sr2[0] <= sr2[1]; sr2[1] <= sr2[2]; sr2[2] <= row2;

            if (col_fill < 2'd2)
                col_fill <= col_fill + 1;

            out_valid <= (col_fill == 2'd2) ? 1'b1 : 1'b0;
        end
        else begin
            out_valid <= 1'b0;
        end
    end

    genvar r, c;
    generate
        for (r = 0; r < 3; r = r + 1) begin : ROW
            for (c = 0; c < 3; c = c + 1) begin : COL
                assign window_flat[(r*3+c)*DATA_WIDTH +: DATA_WIDTH] =
                    (r == 0) ? sr0[c] :
                    (r == 1) ? sr1[c] : sr2[c];
            end
        end
    endgenerate

endmodule
