// ============================================================
// fifo_sync.sv
// 8비트 동기식 FIFO
// DEPTH: 2의 거듭제곱 권장
// ============================================================
module fifo_sync #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH      = 256   // 256바이트
)(
    input  logic                  clk,
    input  logic                  rst_n,

    // Write port
    input  logic [DATA_WIDTH-1:0] wr_data,
    input  logic                  wr_en,
    output logic                  full,

    // Read port
    output logic [DATA_WIDTH-1:0] rd_data,
    input  logic                  rd_en,
    output logic                  empty
);

localparam ADDR_W = $clog2(DEPTH);

logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];
logic [ADDR_W:0]       wr_ptr;   // 1비트 여유 (full/empty 구분)
logic [ADDR_W:0]       rd_ptr;

assign full  = (wr_ptr[ADDR_W] != rd_ptr[ADDR_W]) &&
               (wr_ptr[ADDR_W-1:0] == rd_ptr[ADDR_W-1:0]);
assign empty = (wr_ptr == rd_ptr);

// Write
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_ptr <= '0;
    end else begin
        if (wr_en && !full) begin
            mem[wr_ptr[ADDR_W-1:0]] <= wr_data;
            wr_ptr <= wr_ptr + 1;
        end
    end
end

// Read
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_ptr  <= '0;
        rd_data <= '0;
    end else begin
        if (rd_en && !empty) begin
            rd_data <= mem[rd_ptr[ADDR_W-1:0]];
            rd_ptr  <= rd_ptr + 1;
        end
    end
end

endmodule
