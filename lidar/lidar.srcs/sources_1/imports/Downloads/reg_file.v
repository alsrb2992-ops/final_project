`timescale 1ns / 1ps

module reg_file (
    input        clk,
    input  [7:0] wdata,
    input        wr,
    input        rd,
    input  [3:0] waddr,
    input  [3:0] raddr,
    output [7:0] rdata
);

    reg [7:0] reg_file[0:15];

    always @(posedge clk) begin
        if (wr) begin
            reg_file[waddr] <= wdata;
        end
    end

    assign rdata = reg_file[raddr];

endmodule
