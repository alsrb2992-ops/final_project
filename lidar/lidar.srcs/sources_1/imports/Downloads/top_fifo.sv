`timescale 1ns / 1ps

module top_fifo (
    input        clk,
    input        reset,
    input  [7:0] wdata,
    input        wr,
    input        rd,
    output [7:0] rdata,
    output full,
    output empty
);

    wire [3:0] w_wptr, w_rptr;

    reg_file U_reg_file (
        .clk(clk),
        .wdata(wdata),
        .wr(wr && (~full) ),
        .rd(rd),
        .waddr(w_wptr),
        .raddr(w_rptr),
        .rdata(rdata)
    );

    control_unit U_control_unit (
        .clk(clk),
        .reset(reset),
        .wr(wr),
        .rd(rd),
        .full(full),
        .empty(empty),
        .wptr(w_wptr),
        .rptr(w_rptr)
    );
endmodule
