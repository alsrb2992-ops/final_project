`timescale 1ns / 1ps

module control_unit(
    input clk,
    input reset,
    input wr,
    input rd,
    output full,
    output empty,
    output [3:0] wptr,
    output [3:0] rptr
    );
    
    logic c_full, n_full, c_empty, n_empty;
    logic [3:0] c_wptr, n_wptr, c_rptr, n_rptr;

    assign full = c_full;
    assign empty = c_empty;
    assign wptr = c_wptr;
    assign rptr = c_rptr;

    always_ff @( posedge clk, posedge reset ) begin
        if (reset) begin
            c_full <= 0;
            c_empty <= 1;
            c_wptr <= 0;
            c_rptr <= 0;
        end else begin
            c_full <= n_full;
            c_empty <= n_empty;
            c_wptr <= n_wptr;
            c_rptr <= n_rptr;
        end
    end 

    always_comb begin
        n_empty = c_empty;
        n_full = c_full;
        n_rptr = c_rptr;
        n_wptr = c_wptr;
        case ({wr,rd}) 
            2'b01: begin // pop
                n_full = 0;
                if ( c_empty == 1'b0 ) begin
                    n_rptr = c_rptr + 1;
                    if ( n_rptr == c_wptr ) n_empty = 1'b1;
                end
            end
            2'b10: begin  // push
                n_empty = 0;
                if ( c_full == 1'b0 ) begin
                    n_wptr = c_wptr + 1;
                    if ( n_wptr == c_rptr ) n_full = 1'b1;
                end
            end
            2'b11: begin
                if ( c_full == 1'b1 ) begin // pop
                    n_rptr = c_rptr + 1;
                    n_full = 0;
                    if ( n_rptr == c_wptr ) n_empty = 1'b1;
                end else
                if ( c_empty == 1'b1 ) begin  // push
                    n_wptr = c_wptr + 1;
                    n_empty = 0;
                    if ( n_wptr == c_rptr ) n_full = 1'b1;
                end else begin
                    n_rptr = c_rptr + 1;
                    n_wptr = c_wptr + 1;
                end
            end
        endcase  
    end
endmodule
