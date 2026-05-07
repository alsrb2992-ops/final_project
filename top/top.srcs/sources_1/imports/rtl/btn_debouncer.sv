`timescale 1ns / 1ps

module btn_debouncer (
    input  clk,
    input  reset,
    input  i_btn,
    output o_btn
);

    // clock divider 
    // 100MHz -> 100kHz
    parameter FCOUNT = 125_000_000 / 100_000;
    // parameter FCOUNT = 100_000_000 / 100_000_00;  // Simulation 용
    logic [$clog2(FCOUNT)-1:0] counter_100kHz;
    logic r_clock_100kHz;

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            counter_100kHz <= 1'b0;
            r_clock_100kHz <= 0;
        end else begin
            if (counter_100kHz == FCOUNT - 1) begin
                counter_100kHz <= 0;
                r_clock_100kHz <= 1'b1;
            end else begin
                counter_100kHz <= counter_100kHz + 1;
                r_clock_100kHz <= 1'b0;
            end
        end
    end

    // debounce 8FF-8input AND gate
    logic [7:0] shift_reg;  //logic == reg 
    logic debounce;

    // 8 SIPO serial input paraller output
    always_ff @(posedge r_clock_100kHz, posedge reset) begin
        if (reset) begin
            shift_reg <= 8'h00;
        end else begin
            shift_reg <= {i_btn, shift_reg[7:1]};
        end
    end
    assign debounce = &{shift_reg};

    logic edge_detect;

    // rising edge detector 
    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            edge_detect <= 1'b0;
        end else begin
            edge_detect <= debounce;
        end
    end

    assign o_btn = debounce & (~edge_detect);

endmodule
