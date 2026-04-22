
module rc_car_mode_change (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_done,
    input  wire [7:0] rx_data,
    output reg        auto_mode
);


    localparam MODE_CHANGE = "Y";

    localparam MANUAL_MODE = 0,
               AUTO_MODE = 1;

    reg c_state, n_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_state <= MANUAL_MODE;
        end else begin
            c_state <= n_state;
        end
    end


    always @(*) begin
        n_state   = c_state;
        auto_mode = 0;
        case (c_state)
            MANUAL_MODE: begin
                auto_mode = 0;
                if (rx_done && rx_data == MODE_CHANGE) begin
                    n_state = AUTO_MODE;
                end
            end
            AUTO_MODE: begin
                auto_mode = 1;
                if (rx_done && rx_data == MODE_CHANGE) begin
                    n_state = MANUAL_MODE;
                end
            end
        endcase

    end

endmodule
