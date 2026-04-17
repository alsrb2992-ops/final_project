
module rc_car_mode_change (
    input  logic clk,
    input  logic rst_n,
    input  logic rx_done,
    input  logic rx_data,
    output logic auto_mode
);


    localparam MODE_CHANGE = "M";

    typedef enum {
        MANUAL_MODE,
        AUTO_MODE
    } state_t;

    state_t c_state, n_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_state <= MANUAL_MODE;
        end else begin
            c_state <= n_state;
        end
    end


    always_comb begin
        n_state = c_state;
        case (c_state)
            MANUAL_MODE: begin
                auto_mode = 0;
                if (rx_done && rx_data == "M") begin
                    n_state = AUTO_MODE;
                end
            end
            AUTO_MODE: begin
                auto_mode = 1;
                if (rx_done && rx_data == "M") begin
                    n_state = MANUAL_MODE;
                end
            end
        endcase

    end

endmodule
