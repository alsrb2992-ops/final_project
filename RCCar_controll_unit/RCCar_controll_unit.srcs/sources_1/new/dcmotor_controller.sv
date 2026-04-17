`timescale 1ns / 1ps

module dcmotor_controller (
    input              clk,
    input              reset_n,
    input        [3:0] car_control,
    input              stop,
    output logic       pwm_dc,
    output logic [1:0] dir_dc
);
    // 10MHz

    localparam pwm_period = 5000;  // Hz  
    localparam forward_back_ms = 100;  // %     
    localparam turn_ms = 80;  // %     

    localparam pwm_period_cnt = 125000000 / pwm_period;
    localparam forward_back_cnt = (pwm_period_cnt * forward_back_ms) / 100;
    localparam turn_cnt = (pwm_period_cnt * turn_ms) / 100;

    logic [$clog2(125000000)-1:0] count;
    logic [$clog2(125000000)-1:0] period_set;
    logic [1:0] dir_set;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            count <= 0;
        end else begin
            if (count > pwm_period_cnt) count <= 0;
            else count <= count + 1;
        end
    end

    always_comb begin
        period_set = 0;
        dir_set = 2'b00;
        if (!stop) begin
            case (car_control)
                4'b0000: begin  // S
                    period_set = 0;
                    dir_set = 2'b00;
                end
                4'b0001: begin  // F
                    period_set = forward_back_cnt;
                    dir_set = 2'b01;
                end
                4'b0010: begin  // B
                    period_set = forward_back_cnt;
                    dir_set = 2'b10;
                end
                4'b0100: begin  // R
                    period_set = turn_cnt;
                    dir_set = 2'b01;
                end
                4'b1000: begin  // L
                    period_set = turn_cnt;
                    dir_set = 2'b01;
                end
            endcase
        end
    end

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pwm_dc <= 0;
            dir_dc <= 0;
        end else begin
            pwm_dc <= (count < period_set);
            dir_dc <= dir_set;
        end
    end
endmodule
