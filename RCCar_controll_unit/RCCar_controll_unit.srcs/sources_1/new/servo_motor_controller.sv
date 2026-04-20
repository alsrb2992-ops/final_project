`timescale 1ns / 1ps

module servo_motor_controller(
    input clk,
    input reset_n,
    input [3:0] car_control,
    output logic pwm_servo
);

    // 1.5ms 가 중간

    logic [$clog2(125000000)-1:0] count;  
    logic [$clog2(125000000)-1:0] period_set;

    localparam pwm_period   = 50;   // Hz  
    localparam center_ms    = 1.5;  // ms     
    localparam left_ms      = 1.0;  // ms     
    localparam right_ms     = 2.0;  // ms   

    localparam pwm_period_cnt   = 125000000 / pwm_period;
    localparam center_cnt       = center_ms * 1000000 / 8;     
    localparam left_cnt         = left_ms * 1000000 / 8;    
    localparam right_cnt        = right_ms * 1000000 / 8;    

    always @(posedge clk or negedge reset_n ) begin
        if(!reset_n) begin
            count <= 0;
        end
        else begin
            if(count > pwm_period_cnt) begin
                count <= 0;
            end
            else begin
                count <= count + 1;
            end
        end
    end

    always_comb begin
        period_set = center_cnt;
        case(car_control)
            4'b0000 : begin // 정지
                period_set = center_cnt;
            end
            4'b0001 : begin // 직진
                period_set = center_cnt;
            end
            4'b0010 : begin // 후진
                period_set = center_cnt;
            end
            4'b0011 : begin // 좌회전
                period_set = left_cnt;
            end
            4'b0100 : begin // 우회전
                period_set = right_cnt;
            end
            4'b0101 : begin // 직진 + 좌회전
                period_set = left_cnt;
            end
            4'b0110 : begin // 직진 + 우회전
                period_set = right_cnt;
            end
            4'b0111 : begin // 후진 + 좌회전
                period_set = left_cnt;
            end
            4'b1000 : begin // 후진 + 우회전
                period_set = right_cnt;
            end
        endcase    
    end

    always_ff @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            pwm_servo <= 0;
        end
        else begin
            pwm_servo <= (count < period_set);
        end
    end

endmodule