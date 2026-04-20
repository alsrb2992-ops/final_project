module command_decoder(
    input clk,
    input reset_n,
    input rx_done,
    input [7:0] rx_data,
    output reg [3:0] car_control
);

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            car_control <= 0;
        end
        else begin
            car_control <= car_control;
            if(rx_done) begin
                case(rx_data)
                    "S" : begin // 정지
                        car_control <= 4'b0000;
                    end
                    "F" : begin // 직진
                        car_control <= 4'b0001; 
                    end
                    "B" : begin // 후진
                        car_control <= 4'b0010; 
                    end
                    "L" : begin // 좌회전
                        car_control <= 4'b0011; 
                    end
                    "R" : begin // 우회전
                        car_control <= 4'b0100; 
                    end
                    "G" : begin // 직진 + 좌회전
                        car_control <= 4'b0101; 
                    end
                    "H" : begin // 직진 + 우회전
                        car_control <= 4'b0110; 
                    end
                    "I" : begin // 후진 + 좌회전
                        car_control <= 4'b0111; 
                    end
                    "J" : begin // 후진 + 우회전
                        car_control <= 4'b1000; 
                    end
                endcase
            end
        end
    end

endmodule