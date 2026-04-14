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
                    "F" : begin
                        car_control <= 4'b0001; 
                    end
                    "B" : begin
                        car_control <= 4'b0010; 
                    end
                    "R" : begin
                        car_control <= 4'b0100; 
                    end
                    "L" : begin
                        car_control <= 4'b1000; 
                    end
                    "S" : begin
                        car_control <= 0;
                    end
                endcase
            end
        end
    end

endmodule