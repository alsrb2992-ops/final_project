`include "define.vh"

module command_decoder (
    input logic       clk,
    input logic       reset_n,
    input logic       rx_done,
    input logic [7:0] rx_data,
    input logic       auto_mode,

    input logic [2:0] auto_direction, // 거리에 따라 자동으로 제어할 때 사용할 입력
    output logic [3:0] car_control
);

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            car_control <= 0;
        end else begin
            car_control <= car_control;
            if (auto_mode) begin
                case (auto_direction)
                    `FORWARD: begin
                        car_control <= 4'b0001;
                    end
                    `BACKWARD: begin
                        car_control <= 4'b0010;
                    end
                    `RIGHT: begin
                        car_control <= 4'b0100;
                    end
                    `LEFT: begin
                        car_control <= 4'b1000;
                    end
                    `STOP: begin
                        car_control <= 0;
                    end
                endcase

            end else begin
                if (rx_done) begin
                    case (rx_data)
                        "F": begin
                            car_control <= 4'b0001;
                        end
                        "B": begin
                            car_control <= 4'b0010;
                        end
                        "R": begin
                            car_control <= 4'b0100;
                        end
                        "L": begin
                            car_control <= 4'b1000;
                        end
                        "S": begin
                            car_control <= 0;
                        end
                    endcase
                end
            end
        end
    end

endmodule
