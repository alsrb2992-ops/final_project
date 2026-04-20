`include "lidar_define.vh"
`include "RCcar_define.vh"

module command_decoder (
    input logic       clk,
    input logic       reset_n,
    input logic       rx_done,
    input logic [7:0] rx_data,
    input logic       auto_mode,
    input logic       brake_signal, // 자동 제어 시 제동 신호 입력

    input logic [2:0] direction_degree, // 거리에 따라 자동으로 제어할 때 사용할 입력
    output logic [3:0] car_control
);

    logic brake_signal_prev;
    wire  brake_siganal_edge = brake_signal & ~brake_signal_prev;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            brake_signal_prev <= 0;
        end else begin
            brake_signal_prev <= brake_signal;
        end
    end


    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            car_control <= 0;
        end else begin
            car_control <= car_control;

            if (auto_mode) begin
                if (brake_signal) begin
                    car_control <= `RC_BACKWARD;
                end else begin
                    case (direction_degree)
                        `CENTER: begin
                            car_control <= `RC_FORWARD;
                        end
                        `TURN_RIGHT_BIG: begin
                            car_control <= `RC_TURN_RIGHT_BIG;
                        end
                        `TURN_RIGHT_SMALL: begin
                            car_control <= `RC_TURN_RIGHT_SMALL;
                        end
                        `TURN_LEFT_BIG: begin
                            car_control <= `RC_TURN_LEFT_BIG;
                        end
                        `TURN_LEFT_SMALL: begin
                            car_control <= `RC_TURN_LEFT_SMALL;
                        end
                    endcase
                end
            end else begin
                if (brake_siganal_edge) begin
                    car_control <= `RC_STOP;
                end else begin
                    if (rx_done) begin
                        case (rx_data)
                            "F": begin
                                if (~brake_signal) begin
                                    car_control <= `RC_FORWARD;
                                end
                            end
                            "B": begin
                                car_control <= `RC_BACKWARD;
                            end
                            "R": begin
                                car_control <= `RC_TURN_RIGHT_BIG;
                            end
                            "L": begin
                                car_control <= `RC_TURN_LEFT_BIG;
                            end
                            "S": begin
                                car_control <= `RC_STOP;
                            end

                        endcase
                    end
                end
            end
        end
    end

endmodule
