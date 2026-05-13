`include "lidar_define.vh"
`include "RCcar_define.vh"

module command_decoder (
    input wire clk,
    input wire reset_n,
    input wire rx_done,
    input wire [7:0] rx_data,
    input wire auto_mode,
    input wire brake_signal,  // 자동 제어 시 제동 신호 입력
    input wire [2:0] direction_degree, // 거리에 따라 자동으로 제어할 때 사용할 입력
    output reg [3:0] car_control
);

    reg  brake_signal_prev;
    wire brake_siganal_edge = brake_signal & ~brake_signal_prev;

    always @(posedge clk or negedge reset_n) begin
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
                    case (direction_degree)
                        `CENTER: begin
                            car_control <= `RC_BACKWARD_RIGHT;
                        end
                        `TURN_LEFT_SMALL, `TURN_LEFT_BIG: begin
                            car_control <= `RC_BACKWARD_RIGHT;
                        end
                        `TURN_RIGHT_SMALL, `TURN_RIGHT_BIG: begin
                            car_control <= `RC_BACKWARD_LEFT;
                        end
                        default: begin
                            car_control <= `RC_BACKWARD_RIGHT;
                        end
                    endcase
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
                        default: begin
                            car_control <= `RC_FORWARD;
                        end
                    endcase
                end
            end else begin
                if (brake_siganal_edge) begin
                    car_control <= `RC_STOP;
                end else if (rx_done) begin
                    case (rx_data)
                        "S": begin  // 정지
                            car_control <= `RC_STOP;
                        end
                        "F": begin  // 직진
                            if (!brake_signal) car_control <= `RC_FORWARD;
                        end
                        "B": begin  // 후진
                            car_control <= `RC_BACKWARD;
                        end
                        "L": begin  // 좌회전
                            car_control <= `RC_LEFT;
                        end
                        "R": begin  // 우회전
                            car_control <= `RC_RIGHT;
                        end
                        "G": begin  // 직진 + 좌회전
                            if (!brake_signal) car_control <= `RC_FORWARD_LEFT;
                        end
                        "H": begin  // 직진 + 우회전
                            if (!brake_signal) car_control <= `RC_FORWARD_RIGHT;
                        end
                        "I": begin  // 후진 + 좌회전
                            car_control <= `RC_BACKWARD_LEFT;
                        end
                        "J": begin  // 후진 + 우회전
                            car_control <= `RC_BACKWARD_RIGHT;
                        end
                    endcase
                end
            end
        end
    end
endmodule
