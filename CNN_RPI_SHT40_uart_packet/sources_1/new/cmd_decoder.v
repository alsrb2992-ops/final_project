`timescale 1ns / 1ps
//=============================================================================
// Module  : cmd_decoder
// Purpose : Parse incoming command bytes received from ESP32/PC via uart_rx.
//
//  Command format: [0xBC][0xDE][CMD][LEN][DATA×LEN][CS]
//  CS  = CMD ^ LEN ^ DATA[0..N-1]
//  Current design supports LEN = 0 or 1 (max 1 payload byte).
//  cmd_valid pulses for 1 cycle when a valid, checksum-correct command arrives.
//=============================================================================
module cmd_decoder (
    input  wire        clk,
    input  wire        rst,
    // uart_rx interface
    input  wire [7:0]  rx_data,
    input  wire        rx_done,
    // decoded command output
    output reg  [7:0]  cmd_out,
    output reg  [7:0]  cmd_payload,
    output reg         cmd_valid
);

    localparam [2:0] CD_IDLE = 3'd0;
    localparam [2:0] CD_HDR2 = 3'd1;
    localparam [2:0] CD_CMD  = 3'd2;
    localparam [2:0] CD_LEN  = 3'd3;
    localparam [2:0] CD_DATA = 3'd4;
    localparam [2:0] CD_CS   = 3'd5;

    reg [2:0] state;
    reg [7:0] cmd_reg;
    reg [7:0] len_reg;
    reg [7:0] data_reg;
    reg [7:0] cs_calc;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= CD_IDLE;
            cmd_out     <= 8'h00;
            cmd_payload <= 8'h00;
            cmd_valid   <= 1'b0;
            cmd_reg     <= 8'h00;
            len_reg     <= 8'h00;
            data_reg    <= 8'h00;
            cs_calc     <= 8'h00;
        end else begin
            cmd_valid <= 1'b0;

            if (rx_done) begin
                case (state)
                    CD_IDLE: begin
                        if (rx_data == 8'hBC) state <= CD_HDR2;
                    end

                    CD_HDR2: begin
                        if      (rx_data == 8'hDE) state <= CD_CMD;
                        else if (rx_data == 8'hBC) state <= CD_HDR2;
                        else                       state <= CD_IDLE;
                    end

                    CD_CMD: begin
                        cmd_reg <= rx_data;
                        cs_calc <= rx_data;
                        state   <= CD_LEN;
                    end

                    CD_LEN: begin
                        len_reg <= rx_data;
                        cs_calc <= cs_calc ^ rx_data;
                        if (rx_data == 8'h00)
                            state <= CD_CS;
                        else
                            state <= CD_DATA;
                    end

                    CD_DATA: begin
                        data_reg <= rx_data;
                        cs_calc  <= cs_calc ^ rx_data;
                        state    <= CD_CS;
                    end

                    CD_CS: begin
                        if (rx_data == cs_calc) begin
                            cmd_out     <= cmd_reg;
                            cmd_payload <= data_reg;
                            cmd_valid   <= 1'b1;
                        end
                        state <= CD_IDLE;
                    end

                    default: state <= CD_IDLE;
                endcase
            end
        end
    end

endmodule