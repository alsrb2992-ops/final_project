`timescale 1ns / 1ps

module packet_builder (
    input  wire        clk,
    input  wire        rst,
    // Events
    input  wire        rpi_event,
    input  wire        rpi_active,
    input  wire        cnn_event,
    input  wire [7:0]  cnn_data,
    input  wire        sht_event,
    input  wire [31:0] sht_data,
    // UART interface
    input  wire        tx_busy,
    input  wire        tx_done,
    output reg         tx_start,
    output reg  [7:0]  tx_byte
);

    localparam [3:0] PB_IDLE    = 4'd0;
    localparam [3:0] PB_STX1   = 4'd1;
    localparam [3:0] PB_STX2   = 4'd2;
    localparam [3:0] PB_TYPE   = 4'd3;
    localparam [3:0] PB_LEN    = 4'd4;
    localparam [3:0] PB_PAYLOAD = 4'd5;
    localparam [3:0] PB_CS     = 4'd6;
    localparam [3:0] PB_WAIT   = 4'd7;

    reg [3:0] state, next_state;
    reg [7:0] p_type, p_len, p_cs;
    reg [7:0] p_payload [0:3];
    reg [2:0] p_idx;

    // Internal Pending Registers
    reg        r_pend, c_pend, s_pend;
    reg [7:0]  c_data_reg;
    reg [31:0] s_data_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state    <= PB_IDLE;
            r_pend   <= 1'b0;
            c_pend   <= 1'b0;
            s_pend   <= 1'b0;
            tx_start <= 1'b0;
            p_idx    <= 3'd0;
        end else begin
            tx_start <= 1'b0;
            if (rpi_event) r_pend <= 1'b1;
            if (cnn_event) begin c_pend <= 1'b1; c_data_reg <= cnn_data; end
            if (sht_event) begin s_pend <= 1'b1; s_data_reg <= sht_data; end

            case (state)
                PB_IDLE: begin
                    p_idx <= 3'd0;
                    if (c_pend) begin // CNN Priority 1
                        p_type      <= 8'h02; p_len <= 8'h01; p_payload[0] <= c_data_reg;
                        p_cs        <= 8'h02 ^ 8'h01 ^ c_data_reg;
                        c_pend      <= 1'b0; state <= PB_STX1;
                    end else if (r_pend) begin // RPi Priority 2
                        p_type      <= 8'h01; p_len <= 8'h01; p_payload[0] <= 8'h01;
                        p_cs        <= 8'h01 ^ 8'h01 ^ 8'h01;
                        r_pend      <= 1'b0; state <= PB_STX1;
                    end else if (s_pend) begin // SHT40 Priority 3
                        p_type       <= 8'h03; p_len <= 8'h04;
                        p_payload[0] <= s_data_reg[31:24]; p_payload[1] <= s_data_reg[23:16];
                        p_payload[2] <= s_data_reg[15:8];  p_payload[3] <= s_data_reg[7:0];
                        p_cs <= 8'h03 ^ 8'h04 ^ s_data_reg[31:24] ^ s_data_reg[23:16]
                                       ^ s_data_reg[15:8] ^ s_data_reg[7:0];
                        s_pend <= 1'b0; state <= PB_STX1;
                    end
                end
                PB_STX1: if (!tx_busy) begin tx_byte <= 8'hAB; tx_start <= 1'b1; next_state <= PB_STX2;    state <= PB_WAIT; end
                PB_STX2: if (!tx_busy) begin tx_byte <= 8'hCD; tx_start <= 1'b1; next_state <= PB_TYPE;    state <= PB_WAIT; end
                PB_TYPE: if (!tx_busy) begin tx_byte <= p_type; tx_start <= 1'b1; next_state <= PB_LEN;    state <= PB_WAIT; end
                PB_LEN:  if (!tx_busy) begin tx_byte <= p_len;  tx_start <= 1'b1; next_state <= PB_PAYLOAD; state <= PB_WAIT; end
                PB_PAYLOAD: if (!tx_busy) begin
                    tx_byte  <= p_payload[p_idx]; tx_start <= 1'b1;
                    if (p_idx == p_len - 1) next_state <= PB_CS;
                    else begin p_idx <= p_idx + 1'b1; next_state <= PB_PAYLOAD; end
                    state <= PB_WAIT;
                end
                PB_CS:  if (!tx_busy) begin tx_byte <= p_cs; tx_start <= 1'b1; next_state <= PB_IDLE; state <= PB_WAIT; end
                PB_WAIT: if (tx_done) state <= next_state;
                default: state <= PB_IDLE;
            endcase
        end
    end
endmodule