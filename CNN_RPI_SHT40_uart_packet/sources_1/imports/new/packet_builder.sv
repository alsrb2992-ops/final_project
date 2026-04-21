`timescale 1ns / 1ps

module packet_builder (
    input  logic        clk,
    input  logic        rst,
    // Events
    input  logic        rpi_event,
    input  logic        rpi_active,
    input  logic        cnn_event,
    input  logic [7:0]  cnn_data,
    input  logic        sht_event,
    input  logic [31:0] sht_data,
    // UART interface
    input  logic        tx_busy,
    input  logic        tx_done,
    output logic        tx_start,
    output logic [7:0]  tx_byte
);

    typedef enum logic [3:0] {
        PB_IDLE, PB_STX1, PB_STX2, PB_TYPE, PB_LEN, PB_PAYLOAD, PB_CS, PB_WAIT
    } pb_state_t;

    pb_state_t state, next_state;
    logic [7:0] p_type, p_len, p_cs;
    logic [7:0] p_payload [0:3];
    logic [2:0] p_idx;

    // Internal Pending Registers
    logic r_pend, c_pend, s_pend;
    logic [7:0] c_data_reg;
    logic [31:0] s_data_reg;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= PB_IDLE; r_pend <= 0; c_pend <= 0; s_pend <= 0;
            tx_start <= 0; p_idx <= 0;
        end else begin
            tx_start <= 1'b0;
            if (rpi_event) r_pend <= 1'b1;
            if (cnn_event) begin c_pend <= 1'b1; c_data_reg <= cnn_data; end
            if (sht_event) begin s_pend <= 1'b1; s_data_reg <= sht_data; end

            case (state)
                PB_IDLE: begin
                    p_idx <= 0;
                    if (c_pend) begin // CNN Priority 1
                        p_type <= 8'h02; p_len <= 8'h01; p_payload[0] <= c_data_reg;
                        p_cs   <= 8'h02 ^ 8'h01 ^ c_data_reg;
                        c_pend <= 1'b0; state <= PB_STX1;
                    end else if (r_pend) begin // RPi Priority 2
                        p_type <= 8'h01; p_len <= 8'h01; p_payload[0] <= 8'h01; // Active status
                        p_cs   <= 8'h01 ^ 8'h01 ^ 8'h01;
                        r_pend <= 1'b0; state <= PB_STX1;
                    end else if (s_pend) begin // SHT40 Priority 3
                        p_type <= 8'h03; p_len <= 8'h04;
                        p_payload[0] <= s_data_reg[31:24]; p_payload[1] <= s_data_reg[23:16];
                        p_payload[2] <= s_data_reg[15:8];  p_payload[3] <= s_data_reg[7:0];
                        p_cs <= 8'h03 ^ 8'h04 ^ s_data_reg[31:24] ^ s_data_reg[23:16] ^ s_data_reg[15:8] ^ s_data_reg[7:0];
                        s_pend <= 1'b0; state <= PB_STX1;
                    end
                end
                PB_STX1: if(!tx_busy) begin tx_byte <= 8'hAB; tx_start <= 1; next_state <= PB_STX2; state <= PB_WAIT; end
                PB_STX2: if(!tx_busy) begin tx_byte <= 8'hCD; tx_start <= 1; next_state <= PB_TYPE; state <= PB_WAIT; end
                PB_TYPE: if(!tx_busy) begin tx_byte <= p_type; tx_start <= 1; next_state <= PB_LEN; state <= PB_WAIT; end
                PB_LEN:  if(!tx_busy) begin tx_byte <= p_len;  tx_start <= 1; next_state <= PB_PAYLOAD; state <= PB_WAIT; end
                PB_PAYLOAD: if(!tx_busy) begin
                    tx_byte <= p_payload[p_idx]; tx_start <= 1;
                    if (p_idx == p_len - 1) next_state <= PB_CS;
                    else begin p_idx <= p_idx + 1; next_state <= PB_PAYLOAD; end
                    state <= PB_WAIT;
                end
                PB_CS:   if(!tx_busy) begin tx_byte <= p_cs; tx_start <= 1; next_state <= PB_IDLE; state <= PB_WAIT; end
                PB_WAIT: if(tx_done) state <= next_state;
                default: state <= PB_IDLE;
            endcase
        end
    end
endmodule