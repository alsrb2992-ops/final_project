`timescale 1ns / 1ps

module sht40_i2c #(
    parameter integer CLK_FREQ = 100_000_000
)(
    input  wire         clk,
    input  wire         rst,
    inout  wire         i2c_sda,
    output reg          i2c_scl,
    output reg  [31:0]  sht_data,  // {Temp_MSB, Temp_LSB, Hum_MSB, Hum_LSB}
    output reg          sht_valid
);
    // ─── 타이머 (1초 주기 트리거) ──────────────────────────────────────────
    localparam integer TIMER_1SEC = CLK_FREQ;
    //localparam integer WAIT_10MS  = CLK_FREQ / 100;   // 10ms
    localparam integer WAIT_10MS  = 400_000 / 100;      // 4_000 (phase_tick 기준, 10ms 정확)

    reg [31:0] sec_cnt;
    reg        trigger_read;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sec_cnt      <= 32'd0;
            trigger_read <= 1'b0;
        end else begin
            trigger_read <= 1'b0;
            if (sec_cnt == TIMER_1SEC - 1) begin
                sec_cnt      <= 32'd0;
                trigger_read <= 1'b1;
            end else
                sec_cnt <= sec_cnt + 1;
        end
    end

    // ─── I2C SCL 클럭 분주 (100 kHz) ──────────────────────────────────────
    // SCL 1주기 = 4 phase: PULL_LOW → LOW_HOLD → RELEASE → HIGH_HOLD
    localparam integer PHASE_DIV = CLK_FREQ / 400_000; // 각 phase 길이

    reg [$clog2(PHASE_DIV)-1:0] phase_cnt;
    reg                          phase_tick;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            phase_cnt  <= 0;
            phase_tick <= 1'b0;
        end else begin
            phase_tick <= 1'b0;
            if (phase_cnt == PHASE_DIV - 1) begin
                phase_cnt  <= 0;
                phase_tick <= 1'b1;
            end else
                phase_cnt <= phase_cnt + 1;
        end
    end

    // ─── FSM 상태 (localparam) ────────────────────────────────────────────
    localparam [3:0]
        S_IDLE        = 4'd0,
        S_START       = 4'd1,
        S_TX_BYTE     = 4'd2,
        S_ACK_W       = 4'd3,
        S_WAIT_MEAS   = 4'd4,
        S_RESTART     = 4'd5,
        S_TX_ADDR_R   = 4'd6,
        S_ACK_R       = 4'd7,
        S_RX_BYTE     = 4'd8,
        S_SEND_ACK    = 4'd9,
        S_SEND_NACK   = 4'd10,
        S_STOP        = 4'd11,
        S_DONE        = 4'd12;

    // SHT40 I2C 주소 0x44, R=1 / W=0
    localparam [7:0] ADDR_W = 8'h88; // 0x44<<1 | 0
    localparam [7:0] ADDR_R = 8'h89; // 0x44<<1 | 1
    localparam [7:0] CMD_HP = 8'hFD; // 고정밀 측정 명령

    reg [3:0]  state;
    reg [2:0]  bit_idx;       // 현재 전송/수신 비트 (7→0)
    reg [3:0]  rx_cnt;        // 수신한 바이트 수
    reg [2:0]  phase;         // SCL 4-phase 카운터 (0~3)
    reg [7:0]  tx_byte;
    reg [7:0]  rx_byte;
    reg [31:0] wait_cnt;

    // SDA open-drain
    reg  sda_out;
    reg  sda_oe;              // 1=드라이브, 0=Hi-Z
    // I2C open-drain: 드라이브할 때는 항상 LOW(I=0), T=0이면 출력, T=1이면 Hi-Z
    wire sda_in;
    wire sda_drive = sda_oe & ~sda_out;  // 실제로 LOW를 구동해야 할 때만 1

    IOBUF sda_iobuf (
        .IO(i2c_sda),       // 물리 핀
        .I (1'b0),          // 출력값은 항상 0 (open-drain이므로)
        .O (sda_in),        // 핀에서 읽은 값
        .T (~sda_drive)     // T=0이면 드라이브, T=1이면 Hi-Z
    );

    // 수신 버퍼: T_H, T_L, CRC_T, H_H, H_L, CRC_H (6바이트)
    reg [7:0] rx_buf [0:5];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= S_IDLE;
            i2c_scl   <= 1'b1;
            sda_out   <= 1'b1;
            sda_oe    <= 1'b1;
            sht_valid <= 1'b0;
            sht_data  <= 32'd0;
            wait_cnt  <= 32'd0;
            rx_cnt    <= 4'd0;
            phase     <= 3'd0;
        end else begin
            sht_valid <= 1'b0;

            if (phase_tick) begin
                case (state)

                    // ── IDLE ──────────────────────────────────────────────
                    S_IDLE: begin
                        i2c_scl <= 1'b1;
                        sda_out <= 1'b1; sda_oe <= 1'b1;
                        if (trigger_read) begin
                            state <= S_START;
                            phase <= 3'd0;
                        end
                    end

                    // ── START condition: SDA 1→0 while SCL=1 ─────────────
                    S_START: begin
                        case (phase)
                            3'd0: begin i2c_scl<=1; sda_out<=1; sda_oe<=1; phase<=3'd1; end
                            3'd1: begin sda_out<=0; phase<=3'd2; end
                            3'd2: begin i2c_scl<=0; phase<=3'd3; end
                            3'd3: begin
                                tx_byte <= ADDR_W;
                                bit_idx <= 3'd7;
                                state   <= S_TX_BYTE;
                                phase   <= 3'd0;
                            end
                            default: phase <= 3'd0;
                        endcase
                    end

                    // ── TX_BYTE: MSB first ────────────────────────────────
                    S_TX_BYTE: begin
                        case (phase)
                            3'd0: begin
                                sda_out <= tx_byte[bit_idx];
                                sda_oe  <= 1'b1;
                                i2c_scl <= 0;
                                phase   <= 3'd1;
                            end
                            3'd1: begin i2c_scl <= 1; phase <= 3'd2; end
                            3'd2: begin phase <= 3'd3; end
                            3'd3: begin
                                i2c_scl <= 0;
                                if (bit_idx == 3'd0) begin
                                    phase <= 3'd0;
                                    if (tx_byte == ADDR_W) begin
                                        state <= S_ACK_W;
                                    end else if (tx_byte == CMD_HP) begin
                                        state <= S_ACK_W;
                                    end else if (tx_byte == ADDR_R) begin
                                        state <= S_ACK_R;
                                    end
                                end else begin
                                    bit_idx <= bit_idx - 1'b1;
                                    phase   <= 3'd0;
                                end
                            end
                            default: phase <= 3'd0;
                        endcase
                    end

                    // ── ACK 수신 (쓰기 후) ────────────────────────────────
                    S_ACK_W: begin
                        case (phase)
                            3'd0: begin sda_oe<=0; i2c_scl<=0; phase<=3'd1; end
                            3'd1: begin i2c_scl<=1; phase<=3'd2; end
                            3'd2: begin phase<=3'd3; end  // ACK 샘플 (무시)
                            3'd3: begin
                                i2c_scl <= 0;
                                phase   <= 3'd0;
                                if (tx_byte == ADDR_W) begin
                                    tx_byte <= CMD_HP;
                                    bit_idx <= 3'd7;
                                    state   <= S_TX_BYTE;
                                end else begin
                                    wait_cnt <= 32'd0;
                                    state    <= S_WAIT_MEAS;
                                end
                            end
                            default: phase <= 3'd0;
                        endcase
                    end

                    // ── 10ms 측정 대기 ────────────────────────────────────
                    S_WAIT_MEAS: begin
                        wait_cnt <= wait_cnt + 1;
                        if (wait_cnt == WAIT_10MS - 1) begin
                            wait_cnt <= 32'd0;
                            state    <= S_RESTART;
                            phase    <= 3'd0;
                        end
                    end

                    // ── RESTART condition ─────────────────────────────────
                    S_RESTART: begin
                        case (phase)
                            3'd0: begin sda_out<=1; sda_oe<=1; i2c_scl<=0; phase<=3'd1; end
                            3'd1: begin i2c_scl<=1; phase<=3'd2; end
                            3'd2: begin sda_out<=0; phase<=3'd3; end
                            3'd3: begin
                                i2c_scl <= 0;
                                tx_byte <= ADDR_R;
                                bit_idx <= 3'd7;
                                state   <= S_TX_BYTE;
                                phase   <= 3'd0;
                            end
                            default: phase <= 3'd0;
                        endcase
                    end

                    // ── ACK 수신 (읽기 주소 후) ───────────────────────────
                    S_ACK_R: begin
                        case (phase)
                            3'd0: begin sda_oe<=0; i2c_scl<=0; phase<=3'd1; end
                            3'd1: begin i2c_scl<=1; phase<=3'd2; end
                            3'd2: begin phase<=3'd3; end
                            3'd3: begin
                                i2c_scl <= 0;
                                bit_idx <= 3'd7;
                                rx_cnt  <= 4'd0;
                                state   <= S_RX_BYTE;
                                phase   <= 3'd0;
                            end
                            default: phase <= 3'd0;
                        endcase
                    end

                    // ── RX_BYTE ───────────────────────────────────────────
                    S_RX_BYTE: begin
                        case (phase)
                            3'd0: begin sda_oe<=0; i2c_scl<=0; phase<=3'd1; end
                            3'd1: begin i2c_scl<=1; phase<=3'd2; end
                            3'd2: begin
                                rx_byte[bit_idx] <= sda_in;
                                phase <= 3'd3;
                            end
                            3'd3: begin
                                i2c_scl <= 0;
                                if (bit_idx == 3'd0) begin
                                    rx_buf[rx_cnt] <= rx_byte;
                                    phase <= 3'd0;
                                    if (rx_cnt == 4'd5) begin
                                        state <= S_SEND_NACK;
                                    end else begin
                                        state <= S_SEND_ACK;
                                    end
                                end else begin
                                    bit_idx <= bit_idx - 1'b1;
                                    phase   <= 3'd0;
                                end
                            end
                            default: phase <= 3'd0;
                        endcase
                    end

                    // ── ACK 전송 (마스터→슬레이브) ───────────────────────
                    S_SEND_ACK: begin
                        case (phase)
                            3'd0: begin sda_out<=0; sda_oe<=1; i2c_scl<=0; phase<=3'd1; end
                            3'd1: begin i2c_scl<=1; phase<=3'd2; end
                            3'd2: begin phase<=3'd3; end
                            3'd3: begin
                                i2c_scl <= 0;
                                rx_cnt  <= rx_cnt + 1'b1;
                                bit_idx <= 3'd7;
                                state   <= S_RX_BYTE;
                                phase   <= 3'd0;
                            end
                            default: phase <= 3'd0;
                        endcase
                    end

                    // ── NACK 전송 (마지막 바이트 후) ─────────────────────
                    S_SEND_NACK: begin
                        case (phase)
                            3'd0: begin sda_out<=1; sda_oe<=1; i2c_scl<=0; phase<=3'd1; end
                            3'd1: begin i2c_scl<=1; phase<=3'd2; end
                            3'd2: begin phase<=3'd3; end
                            3'd3: begin
                                i2c_scl <= 0;
                                state   <= S_STOP;
                                phase   <= 3'd0;
                            end
                            default: phase <= 3'd0;
                        endcase
                    end

                    // ── STOP condition: SDA 0→1 while SCL=1 ──────────────
                    S_STOP: begin
                        case (phase)
                            3'd0: begin sda_out<=0; sda_oe<=1; i2c_scl<=0; phase<=3'd1; end
                            3'd1: begin i2c_scl<=1; phase<=3'd2; end
                            3'd2: begin sda_out<=1; phase<=3'd3; end
                            3'd3: begin state<=S_DONE; phase<=3'd0; end
                            default: phase <= 3'd0;
                        endcase
                    end

                    // ── DONE: 수신 데이터 출력 ────────────────────────────
                    // rx_buf: [0]=T_H [1]=T_L [2]=CRC_T [3]=H_H [4]=H_L [5]=CRC_H
                    S_DONE: begin
                        sht_data  <= {rx_buf[0], rx_buf[1], rx_buf[3], rx_buf[4]};
                        sht_valid <= 1'b1;
                        state     <= S_IDLE;
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end
endmodule