// ===========================================================
// sccb_ctrl.v: SCCB 프로토콜 컨트롤러
// -----------------------------------------------------------
// 기능:
//     - I2C 호환 2-wire 통신 (SCCB)
//     - OV7670 슬레이브 주소: 0x42 (Write)
// 타이밍:
//     - 시스템 클럭: 125MHz
//     - SCCB 클럭: ~100kHz (SCL)
//     - 1 클럭 주기: 10us (4단계 * 2.5us)
// ===========================================================

module sccb_ctrl(
    input clk, rstn,

    input start,

    input [7:0] regAddr, regData,

    output scl, sda,

    output reg txDone
    );

    // ============== 타이밍 상수 (125MHz 기준) ===============
    localparam CLK_DIV = 312;                  // 125MHz / 100kHz / 4 = 312.5
    localparam QUARTER = CLK_DIV - 1;          // 2.5us (0-311)
    localparam HALF    = (CLK_DIV * 2) - 1;    // 5us (0-623)

    localparam WAIT = 625_000;                 // 125MHz * 5ms

    // ================= OV7670 슬레이브 주소 =================
    localparam SLV_ADDR = 8'h42;    // Write 주소 (0x21 << 1)

    // ==================== FSM 상태 정의 =====================
    localparam IDLE=0, START_1=1, START_2=2,
               SLV_ADDR_1=3, SLV_ADDR_2=4, SLV_ADDR_3=5, SLV_ADDR_4=6,
               SLV_ACK_1=7, SLV_ACK_2=8, SLV_ACK_3=9, SLV_ACK_4=10,
               REG_ADDR_1=11, REG_ADDR_2=12, REG_ADDR_3=13, REG_ADDR_4=14,
               REG_ACK_1=15, REG_ACK_2=16, REG_ACK_3=17, REG_ACK_4=18,
               REG_DATA_1=19, REG_DATA_2=20, REG_DATA_3=21, REG_DATA_4=22,
               DATA_ACK_1=23, DATA_ACK_2=24, DATA_ACK_3=25, DATA_ACK_4=26,
               STOP_PREP=27, STOP_1=28, STOP_2=29, WAIT_RESET=30;
    reg [4:0] cState, nState;

    // =================== 데이터 레지스터 ====================
    reg       scl_reg, sda_reg;
    reg [7:0] txData;

    reg [19:0] clk_cnt;
    reg  [2:0] bit_cnt;

    // ========= SCL/SDA 출력 (Open-drain 에뮬레이션) =========
    assign scl = (scl_reg == 0) ? 1'b0 : 1'bz;
    assign sda = (sda_reg == 0) ? 1'b0 : 1'bz;

    // ================ 상태 레지스터 업데이트 =================
    always @(posedge clk or negedge rstn) begin
        if (!rstn) cState <= IDLE;
        else cState <= nState;
    end

    // ====================== 상태 전이 =======================
    always @(*) begin
        nState = cState;

        case (cState)
            IDLE:       if (start) nState = START_1;

            // START 조건
            START_1:    if (clk_cnt == HALF) nState = START_2;
            START_2:    if (clk_cnt == HALF) nState = SLV_ADDR_1;

            // 슬레이브 주소 전송
            SLV_ADDR_1: if (clk_cnt == QUARTER) nState = SLV_ADDR_2;
            SLV_ADDR_2: if (clk_cnt == QUARTER) nState = SLV_ADDR_3;
            SLV_ADDR_3: if (clk_cnt == QUARTER) nState = SLV_ADDR_4;
            SLV_ADDR_4: begin
                if (clk_cnt == QUARTER) begin
                    if (bit_cnt == 7) nState = SLV_ACK_1;
                    else              nState = SLV_ADDR_1;
                end
            end

            // ACK 대기 (슬레이브)
            SLV_ACK_1:  if (clk_cnt == QUARTER) nState = SLV_ACK_2;
            SLV_ACK_2:  if (clk_cnt == QUARTER) nState = SLV_ACK_3;
            SLV_ACK_3:  if (clk_cnt == QUARTER) nState = SLV_ACK_4;
            SLV_ACK_4:  if (clk_cnt == QUARTER) nState = REG_ADDR_1;

            // 레지스터 주소 전송
            REG_ADDR_1: if (clk_cnt == QUARTER) nState = REG_ADDR_2;
            REG_ADDR_2: if (clk_cnt == QUARTER) nState = REG_ADDR_3;
            REG_ADDR_3: if (clk_cnt == QUARTER) nState = REG_ADDR_4;
            REG_ADDR_4: begin
                if (clk_cnt == QUARTER) begin
                    if (bit_cnt == 7) nState = REG_ACK_1;
                    else              nState = REG_ADDR_1;
                end
            end

            // ACK 대기 (레지스터 주소)
            REG_ACK_1:  if (clk_cnt == QUARTER) nState = REG_ACK_2;
            REG_ACK_2:  if (clk_cnt == QUARTER) nState = REG_ACK_3;
            REG_ACK_3:  if (clk_cnt == QUARTER) nState = REG_ACK_4;
            REG_ACK_4:  if (clk_cnt == QUARTER) nState = REG_DATA_1;

            // 레지스터 데이터 전송
            REG_DATA_1: if (clk_cnt == QUARTER) nState = REG_DATA_2;
            REG_DATA_2: if (clk_cnt == QUARTER) nState = REG_DATA_3;
            REG_DATA_3: if (clk_cnt == QUARTER) nState = REG_DATA_4;
            REG_DATA_4: begin
                if (clk_cnt == QUARTER) begin
                    if (bit_cnt == 7) nState = DATA_ACK_1;
                    else              nState = REG_DATA_1;
                end
            end

            // ACK 대기 (데이터)
            DATA_ACK_1: if (clk_cnt == QUARTER) nState = DATA_ACK_2;
            DATA_ACK_2: if (clk_cnt == QUARTER) nState = DATA_ACK_3;
            DATA_ACK_3: if (clk_cnt == QUARTER) nState = DATA_ACK_4;
            DATA_ACK_4: if (clk_cnt == QUARTER) nState = STOP_PREP;

            // STOP 준비 (SCL LOW, SDA LOW)
            STOP_PREP:  if (clk_cnt == QUARTER) nState = STOP_1;
            // STOP 조건
            STOP_1:     if (clk_cnt == HALF) nState = STOP_2;
            STOP_2: begin
                if (clk_cnt == HALF) begin
                    if ((regAddr == 8'h12) && (regData == 8'h80)) nState = WAIT_RESET;    // Reset 레지스터 전용 대기
                    else                                          nState = IDLE;
                end
            end

            // Reset 대기 (500ms)
            WAIT_RESET: if (clk_cnt >= WAIT) nState = IDLE;    // 125MHz * 500ms

            default: nState = IDLE;
        endcase
    end

    // ============ 출력 + 데이터 레지스터 업데이트 ============
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            scl_reg <= 1; sda_reg <= 1;
            txData <= SLV_ADDR;
            clk_cnt <= 0; bit_cnt <= 0;
            txDone <= 0;
        end
        else begin
            txDone <= 1'b0;

            case (cState)
                // ------------------ IDLE -------------------
                IDLE: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b1;
                    txData <= SLV_ADDR;
                    clk_cnt <= 0; bit_cnt <= 0;
                end

                // --------------- START 조건 ----------------
                START_1: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b0;

                    if (clk_cnt == HALF) clk_cnt <= 0;
                    else                 clk_cnt <= clk_cnt + 1;
                end
                START_2: begin
                    scl_reg <= 1'b0; sda_reg <= 1'b0;

                    if (clk_cnt == HALF) clk_cnt <= 0;
                    else                 clk_cnt <= clk_cnt + 1; 
                end

                // ------------ 슬레이브 주소 전송 ------------
                SLV_ADDR_1: begin
                    scl_reg <= 1'b0; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                SLV_ADDR_2: begin
                    scl_reg <= 1'b1; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                SLV_ADDR_3: begin
                    scl_reg <= 1'b1; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                SLV_ADDR_4: begin
                    scl_reg <= 1'b0; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) begin
                        clk_cnt <= 0;
                        if (bit_cnt == 7) begin
                            bit_cnt <= 0;
                            txData <= regAddr;
                        end
                        else begin
                            bit_cnt <= bit_cnt + 1;
                            txData <= {txData[6:0], 1'b0};
                        end
                    end
                    else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // ----------- ACK 대기 (슬레이브) ------------
                SLV_ACK_1: begin
                    scl_reg <= 1'b0; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                SLV_ACK_2: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                SLV_ACK_3: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                SLV_ACK_4: begin
                    scl_reg <= 1'b0; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end

                // ------------ 레지스터 주소 전송 -------------
                REG_ADDR_1: begin
                    scl_reg <= 1'b0; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                REG_ADDR_2: begin
                    scl_reg <= 1'b1; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                REG_ADDR_3: begin
                    scl_reg <= 1'b1; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                REG_ADDR_4: begin
                    scl_reg <= 1'b0; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) begin
                        clk_cnt <= 0;
                        if (bit_cnt == 7) begin
                            bit_cnt <= 0;
                            txData <= regData;
                        end
                        else begin
                            bit_cnt <= bit_cnt + 1;
                            txData <= {txData[6:0], 1'b0};
                        end
                    end
                    else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // --------- ACK 대기 (레지스터 주소) ----------
                REG_ACK_1: begin
                    scl_reg <= 1'b0; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                REG_ACK_2: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                REG_ACK_3: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                REG_ACK_4: begin
                    scl_reg <= 1'b0; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end

                // ----------- 레지스터 데이터 전송 ------------
                REG_DATA_1: begin
                    scl_reg <= 1'b0; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                REG_DATA_2: begin
                    scl_reg <= 1'b1; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                REG_DATA_3: begin
                    scl_reg <= 1'b1; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                REG_DATA_4: begin
                    scl_reg <= 1'b0; sda_reg <= txData[7];

                    if (clk_cnt == QUARTER) begin
                        clk_cnt <= 0;
                        if (bit_cnt == 7) begin
                            bit_cnt <= 0;
                        end
                        else begin
                            bit_cnt <= bit_cnt + 1;
                            txData <= {txData[6:0], 1'b0};
                        end
                    end
                    else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // ------------ ACK 대기 (데이터) -------------
                DATA_ACK_1: begin
                    scl_reg <= 1'b0; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                DATA_ACK_2: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                DATA_ACK_3: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                DATA_ACK_4: begin
                    scl_reg <= 1'b0; sda_reg <= 1'b1;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end

                // ---------------- STOP 준비 -----------------
                STOP_PREP: begin
                    scl_reg <= 1'b0; sda_reg <= 1'b0;

                    if (clk_cnt == QUARTER) clk_cnt <= 0;
                    else                    clk_cnt <= clk_cnt + 1;
                end
                // ---------------- STOP 조건 -----------------
                STOP_1: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b0;

                    if (clk_cnt == HALF) clk_cnt <= 0;
                    else                 clk_cnt <= clk_cnt + 1;
                end
                STOP_2: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b1;

                    if (clk_cnt == HALF) begin
                        clk_cnt <= 0;
                        if (!((regAddr == 8'h12) && (regData == 8'h80))) txDone <= 1'b1;
                    end
                    else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // --------------- Reset 대기 ----------------
                WAIT_RESET: begin
                    scl_reg <= 1'b1; sda_reg <= 1'b1;

                    if (clk_cnt >= WAIT) begin
                        clk_cnt <= 0;
                        txDone <= 1'b1;
                    end
                    else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
            endcase
        end
    end

endmodule
