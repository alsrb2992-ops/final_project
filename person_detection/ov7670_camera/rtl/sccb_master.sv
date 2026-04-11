// =======================================================
// sccb_master.v: SCCB 마스터 모듈
// -------------------------------------------------------
// OV7670 카메라 레지스터 설정용
//
// SCCB vs I2C 차이점:
//     - ACK 비트를 전송하지만 체크하지 않음 (무시)
//     - Don't Care 비트 사용
// 동작:
//     1. en=1로 설정
//     2. slvAddr, regAddr, data 입력
//     3. busy=0이 될 때까지 대기
//     4. 완료 (다음 전송 가능)
// 타이밍:
//     - SCL 클럭: 100kHz
//     - 시스템 클럭: 125MHz
// =======================================================

module sccb_master #(
    parameter SYS_CLK_FREQ = 125_000_000,    // 시스템 클럭 주파수 (Hz)
    parameter SCCB_CLK_FREQ = 100_000        // SCCB 클럭 주파수 (Hz)
)(
    input clk, rstn,

    // Control Interface
    input en,                 // 전송 시작 (1 펄스)

    input [6:0] slvAddr,      // 슬레이브 주소 (OV7670 = 0x21, Write 시 0x42)
    input [7:0] regAddr,      // 레지스터 주소
    input [7:0] data,         // 데이터

    output reg busy, done,    // 전송 중 플래그 / 전송 완료 (1 펄스)

    // SCCB Physical Interface
    output reg scl,               // SCCB 클럭
    inout      sda                // SCCB 데이터 (양방향)
    );

    // =========== 클럭 분주기 (SCCB 클럭 생성 ) ===========
    localparam CLK_DIV = SYS_CLK_FREQ / (SCCB_CLK_FREQ * 4);

    reg [$clog2(CLK_DIV)-1:0] clk_cnt;
    reg [1:0] sccb_phase;                 // 0: 준비, 1: SCL 상승, 2: 유지, 3: SCL 하강

    wire tick = (clk_cnt == CLK_DIV - 1);

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            clk_cnt <= 0;
            sccb_phase <= 0;
        end
        else if (busy) begin
            if (tick) begin
                clk_cnt <= '0;
                sccb_phase <= sccb_phase + 1'b1;
            end
            else begin
                clk_cnt <= clk_cnt + 1'b1;
            end
        end
        else begin
            clk_cnt <= 0;
            sccb_phase <= 0;
        end
    end

    // ================== FSM 상태 정의 ===================
    typedef enum logic [4:0] {IDLE, START,
                              ADDR_7, ADDR_6, ADDR_5, ADDR_4, ADDR_3, ADDR_2, ADDR_1, ADDR_0, ADDR_ACK,    // Slave Addr bit / ACK (무시)
                              REG_7, REG_6, REG_5, REG_4, REG_3, REG_2, REG_1, REG_0, REG_ACK,             // Register Addr bit
                              DATA_7, DATA_6, DATA_5, DATA_4, DATA_3, DATA_2, DATA_1, DATA_0, DATA_ACK,    // Data bit
                              STOP, DONE} state_t;
    state_t cState, nState;

    // ================ 전송 데이터 래치 ==================
    logic [6:0] slvAddr_reg;
    logic [7:0] regAddr_reg;
    logic [7:0] data_reg;

    // ============== SDA 출력 제어 (양방향) ==============
    logic sda_out;
    logic sda_out_en;

    assign sda = (sda_out_en) ? sda_out : 1'bz;

    // ====================== FSM ========================
    // ------------------ 상태 업데이트 -------------------
    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) cState <= IDLE;
        else cState <= nState;
    end

    // ---------------- Next State Logic -----------------
    always_comb begin
        nState = cState;

        unique case (cState)
            IDLE:     if (en)                 nState = START;
            START:    if (sccb_phase == 2'd3) nState = ADDR_7;
            // Slave Address
            ADDR_7:   if (sccb_phase == 2'd3) nState = ADDR_6;
            ADDR_6:   if (sccb_phase == 2'd3) nState = ADDR_5;
            ADDR_5:   if (sccb_phase == 2'd3) nState = ADDR_4;
            ADDR_4:   if (sccb_phase == 2'd3) nState = ADDR_3;
            ADDR_3:   if (sccb_phase == 2'd3) nState = ADDR_2;
            ADDR_2:   if (sccb_phase == 2'd3) nState = ADDR_1;
            ADDR_1:   if (sccb_phase == 2'd3) nState = ADDR_0;
            ADDR_0:   if (sccb_phase == 2'd3) nState = ADDR_ACK;
            ADDR_ACK: if (sccb_phase == 2'd3) nState = REG_7;
            // Register Address
            REG_7:    if (sccb_phase == 2'd3) nState = REG_6;
            REG_6:    if (sccb_phase == 2'd3) nState = REG_5;
            REG_5:    if (sccb_phase == 2'd3) nState = REG_4;
            REG_4:    if (sccb_phase == 2'd3) nState = REG_3;
            REG_3:    if (sccb_phase == 2'd3) nState = REG_2;
            REG_2:    if (sccb_phase == 2'd3) nState = REG_1;
            REG_1:    if (sccb_phase == 2'd3) nState = REG_0;
            REG_0:    if (sccb_phase == 2'd3) nState = REG_ACK;
            REG_ACK:  if (sccb_phase == 2'd3) nState = DATA_7;
            // Data
            DATA_7:   if (sccb_phase == 2'd3) nState = DATA_6;
            DATA_6:   if (sccb_phase == 2'd3) nState = DATA_5;
            DATA_5:   if (sccb_phase == 2'd3) nState = DATA_4;
            DATA_4:   if (sccb_phase == 2'd3) nState = DATA_3;
            DATA_3:   if (sccb_phase == 2'd3) nState = DATA_2;
            DATA_2:   if (sccb_phase == 2'd3) nState = DATA_1;
            DATA_1:   if (sccb_phase == 2'd3) nState = DATA_0;
            DATA_0:   if (sccb_phase == 2'd3) nState = DATA_ACK;
            DATA_ACK: if (sccb_phase == 2'd3) nState = STOP;
            STOP:     if (sccb_phase == 2'd3) nState = DONE;
            DONE:                             nState = IDLE;
            default: nState = IDLE;
        endcase
    end

    // ------------------ 출력 레지스터 -------------------
    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            busy <= 0; done <= 0;
            slvAddr_reg <= 0; regAddr_reg <= 0; data_reg <= 0;
            scl <= 1;
            sda_out <= 1; sda_out_en <= 0;
        end
        else begin
            done <= 1'b0;    // 1 클럭 펄스
            
            unique case (cState)
                IDLE: begin
                    busy <= 1'b0;
                    scl <= 1'b1;
                    sda_out <= 1'b1; sda_out_en <= 1'b0;

                    // 데이터 래치
                    if (en) begin
                        slvAddr_reg <= slvAddr; regAddr_reg <= regAddr; data_reg <= data;
                    end
                end
                START: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;

                    if (sccb_phase == 2'd0) begin
                        scl <= 1'b1;
                        sda_out <= 1'b1;
                    end
                    else if (sccb_phase == 2'd1) begin
                        scl <= 1'b1;
                        sda_out <= 1'b0;                  // SDA falling
                    end
                    else if (sccb_phase == 2'd2) begin
                        scl <= 1'b1;
                        sda_out <= 1'b0;
                    end
                    else begin
                        scl <= 1'b0;                      // SCL falling
                        sda_out <= 1'b0;
                    end
                end

                // Slave Address
                ADDR_7: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= slvAddr_reg[6];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                ADDR_6: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= slvAddr_reg[5];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                ADDR_5: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= slvAddr_reg[4];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                ADDR_4: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= slvAddr_reg[3];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                ADDR_3: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= slvAddr_reg[2];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                ADDR_2: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= slvAddr_reg[1];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                ADDR_1: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= slvAddr_reg[0];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                ADDR_0: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= 1'b0;       // Write bit

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                ADDR_ACK: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b0;    // 입력 모드

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end

                // Register Address
                REG_7: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= regAddr_reg[7];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                REG_6: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= regAddr_reg[6];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                REG_5: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= regAddr_reg[5];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                REG_4: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= regAddr_reg[4];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                REG_3: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= regAddr_reg[3];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                REG_2: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= regAddr_reg[2];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                REG_1: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= regAddr_reg[1];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                REG_0: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= regAddr_reg[0];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                REG_ACK: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b0;

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end

                // Data
                DATA_7: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= data_reg[7];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                DATA_6: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= data_reg[6];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                DATA_5: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= data_reg[5];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                DATA_4: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= data_reg[4];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                DATA_3: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= data_reg[3];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                DATA_2: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= data_reg[2];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                DATA_1: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= data_reg[1];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                DATA_0: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;
                    sda_out <= data_reg[0];

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end
                DATA_ACK: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b0;

                    if      (sccb_phase == 2'd0) scl <= 1'b0;
                    else if (sccb_phase == 2'd1) scl <= 1'b1;
                    else if (sccb_phase == 2'd2) scl <= 1'b1;
                    else                         scl <= 1'b0;
                end

                STOP: begin
                    busy <= 1'b1;
                    sda_out_en <= 1'b1;

                    if (sccb_phase == 2'd0) begin
                        scl <= 1'b0;
                        sda_out <= 1'b0;
                    end
                    else if (sccb_phase == 2'd1) begin
                        scl <= 1'b1;
                        sda_out <= 1'b0;
                    end
                    else if (sccb_phase == 2'd2) begin
                        scl <= 1'b1;
                        sda_out <= 1'b1;                  // SDA rising
                    end
                    else begin
                        scl <= 1'b1;
                        sda_out <= 1'b1;
                    end
                end
                DONE: begin
                    busy <= 1'b0; done <= 1'b1;
                    scl <= 1'b1;
                    sda_out <= 1'b1; sda_out_en <= 1'b0;
                end
            endcase
        end
    end

endmodule
