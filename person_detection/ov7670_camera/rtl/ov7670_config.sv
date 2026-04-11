// =======================================================
// ov7670_config.sv: OV7670 레지스터 설정 컨트롤러
// -------------------------------------------------------
// SCCB 마스터를 사용하여 OV7670 초기화
//
// 동작:
//     1. start=1 -> 초기화 시작
//     2. ROM에서 레지스터 설정값 읽기
//     3. SCCB로 전송
//     4. done=1 -> 완료
// 설정 내용:
//     - 해상도: QVGA (320x240)
//     - 포맷: RGB565
//     - 클럭 설정
// =======================================================

module ov7670_config #(
    parameter SYS_CLK_FREQ = 125_000_000
)(
    input clk, rstn,

    input      start,        // 초기화 시작
    output reg done, err,    // 초기화 완료 / 에러 플래그 (타임아웃)

    // SCCB Interface
    output reg scl,
    inout      sda
    );

    // ================ 레지스터 설정 ROM =================
    localparam NUM_REGS = 50;

    typedef logic [15:0] regData_t;
    regData_t config_rom [NUM_REGS];

    // -------- OV7670 기본 초기화 (QVGA, RGB565) ---------
    initial begin
        // Reset all registers
        config_rom[0] = 16'h12_80;     // COM7: Reset

        // Delay 필요
        config_rom[1] = 16'hFF_FF;     // Delay marker

        // Clock settings
        config_rom[2] = 16'h11_80;     // CLKRC: Use external clock
        config_rom[3] = 16'h0C_00;     // COM3: Default
        config_rom[4] = 16'h3E_00;     // COM14: Default

        // Image format: RGB565
        config_rom[5] = 16'h8C_00;     // RGB444 disabled
        config_rom[6] = 16'h40_10;     // COM15: RGB565
        config_rom[7] = 16'h12_04;     // COM7: QVGA + RGB

        // Resolution: QVGA (320x240)
        config_rom[8] = 16'h32_80;     // HREF
        config_rom[9] = 16'h17_16;     // HSTART
        config_rom[10] = 16'h18_05;    // HSTOP
        config_rom[11] = 16'h19_02;    // VSTART
        config_rom[12] = 16'h1A_7A;    // VSTOP
        config_rom[13] = 16'h03_0A;    // VREF

        // COM registers
        config_rom[14] = 16'h0E_61;    // COM5
        config_rom[15] = 16'h0F_4B;    // COM6
        config_rom[16] = 16'h16_02;    // Reserved
        config_rom[17] = 16'h1E_07;    // MVFP: Mirror/VFlip
        config_rom[18] = 16'h21_02;    // ADCCTR0
        config_rom[19] = 16'h22_91;    // ADCCTR1
        config_rom[20] = 16'h29_07;    // RSVD
        config_rom[21] = 16'h33_0B;    // CHLF
        config_rom[22] = 16'h35_0B;    // Reserved
        config_rom[23] = 16'h37_1D;    // ADC
        config_rom[24] = 16'h38_71;    // ACOM
        config_rom[25] = 16'h39_2A;    // OFON
        config_rom[26] = 16'h3C_78;    // COM12
        config_rom[27] = 16'h4D_40;    // Reserved
        config_rom[28] = 16'h4E_20;    // Reserved
        config_rom[29] = 16'h69_00;    // GFIX

        // Matrix coefficients
        config_rom[30] = 16'h6B_4A;    // DBLV
        config_rom[31] = 16'h74_10;    // REG74
        config_rom[32] = 16'h8D_4F;    // Reserved
        config_rom[33] = 16'h8E_00;    // Reserved
        config_rom[34] = 16'h8F_00;    // Reserved
        config_rom[35] = 16'h90_00;    // Reserved
        config_rom[36] = 16'h91_00;    // Reserved
        config_rom[37] = 16'h96_00;    // Reserved
        config_rom[38] = 16'h9A_00;    // Reserved
        config_rom[39] = 16'hB0_84;    // Reserved
        config_rom[40] = 16'hB1_0C;    // ABLC1
        config_rom[41] = 16'hB2_0E;    // Reserved
        config_rom[42] = 16'hB3_82;    // THL_ST
        config_rom[43] = 16'hB8_0A;    // Reserved

        // AGC/AEC
        config_rom[44] = 16'h13_E0;    // COM8: AGC/AEC/AWB enable
        config_rom[45] = 16'h00_00;    // GAIN
        config_rom[46] = 16'h10_00;    // AECH
        config_rom[47] = 16'h0D_40;    // COM4
        config_rom[48] = 16'h14_18;    // COM9: AGC gain ceiling
        config_rom[49] = 16'hA5_05;    // BD50MAX
    end

    // ============== SCCB 마스터 인터페이스 ===============
    logic sccb_en;
    logic [7:0] sccb_regAddr;
    logic [7:0] sccb_data;
    logic sccb_busy, sccb_done;

    sccb_master #(.SYS_CLK_FREQ(SYS_CLK_FREQ), .SCCB_CLK_FREQ(100_000)) u_sccb (
        .clk(clk), .rstn(rstn),
        .en(sccb_en),
        .slvAddr(7'h21), .regAddr(sccb_regAddr), .data(sccb_data),
        .busy(sccb_busy), .done(sccb_done),
        .scl(scl), .sda(sda));

    // ================== FSM 상태 정의 ===================
    typedef enum logic [2:0] {IDLE, DELAY, LOAD_REG, SEND, WAIT_DONE, COMPLETE} state_t;
    state_t cState, nState;

    // ================== 내부 레지스터 ====================
    logic [$clog2(NUM_REGS)-1:0] reg_idx;

    localparam DELAY_CYCLES = SYS_CLK_FREQ / 100;    // 10ms
    logic [$clog2(DELAY_CYCLES)-1:0] delay_cnt;

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
            IDLE:      if (start)                         nState = DELAY;
            DELAY:     if (delay_cnt == DELAY_CYCLES - 1) nState = LOAD_REG;
            LOAD_REG: begin
                if (reg_idx < NUM_REGS) begin
                    if (config_rom[reg_idx] == 16'hFFFF) nState = DELAY;       // Delay marker 체크
                    else                                 nState = SEND;
                end
                else                                     nState = COMPLETE;
            end
            SEND:                     nState = WAIT_DONE;
            WAIT_DONE: if (sccb_done) nState = LOAD_REG;
            COMPLETE:                 nState = IDLE;
            default: nState = IDLE;
        endcase
    end

    // ---------- 출력 레지스터 + 내부 레지스터 ------------
    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            done <= 0; err <= 0;
            sccb_en <= 0;
            sccb_regAddr <= 0; sccb_data <= 0;
            reg_idx <= 0;
            delay_cnt <= 0;
        end
        else begin
            done <= 1'b0;       // 1 클럭 펄스
            sccb_en <= 1'b0;    // 1 클럭 펄스

            unique case (cState)
                IDLE: begin
                    err <= 1'b0;

                    if (start) begin
                        reg_idx <= '0;
                        delay_cnt <= '0;
                    end
                end
                DELAY: begin
                    if (delay_cnt == DELAY_CYCLES - 1) delay_cnt <= '0;
                    else                               delay_cnt <= delay_cnt + 1'b1;
                end
                LOAD_REG: begin
                    if (reg_idx < NUM_REGS) begin
                        sccb_regAddr <= config_rom[reg_idx][15:8];
                        sccb_data <= config_rom[reg_idx][7:0];
                        
                        // Delay marker면 reg_idx 증가
                        if (config_rom[reg_idx] == 16'hFFFF) begin
                            reg_idx <= reg_idx + 1'b1;
                            delay_cnt <= '0;
                        end
                    end
                end
                SEND: begin
                    sccb_en <= 1'b1;
                end
                WAIT_DONE: begin
                    if (sccb_done) reg_idx <= reg_idx + 1'b1;
                end
                COMPLETE: begin
                    done <= 1'b1;
                end
            endcase
        end
    end

endmodule
