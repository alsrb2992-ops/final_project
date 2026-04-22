`timescale 1ns / 1ps

// ==========================================================
// tb_sccb_top.sv: SCCB 통합 모듈 테스트벤치
// ----------------------------------------------------------
// 검증 항목:
//     1. ready 신호 (100ms 후 활성화)
//     2. SCCB 타이밍 (SCL ~100kHz, START/STOP 조건)
//     3. 레지스터 순차 전송
//     4. ACK 비트 확인
//     5. cfgDone 신호 활성화
// ==========================================================

module tb_sccb_top;
    logic clk, rstn;

    // SCCB 신호
    wire  scl, sda;
    logic cfgDone;

    // ===================== 풀업 저항 ======================
    pullup(scl);
    pullup(sda);

    // ======================== DUT =========================
    sccb_top dut (.*);

    // =========== 슬레이브 SDA 구동 (Open-drain) ============
    logic sda_drive;
    assign sda = (sda_drive) ? 1'b0 : 1'bz;

    // ================= 클럭 생성 (125MHz) ==================
    initial clk = 0;
    always #4 clk = ~clk;

    // ======================= 초기화 ========================
    initial begin
        rstn = 0;
        sda_drive = 0;

        #100 rstn = 1'b1;
        $display("===================================== SCCB Simulation Start =====================================");
    end

    // ======== OV7670 슬레이브 에뮬레이션 (ACK 생성) =========
    logic       inTx;
    logic [5:0] bit_cnt;

    initial begin
        inTx = 0;
        bit_cnt = 0;
    end

    // SCL 하강 엣지에서 비트 카운트
    always @(negedge scl or negedge rstn) begin
        if (!rstn) begin
            inTx = 0;
            bit_cnt = 0;
            sda_drive = 0;
        end
        else begin
            if (inTx) begin
                bit_cnt = bit_cnt + 1;

                // 8, 17, 26번째 비트: 다음 클럭의 ACK 준비
                if ((bit_cnt == 8) || (bit_cnt == 17) || (bit_cnt == 26)) begin
                    sda_drive = 1'b1;    // ACK (SDA = 0)
                    $display("[%0d] ACK prepared (bit #%0d -> ACK at bit #%0d)", $time, bit_cnt, (bit_cnt + 1));
                end
                // 9, 18, 27번째 비트: ACK 해제
                else if ((bit_cnt == 9) || (bit_cnt == 18) || (bit_cnt == 27)) begin
                    sda_drive = 1'b0;    // ACK 해제
                    $display("[%0d] ACK released (bit #%0d)", $time, bit_cnt);
                end
                else begin
                    sda_drive = 1'b0;    // Release SDA
                end

                // 전송 완료 (27비트: 슬레이브 주소 + 레지스터 주소 + 데이터)
                if (bit_cnt >= 27) begin
                    bit_cnt = 0;
                    inTx = 1'b0;
                end
            end
        end
    end

    // ================ START/STOP 조건 감지 =================
    logic sda_prev;

    always @(posedge clk) begin
        sda_prev <= sda;

        // START 조건 (SDA: 1->0, SCL: 1)
        if (scl && sda_prev && !sda) begin
            inTx = 1'b1;
            bit_cnt = 0;
            sda_drive = 1'b0;
            $display("[%0d] START condition detected", $time);
        end

        // STOP 조건 (SDA: 0->1, SCL: 1)
        if (scl && !sda_prev && sda) begin
            inTx = 1'b0;
            $display("[%0d] STOP condition detected\n", $time);
        end
    end

    // ====================== 모니터링 =======================
    integer reg_cnt = 0;
//    logic cfgDone_prev = 0;

    // ready
    always @(posedge dut.ready) begin
        $display("[%0d] Init timer complete (ready=1)", $time);
    end

    // txDone
    always @(posedge dut.txDone) begin
        $display("[%0d] txDone pulse", $time);
    end

    // romAddr
    always @(dut.romAddr) begin
        $display("[%0d] romAddr = %0d (regAddr=0x%02X, regData=0x%02X)", $time, dut.romAddr, dut.regAddr, dut.regData);
    end

    // cfgDone
    always @(posedge cfgDone) begin
        $display("-------------------------------------------------------------------------------------------------");
        $display("[%0d] Configuration complete!", $time);
        $display("Total Registers Transmitted: %0d", reg_cnt);

        #1000 $finish;
    end

    // STOP 조건마다 레지스터 카운트 증가
    always @(posedge clk) begin
        if (scl && !sda_prev && sda && !inTx) begin
            reg_cnt = reg_cnt + 1;
            $display("[%0d] Register #%0d transmitted", $time, reg_cnt);
        end
    end

    // ====================== 타임아웃 =======================
    initial begin
        #200_000_000;    // 200ms
        $display("-------------------------------------------------------------------------------------------------");
        $display("ERROR: Timeout!");
        $display("cfgDone not asserted within 500ms");
        $display("Registers transmitted: %0d / 18", reg_cnt);
        $display("Current romAddr: %0d", dut.romAddr);
        $display("Current FSM state: %0d", dut.u_ctrl.cState);
        $finish;
    end

endmodule
