`timescale 1ns / 1ps

// ==========================================================
// tb_ov7670_capture.sv: OV7670 캡처 모듈 테스트벤치
// ----------------------------------------------------------
// 테스트 시나리오:
//     1. 정상 프레임 캡처 (VSYNC 하강 -> 픽셀 전송)
//     2. VSYNC 하강 시 wAddr 리셋 검증
//     3. VSYNC 하강 직전 wEn=1 엣지 케이스
//     4. 여러 프레임 연속 캡처
//     5. byte_toggle 상태 확인
// 검증 항목:
//     - VSYNC 하강 엣지에서 wAddr=0 리셋
//     - href=1, vsync=0일 때만 픽셀 캡처
//     - 2바이트 -> 1픽셀 RGB565 조합
//     - wEn 펄스 타이밍 (픽셀당 1번)
//     - wAddr 순차 증가 (0-76799)
// ==========================================================

module tb_ov7670_capture;
    reg pclk;

    // OV7670 신호
    reg       href, vsync;
    reg [7:0] data;

    // BRAM Write 신호 (출력)
    wire        wEn;
    wire [16:0] wAddr;
    wire [15:0] wData;

    // ======================== DUT =========================
    ov7670_capture dut (.*);

    // ====================== 클럭 생성 ======================
    // pclk: 6MHz (166ns 주기)
    initial pclk = 0;
    always #83 pclk = ~pclk;

    // =================== 테스트 시나리오 ===================
    initial begin
        $display("================================ OV7670 Capture Module Simulation ================================");

        // 초기화
        href = 1'b0;
        vsync = 1'b1;
        data = 8'h00;
        #500;

        // ------- 시나리오 1: VSYNC 하강 시 wAddr 리셋 -------
        $display("[%0d] Scenario 1: VSYNC Falling Edge Reset", $time);

        // V-Blank (VSYNC=1)
        vsync = 1'b1;
        href = 1'b0;
        #500;

        // VSYNC 하강 (프레임 시작)
        $display("[%0d] VSYNC falling edge (frame start)", $time);
        vsync = 1'b0;
        #500;

        // wAddr 확인
        if (wAddr == 0) $display("[%0d] PASS: wAddr reset to 0", $time);
        else $display("[%0d] FAIL: wAddr = %0d (expected 0)", $time, wAddr);
        $display("");

        // ------- 시나리오 2: 정상 픽셀 전송 (한 라인) -------
        $display("[%0d] Scenario 2: Normal Pixel Transmission", $time);

        // 첫 번째 라인 시작
        #166;
        href = 1'b1;
        $display("[%0d] HREF=1, starting line transmission", $time);

        // 10개 픽셀 전송 (20바이트)
        repeat (10) begin
            // 상위 바이트
            data = $random & 8'hFF;
            #166;
            // 하위 바이트
            data = $random & 8'hFF;
            #166;
        end

        // 라인 종료
        href = 1'b0;
        $display("[%0d] HREF=0, line ended. wAddr = %0d", $time, wAddr);

        // wAddr 확인
        if (wAddr == 9) $display("[%0d] PASS: wAddr = 9 (10 pixels written)", $time);
        else $display("[%0d] FAIL: wAddr = %0d (expected 9)", $time, wAddr);
        $display("");
        #1000;

        // ------------- 시나리오 3: 두 번째 라인 -------------
        $display("[%0d] Scenario 3: Second Line", $time);

        href = 1'b1;

        // 5개 픽셀 추가
        repeat (5) begin
            data = $random & 8'hFF;
            #166;
            data = $random & 8'hFF;
            #166;
        end

        href = 1'b0;
        $display("[%0d] Second line ended. wAddr = %0d", $time, wAddr);

        // wAddr 확인
        if (wAddr == 14) $display("[%0d] PASS: wAddr = 14 (cumulative)", $time);
        else $display("[%0d] FAIL: wAddr = %0d (expected 14)", $time, wAddr);
        $display("");
        #1000;

        // -------- 시나리오 4: VSYNC 하강 직전 wEn=1 --------
        $display("[%0d] Scenario 4: wEn=1 Before VSYNC Falling", $time);

        // 픽셀 전송 중
        href = 1'b1;
        data = 8'hAB;    // 상위 바이트
        #166;
        data = 8'hCD;    // 하위 바이트
        #83;
        $display("[%0d] wEn=%b, wAddr=%0d, (before VSYNC change)", $time, wEn, wAddr);

        // 바로 VSYNC 상승 (V-Blank 시작)
        #83;
        vsync = 1'b1;
        href = 1'b0;
        #500;

        // VSYNC 하강 (새 프레임)
        $display("[%0d] VSYNC falling (new frame start)", $time);
        vsync = 1'b0;
        #500;

        // wAddr 확인
        if (wAddr == 0) $display("[%0d] PASS: wAddr reset to 0 (new frame)", $time);
        else $display("[%0d] FAIL: wAddr = %0d (expected 0)", $time, wAddr);
        $display("");

        // ------------- 시나리오 5: 연속 프레임 --------------
        $display("[%0d] Scenario 5: Continuous Frames", $time);

        // 프레임 1
        $display("[%0d] Frame 1 start", $time);
        vsync = 1'b0;
        href = 1'b1;

        repeat (50) begin    // 50 픽셀
            data = $random & 8'hFF;
            #166;
            data = $random & 8'hFF;
            #166;
        end

        href = 1'b0;
        $display("[%0d] Frame 1 ended. wAddr = %0d", $time, wAddr);

        // V-Blank
        #1000;
        vsync = 1'b1;
        #2000;

        // 프레임 2
        $display("[%0d] Frame 2 start", $time);
        vsync = 1'b0;
        #500;

        // wAddr 확인
        if (wAddr == 0) $display("[%0d] PASS: wAddr reset for frame 2", $time);
        else $display("[%0d] FAIL: wAddr = %0d (expected 0)", $time, wAddr);

        href = 1'b1;

        repeat (30) begin    // 30 픽셀
            data = $random & 8'hFF;
            #166;
            data = $random & 8'hFF;
            #166;
        end

        href = 1'b0;
        $display("[%0d] Frame 2 ended. wAddr = %0d", $time, wAddr);
        $display("");
        #2000;

        // ---------- 시나리오 6: RGB565 데이터 확인 ----------
        $display("[%0d] Scenario 6: RGB565 Data Composition", $time);

        vsync = 1'b1;
        #500;
        vsync = 1'b0;
        #500;

        href = 1'b1;

        // 특정 RGB565 값 전송: 0xF81F(마젠타)
        data = 8'hF8;
        #166;
        data = 8'h1F;
        #166;

        // wData 확인
        #10;
        if (wData == 16'hF81F) $display("[%0d] PASS: wData = 0x%04h (magenta)", $time, wData);
        else $display("[%0d] FAIL: wData = 0x%04h (expected 0xF81F)", $time, wData);
        $display("");

        href = 1'b0;
        #2000;

        $display("Simulation finished");
        #1000 $finish;
    end

    // ====================== 모니터링 =======================
    // 주요 이벤트 출력
    always @(posedge pclk) begin
        if (wEn) $display("[%0d] WRITE: wAddr=%0d, wData=0x%04h", $time, wAddr, wData);
    end

    // VSYNC 엣지 검출
    logic vsync_prev;
    always @(posedge pclk) begin
        vsync_prev <= vsync;
        if (~vsync & vsync_prev) $display("[%0d] >>> VSYNC FALLING EDGE <<<", $time);
        if (vsync & ~vsync_prev) $display("[%0d] >>> VSYNC RISING EDGE <<<", $time);
    end

endmodule
