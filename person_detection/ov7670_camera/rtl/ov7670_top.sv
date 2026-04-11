// =================================================
// ov7670_top.sv: OV7670 Camera Top Module
// -------------------------------------------------
// OV7670 카메라 전체 시스템 통합
//
// 구조:
//     1. 클럭 생성 (125MHz -> 24MHz XCLK)
//     2. SCCB 초기화 (ov7670_config)
//     3. 픽셀 캡처 (ov7670_capture)
//     4. CDC FIFO (PCLK -> System Clock)
//     5. 출력 인터페이스
// 동작 시퀀스:
//     1. 리셋 해제
//     2. OV7670 초기화 (SCCB 레지스터 설정)
//     3. 프레임 캡처 시작
//     4. 연속 캡처 모드
// =================================================

module ov7670_top #(
    parameter SYS_CLK_FREQ = 125_000_000,
    parameter FRAME_WIDTH = 320,
    parameter FRAME_HEIGHT = 240
)(
    // ------------- 시스템 인터페이스 --------------
    input clk, rstn,    // 시스템 클럭 (125MHz) / 리셋 (active LOW)

    // -------------- OV7670 물리 핀 ---------------
    // 카메라 데이터 입력
    input       ov_pclk,              // 픽셀 클럭 (~25MHz)
    input       ov_href, ov_vsync,    // 수평 참조 / 수직 동기
    input [7:0] ov_data,              // 픽셀 데이터

    // 카메라 클럭 출력
    output ov_xclk,

    // SCCB 인터페이스
    output ov_scl,    // SCCB 클럭
    inout  ov_sda,    // SCCB 데이터

    // ----------- 프레임 출력 인터페이스 ------------
    // 시스템 클럭 도메인 (125MHz)
    output        frame_valid,    // 프레임 데이터 유효 
//    output [15:0] frame_data,     // RGB565 데이터
    output        frame_done,     // 프레임 완료 (1 펄스)

    // 상태 출력
    output reg init_done,    // 초기화 완료
    output     capturing     // 캡처 진행 중
    );

    // ============= 클럭 생성 (PLL/MMCM) ===========
    wire clk_24m;
    wire pll_locked;
    
    ov7670_clk_gen u_clk_gen (.sys_clk_125m(clk), .reset(!rstn), .cam_xclk_24m(clk_24m), .locked(pll_locked));

    assign ov_xclk = clk_24m;

    // ================ SCCB 초기화 =================
    logic config_start;
    wire  config_done, config_err;

    ov7670_config #(.SYS_CLK_FREQ(SYS_CLK_FREQ)) u_ov7670_config (
        .clk(clk), .rstn(rstn),
        .start(config_start), .done(config_done), .err(config_err),
        .scl(ov_scl), .sda(ov_sda));

    // ============== 초기화 시퀀스 제어 ==============
    typedef enum logic [1:0] {INIT_IDLE, INIT_PLL, INIT_WAIT, INIT_CONFIG} init_state_t;
    init_state_t init_state;

    localparam POWERUP_WAIT = 125_000_000 / 10;    // 100ms @125MHz
    logic [23:0] wait_cnt;    // 대기 카운터 (파워업 대기용)

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            init_state <= INIT_IDLE;
            wait_cnt <= 0;
            config_start <= 0;
            init_done <= 0;
        end
        else begin
            config_start <= 1'b0;    // 1 클럭 펄스
            
            case (init_state)
                INIT_IDLE: begin
                    init_state <= INIT_PLL;
                    wait_cnt <= '0;
                end
                INIT_PLL: begin    // PLL 잠금 대기
                    if (pll_locked) begin
                        init_state <= INIT_WAIT;
                        wait_cnt <= '0;
                    end
                end
                INIT_WAIT: begin    // 파워업 대기 (~100ms)
                    if (wait_cnt == POWERUP_WAIT - 1) begin
                        init_state <= INIT_CONFIG;
                        config_start <= 1'b1;                  // SCCB 초기화 시작
                    end
                    else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end
                INIT_CONFIG: begin    // SCCB 설정
                    if (config_done) begin
                        init_done <= 1'b1;    // 완료 후 계속 유지
                    end
                end
            endcase
        end
    end

    // ================= 픽셀 캡처 ==================
    wire cap_wEn;
    wire [16:0] cap_wAddr;
    wire [15:0] cap_wData;
    wire cap_frame_done;

    ov7670_capture #(.FRAME_WIDTH(FRAME_WIDTH), .FRAME_HEIGHT(FRAME_HEIGHT)) u_ov7670_capture (
        .rstn(rstn & init_done),
        .ov_pclk(ov_pclk), .ov_href(ov_href), .ov_vsync(ov_vsync), .ov_data(ov_data),
        .frame_wEn(cap_wEn), .frame_wAddr(cap_wAddr), .frame_wData(cap_wData), .frame_done(cap_frame_done), .capturing(capturing));

    // ====== CDC FIFO (PCLK -> System Clock) ======
    wire [15:0] frame_data;
    
    wire fifo_wAck;
    wire fifo_full, fifo_almost_full;
    wire fifo_rEn;
    wire fifo_empty, fifo_almost_empty;
    wire fifo_overflow, fifo_underflow;

    ov7670_async_fifo u_fifo (
        .rst(!rstn),
        .wr_clk(ov_pclk), .wr_en(cap_wEn & ~fifo_full), .din(cap_wData), .full(fifo_full), .almost_full(fifo_almost_full), .wr_ack(fifo_wAck),
        .rd_clk(clk), .rd_en(fifo_rEn), .dout(frame_data), .empty(fifo_empty), .almost_empty(fifo_almost_empty), .valid(frame_valid),
        .overflow(fifo_overflow), .underflow(fifo_underflow));

    // ============ FIFO 읽기 제어 로직 =============
    // FIFO에 데이터 있으면 계속 읽기 (실제 시스템에서는 다음 단계의 ready 신호 확인 필요)
    assign fifo_rEn = ~fifo_empty;

    // 프레임 완료 신호 (PCLK 도메인 -> System 클럭 도메인 CDC) 2FF 동기화
    logic [1:0] frame_done_sync;

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) frame_done_sync <= 0;
        else frame_done_sync <= {frame_done_sync[0], cap_frame_done};
    end

    assign frame_done = frame_done_sync[1] & ~frame_done_sync[0];    // 엣지 감지

endmodule
