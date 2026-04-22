// =============================================================================
// ov7670_cam.v  —  OV7670 카메라 인터페이스 (RGB444 → Q4.12)
// =============================================================================
// OV7670 RGB444 타이밍:
//   첫 번째 바이트: [7:4]=R[3:0], [3:0]=G[3:0]
//   두 번째 바이트: [7:4]=B[3:0], [3:0]=don't care
//   → 2클럭에 1픽셀
//
// Q4.12 변환:
//   4bit 색상값을 12비트 왼쪽 시프트
//   예) R=4'hF → 16'hF000 (Q4.12에서 정수부 표현)
// =============================================================================

module ov7670_cam (
    input  wire        clk,
    input  wire        rst_n,

    // OV7670 핀
    input  wire        pclk,
    input  wire        href,
    input  wire        vsync,
    input  wire [7:0]  d,

    // CNN 입력
    output reg         pixel_valid,
    output reg  [15:0] pixel_r,
    output reg  [15:0] pixel_g,
    output reg  [15:0] pixel_b
);

    // pclk 상승 엣지 감지 (시스템 클럭 도메인으로 동기화)
    reg pclk_d;
    wire pclk_rise = (~pclk_d) & pclk;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pclk_d <= 1'b0;
        else        pclk_d <= pclk;
    end

    // RGB444 수신 상태머신
    // href=1 구간에서 2바이트씩 수신
    reg        byte_sel;   // 0: 첫 번째 바이트, 1: 두 번째 바이트
    reg [7:0]  first_byte; // 첫 번째 바이트 래치

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_sel    <= 1'b0;
            first_byte  <= 8'h0;
            pixel_valid <= 1'b0;
            pixel_r     <= 16'h0;
            pixel_g     <= 16'h0;
            pixel_b     <= 16'h0;
        end
        else begin
            pixel_valid <= 1'b0;

            // [Latch 방지] byte_sel, first_byte default 유지 할당
            // vsync=0, pclk_rise=0, href=0 등 조건 불만족 시
            // 할당이 없으면 Vivado가 latch를 추론함
            // → 매 사이클 자기 자신을 유지하도록 명시
            byte_sel   <= byte_sel;
            first_byte <= first_byte;

            if (vsync) begin
                // 프레임 시작 시 리셋 (위의 default 유지를 덮어씀)
                byte_sel <= 1'b0;
            end
            else if (pclk_rise && href) begin
                if (!byte_sel) begin
                    // 첫 번째 바이트: R[3:0]=d[7:4], G[3:0]=d[3:0]
                    // (위의 default 유지를 덮어씀)
                    first_byte <= d;
                    byte_sel   <= 1'b1;
                end
                else begin
                    // 두 번째 바이트: B[3:0]=d[7:4]
                    // Q4.12 변환: 4bit → 16bit (<<12)
                    pixel_r <= {first_byte[7:4], 12'b0};
                    pixel_g <= {first_byte[3:0], 12'b0};
                    pixel_b <= {d[7:4],          12'b0};
                    pixel_valid <= 1'b1;
                    byte_sel    <= 1'b0;
                end
            end
        end
    end

endmodule