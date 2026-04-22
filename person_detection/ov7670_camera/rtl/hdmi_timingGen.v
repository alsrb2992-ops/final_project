// ===============================================================
// hdmi_timingGen.v: HDMI 640x480 @ 60Hz 타이밍 생성기
// ---------------------------------------------------------------
// 기능:
//     - HSYNC, VSYNC, DE(Data Enable) 생성
//     - 픽셀 좌표 출력 (pxl_x, pxl_y)
// 타이밍:
//     해상도: 640x480 @ 60Hz
//     픽셀 클럭: 25MHz
//
//     H: 640 (visible) + 16 (front) + 96 (sync) + 48 (back) = 800
//     V: 480 (visible) + 10 (front) + 2 (sync) + 33 (back) = 525
// 출력:
//     - hsync, vsync: 동기 신호
//     - de: 유효 픽셀 영역 표시
//     - pxl_x, pxl_y: 현재 픽셀 좌표 (0-639, 0-479)
// ===============================================================

module hdmi_timingGen(
    input clk, rstn,

    output       hsync, vsync,
    output       de,
    output [9:0] pxl_x, pxl_y
    );

    // ===================== 타이밍 파라미터 ======================
    localparam H_VISIBLE = 640;
    localparam H_FRONT   = 16;
    localparam H_SYNC    = 96;
    localparam H_BACK    = 48;
    localparam H_TOTAL   = H_VISIBLE + H_FRONT + H_SYNC + H_BACK;    // 800

    localparam V_VISIBLE = 480;
    localparam V_FRONT   = 10;
    localparam V_SYNC    = 2;
    localparam V_BACK    = 33;
    localparam V_TOTAL   = V_VISIBLE + V_FRONT + V_SYNC + V_BACK;    // 525

    // ==================== 수평/수직 카운터 ======================
    reg [9:0] h_cnt, v_cnt;    // 0-799 / 0-524

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            h_cnt <= 0; v_cnt <= 0;
        end
        else begin
            if (h_cnt == H_TOTAL - 1) begin
                h_cnt <= 0;
                if (v_cnt == V_TOTAL - 1) v_cnt <= 0;
                else                      v_cnt <= v_cnt + 1;
            end
            else begin
                h_cnt <= h_cnt + 1;
            end
        end
    end

    // ======================= HSYNC 생성 ========================
    assign hsync = !((h_cnt >= H_VISIBLE + H_FRONT) && (h_cnt < H_VISIBLE + H_FRONT + H_SYNC));

    // ======================= VSYNC 생성 ========================
    assign vsync = !((v_cnt >= V_VISIBLE + V_FRONT) && (v_cnt < V_VISIBLE + V_FRONT + V_SYNC));

    // ======================== DE 생성 ==========================
    assign de = (h_cnt < H_VISIBLE) && (v_cnt < V_VISIBLE);

    // ==================== 픽셀 좌표 출력 ========================
    assign pxl_x = h_cnt;
    assign pxl_y = v_cnt;

endmodule
