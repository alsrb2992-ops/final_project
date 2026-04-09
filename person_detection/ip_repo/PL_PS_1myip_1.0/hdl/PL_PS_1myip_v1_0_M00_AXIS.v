`timescale 1 ns / 1 ps

module PL_PS_1myip_v1_0_M00_AXIS #
(
    parameter integer C_M_AXIS_TDATA_WIDTH = 32,
    parameter integer C_M_START_COUNT      = 32
)
(
    // ── User 포트 (OV7670_MemController → 여기) ───────────────────────
    input  wire        m_axis_tvalid,
    input  wire [15:0] m_axis_tdata,
    input  wire        m_axis_tlast,
    output wire        m_axis_tready,

    // ── AXI4-Stream Master 포트 ───────────────────────────────────────
    input  wire                                M_AXIS_ACLK,
    input  wire                                M_AXIS_ARESETN,
    output wire                                M_AXIS_TVALID,
    output wire [C_M_AXIS_TDATA_WIDTH-1 : 0]   M_AXIS_TDATA,
    output wire [(C_M_AXIS_TDATA_WIDTH/8)-1:0] M_AXIS_TSTRB,
    output wire                                M_AXIS_TLAST,
    input  wire                                M_AXIS_TREADY
);

    // OV7670_MemController 출력을 AXI4-Stream 포트로 연결
    assign M_AXIS_TVALID = m_axis_tvalid;
    assign M_AXIS_TDATA  = {16'b0, m_axis_tdata}; // 16bit → 32bit 패딩
    assign M_AXIS_TLAST  = m_axis_tlast;
    assign M_AXIS_TSTRB  = {(C_M_AXIS_TDATA_WIDTH/8){1'b1}};
    assign m_axis_tready = M_AXIS_TREADY;

endmodule