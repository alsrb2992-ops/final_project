`timescale 1 ns / 1 ps

module PL_PS_1myip_v1_0 #
(
    // Parameters of Axi Slave Bus Interface S00_AXI
    parameter integer C_S00_AXI_DATA_WIDTH  = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH  = 4,

    // Parameters of Axi Master Bus Interface M00_AXIS
    parameter integer C_M00_AXIS_TDATA_WIDTH = 32,
    parameter integer C_M00_AXIS_START_COUNT = 32
)
(
    // ── User 포트: OV7670 카메라 ──────────────────────────────────────
    input  wire        cam_pclk,
    input  wire        cam_href,
    input  wire        cam_vsync,
    input  wire [7:0]  cam_d,

    // ── User 포트: SCCB ───────────────────────────────────────────────
    output wire        sio_c,
    inout  wire        sio_d,

    // Ports of Axi Slave Bus Interface S00_AXI
    input  wire                                s00_axi_aclk,
    input  wire                                s00_axi_aresetn,
    input  wire [C_S00_AXI_ADDR_WIDTH-1 : 0]  s00_axi_awaddr,
    input  wire [2 : 0]                        s00_axi_awprot,
    input  wire                                s00_axi_awvalid,
    output wire                                s00_axi_awready,
    input  wire [C_S00_AXI_DATA_WIDTH-1 : 0]  s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb,
    input  wire                                s00_axi_wvalid,
    output wire                                s00_axi_wready,
    output wire [1 : 0]                        s00_axi_bresp,
    output wire                                s00_axi_bvalid,
    input  wire                                s00_axi_bready,
    input  wire [C_S00_AXI_ADDR_WIDTH-1 : 0]  s00_axi_araddr,
    input  wire [2 : 0]                        s00_axi_arprot,
    input  wire                                s00_axi_arvalid,
    output wire                                s00_axi_arready,
    output wire [C_S00_AXI_DATA_WIDTH-1 : 0]  s00_axi_rdata,
    output wire [1 : 0]                        s00_axi_rresp,
    output wire                                s00_axi_rvalid,
    input  wire                                s00_axi_rready,

    // Ports of Axi Master Bus Interface M00_AXIS
    input  wire                                  m00_axis_aclk,
    input  wire                                  m00_axis_aresetn,
    output wire                                  m00_axis_tvalid,
    output wire [C_M00_AXIS_TDATA_WIDTH-1 : 0]   m00_axis_tdata,
    output wire [(C_M00_AXIS_TDATA_WIDTH/8)-1:0] m00_axis_tstrb,
    output wire                                  m00_axis_tlast,
    input  wire                                  m00_axis_tready
);

    // ── 내부 클럭/리셋 ────────────────────────────────────────────────
    wire clk   = m00_axis_aclk;
    wire rst_n = m00_axis_aresetn;
    wire reset = ~rst_n;

    // ── S00_AXI ↔ CNN 연결 wire ───────────────────────────────────────
    wire        w_pixel_valid;
    wire [15:0] w_pixel_r;
    wire [15:0] w_pixel_g;
    wire [15:0] w_pixel_b;
    wire        w_grid_valid;
    wire [7:0]  w_grid_prob;
    wire        w_inference_done;

    // ── M00_AXIS ↔ OV7670_MemController 연결 wire ────────────────────
    wire        w_m_axis_tvalid;
    wire [15:0] w_m_axis_tdata;
    wire        w_m_axis_tlast;
    wire        w_m_axis_tready;

    // =========================================================================
    // AXI4-Lite Slave 인터페이스 (CNN 레지스터)
    // =========================================================================
    PL_PS_1myip_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH (C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH (C_S00_AXI_ADDR_WIDTH)
    ) PL_PS_1myip_v1_0_S00_AXI_inst (
        .pixel_valid    (w_pixel_valid),
        .pixel_r        (w_pixel_r),
        .pixel_g        (w_pixel_g),
        .pixel_b        (w_pixel_b),
        .grid_valid     (w_grid_valid),
        .grid_prob      (w_grid_prob),
        .inference_done (w_inference_done),
        .S_AXI_ACLK    (s00_axi_aclk),
        .S_AXI_ARESETN (s00_axi_aresetn),
        .S_AXI_AWADDR  (s00_axi_awaddr),
        .S_AXI_AWPROT  (s00_axi_awprot),
        .S_AXI_AWVALID (s00_axi_awvalid),
        .S_AXI_AWREADY (s00_axi_awready),
        .S_AXI_WDATA   (s00_axi_wdata),
        .S_AXI_WSTRB   (s00_axi_wstrb),
        .S_AXI_WVALID  (s00_axi_wvalid),
        .S_AXI_WREADY  (s00_axi_wready),
        .S_AXI_BRESP   (s00_axi_bresp),
        .S_AXI_BVALID  (s00_axi_bvalid),
        .S_AXI_BREADY  (s00_axi_bready),
        .S_AXI_ARADDR  (s00_axi_araddr),
        .S_AXI_ARPROT  (s00_axi_arprot),
        .S_AXI_ARVALID (s00_axi_arvalid),
        .S_AXI_ARREADY (s00_axi_arready),
        .S_AXI_RDATA   (s00_axi_rdata),
        .S_AXI_RRESP   (s00_axi_rresp),
        .S_AXI_RVALID  (s00_axi_rvalid),
        .S_AXI_RREADY  (s00_axi_rready)
    );

    // =========================================================================
    // AXI4-Stream Master 인터페이스
    // =========================================================================
    PL_PS_1myip_v1_0_M00_AXIS #(
        .C_M_AXIS_TDATA_WIDTH (C_M00_AXIS_TDATA_WIDTH),
        .C_M_START_COUNT      (C_M00_AXIS_START_COUNT)
    ) PL_PS_1myip_v1_0_M00_AXIS_inst (
        .m_axis_tvalid (w_m_axis_tvalid),
        .m_axis_tdata  (w_m_axis_tdata),
        .m_axis_tlast  (w_m_axis_tlast),
        .m_axis_tready (w_m_axis_tready),
        .M_AXIS_ACLK   (m00_axis_aclk),
        .M_AXIS_ARESETN(m00_axis_aresetn),
        .M_AXIS_TVALID (m00_axis_tvalid),
        .M_AXIS_TDATA  (m00_axis_tdata),
        .M_AXIS_TSTRB  (m00_axis_tstrb),
        .M_AXIS_TLAST  (m00_axis_tlast),
        .M_AXIS_TREADY (m00_axis_tready)
    );

    // =========================================================================
    // SCCB 초기화
    // =========================================================================
    SCCB_top u_sccb_top (
        .clk   (clk),
        .reset (reset),
        .SIO_C (sio_c),
        .SIO_D (sio_d)
    );

    // =========================================================================
    // OV7670 MemController → AXI4-Stream
    // =========================================================================
    OV7670_MemController u_mem_ctrl (
        .pclk          (cam_pclk),
        .rst_n         (rst_n),
        .href          (cam_href),
        .vsync         (cam_vsync),
        .data          (cam_d),
        .m_axis_tvalid (w_m_axis_tvalid),
        .m_axis_tdata  (w_m_axis_tdata),
        .m_axis_tlast  (w_m_axis_tlast),
        .m_axis_tready (w_m_axis_tready)
    );

    // =========================================================================
    // CNN 추론
    // =========================================================================
    cnn_top u_cnn (
        .clk            (clk),
        .rst_n          (rst_n),
        .pixel_valid    (w_pixel_valid),
        .pixel_r        (w_pixel_r),
        .pixel_g        (w_pixel_g),
        .pixel_b        (w_pixel_b),
        .grid_valid     (w_grid_valid),
        .grid_prob      (w_grid_prob),
        .inference_done (w_inference_done)
    );

endmodule