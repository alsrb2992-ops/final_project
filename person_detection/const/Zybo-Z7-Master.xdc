## =============================================================================
## Zybo Z7-20 XDC — CNN Person Detection (OV7670 + SCCB + CNN)
## Block Design 기반 (PS 클럭/리셋 사용)
## =============================================================================

## =============================================================================
## cam_pclk: OV7670 픽셀 클럭 (25MHz)
## =============================================================================
set_property -dict { PACKAGE_PIN N15 IOSTANDARD LVCMOS33 } [get_ports { cam_pclk }];
create_clock -add -name cam_pclk_pin -period 40.00 -waveform {0 20} [get_ports { cam_pclk }];
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets cam_pclk_IBUF]

## CDC: cam_pclk(25MHz) ↔ FCLK_CLK0(50MHz) 비동기 선언
set_clock_groups -asynchronous \
    -group [get_clocks cam_pclk_pin] \
    -group [get_clocks clk_fpga_0]

## =============================================================================
## Pmod JA — OV7670 제어 신호 + cam_d[4:0]
## =============================================================================
set_property -dict { PACKAGE_PIN L14 IOSTANDARD LVCMOS33 } [get_ports { cam_href }];
set_property -dict { PACKAGE_PIN K16 IOSTANDARD LVCMOS33 } [get_ports { cam_vsync }];
set_property -dict { PACKAGE_PIN K14 IOSTANDARD LVCMOS33 } [get_ports { cam_d[0] }];
set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 } [get_ports { cam_d[1] }];
set_property -dict { PACKAGE_PIN L15 IOSTANDARD LVCMOS33 } [get_ports { cam_d[2] }];
set_property -dict { PACKAGE_PIN J16 IOSTANDARD LVCMOS33 } [get_ports { cam_d[3] }];
set_property -dict { PACKAGE_PIN J14 IOSTANDARD LVCMOS33 } [get_ports { cam_d[4] }];

## =============================================================================
## Pmod JB — cam_d[7:5]
## =============================================================================
set_property -dict { PACKAGE_PIN V8  IOSTANDARD LVCMOS33 } [get_ports { cam_d[5] }];
set_property -dict { PACKAGE_PIN W8  IOSTANDARD LVCMOS33 } [get_ports { cam_d[6] }];
set_property -dict { PACKAGE_PIN U7  IOSTANDARD LVCMOS33 } [get_ports { cam_d[7] }];

## =============================================================================
## Pmod JD — SCCB
## =============================================================================
set_property -dict { PACKAGE_PIN T14 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { sio_c }];
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { sio_d }];

## =============================================================================
## 미사용 포트 (참고용)
## =============================================================================
## [제거됨] clk (K17): PS FCLK_CLK0으로 대체
## [제거됨] rst_n (K18): PS FCLK_RESET0_N으로 대체
## [제거됨] grid_valid (M14): Block Design 외부 핀 미노출
## [제거됨] inference_done (M15): Block Design 외부 핀 미노출

## LEDs (필요 시 주석 해제)
#set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports { grid_valid }];
#set_property -dict { PACKAGE_PIN M15 IOSTANDARD LVCMOS33 } [get_ports { inference_done }];
#set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports { led[2] }];
#set_property -dict { PACKAGE_PIN M17 IOSTANDARD LVCMOS33 } [get_ports { led[3] }];

## Switches (필요 시 주석 해제)
#set_property -dict { PACKAGE_PIN G15 IOSTANDARD LVCMOS33 } [get_ports { sw[0] }];
#set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports { sw[1] }];
#set_property -dict { PACKAGE_PIN W13 IOSTANDARD LVCMOS33 } [get_ports { sw[2] }];
#set_property -dict { PACKAGE_PIN T16 IOSTANDARD LVCMOS33 } [get_ports { sw[3] }];

## Buttons (필요 시 주석 해제)
#set_property -dict { PACKAGE_PIN K18 IOSTANDARD LVCMOS33 } [get_ports { btn[0] }];
#set_property -dict { PACKAGE_PIN P16 IOSTANDARD LVCMOS33 } [get_ports { btn[1] }];
#set_property -dict { PACKAGE_PIN K19 IOSTANDARD LVCMOS33 } [get_ports { btn[2] }];
#set_property -dict { PACKAGE_PIN Y16 IOSTANDARD LVCMOS33 } [get_ports { btn[3] }];

## Pmod JD 미사용 핀
#set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33 } [get_ports { jd[1] }];
#set_property -dict { PACKAGE_PIN R14 IOSTANDARD LVCMOS33 } [get_ports { jd[2] }];
#set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports { jd[3] }];
#set_property -dict { PACKAGE_PIN U15 IOSTANDARD LVCMOS33 } [get_ports { jd[4] }];
#set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports { jd[5] }];
#set_property -dict { PACKAGE_PIN V18 IOSTANDARD LVCMOS33 } [get_ports { jd[6] }];