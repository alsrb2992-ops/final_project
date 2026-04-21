#pragma once
#include <cstdint>

// ─── WiFi SoftAP ──────────────────────────────────────────────────────────────
// ESP32 #1 이 이 AP 에 연결함 (공유기 우회)
// AP 기본 게이트웨이: 192.168.4.1  ← ESP32 #1 의 PC_IP 와 일치해야 함
#define AP_SSID             "ESP32_BRIDGE_AP"
#define AP_PASSWORD         "bridge1234"       // 최소 8자
#define AP_CHANNEL          6
#define AP_MAX_CLIENTS      1                  // ESP32 #1 단독 연결

// ─── TCP Server ───────────────────────────────────────────────────────────────
#define TCP_PORT            8888

// ─── PC 데이터 UART (Serial / UART0 / USB-CDC) ────────────────────────────────
// LiDAR raw ≈ 12.8 kB/s + 프레이밍 오버헤드 → 460800 baud = 약 3.5× 여유
#define PC_UART_BAUD        460800
#define PC_UART_TX_BUF      8192               // Serial TX 소프트웨어 버퍼

// ─── 디버그 UART (Serial2 / GPIO16=RX, GPIO17=TX) ────────────────────────────
// 데이터 스트림과 완전 분리. USB-UART 어댑터로 별도 모니터링 (선택)
#define DBG_UART_BAUD       115200
#define DBG_RX_PIN          16
#define DBG_TX_PIN          17

// ─── Status LED ───────────────────────────────────────────────────────────────
#define STATUS_LED_PIN      2                  // 대부분의 ESP32 DevKit 내장 LED

// ─── TCP Framing (ESP32 #1 과 동일) ───────────────────────────────────────────
#define TCP_MAGIC_1         0xEFu
#define TCP_MAGIC_2         0xBEu
#define CHAN_LIDAR          0x01u
#define CHAN_BOARD          0x02u
#define CHAN_CMD_TO_BOARD   0x03u

// ─── 릴레이 버퍼 ─────────────────────────────────────────────────────────────
#define FWD_BUF_SIZE        2048u