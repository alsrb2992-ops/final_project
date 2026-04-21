// config.h
#pragma once
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>

// ─── LiDAR UART (Serial2) ─── RX only, TX unused ───────────────────────────
#define LIDAR_RX_PIN        16
#define LIDAR_BAUD          128000

// ─── Board UART (Serial1) ────────────────────────────────────────────────────
#define BOARD_RX_PIN        18
#define BOARD_TX_PIN        19
#define BOARD_BAUD          115200

// ─── WiFi ── ESP32 #2 SoftAP 에 직접 연결 (공유기 우회) ─────────────────────
#define WIFI_SSID           "ESP32_BRIDGE_AP"
#define WIFI_PASSWORD       "bridge1234"

// ─── TCP ── ESP32 #2 SoftAP 기본 IP ──────────────────────────────────────────
#define PC_IP               "192.168.4.1"
#define PC_PORT             8888

// ─── TCP Framing Constants ────────────────────────────────────────────────────
#define TCP_MAGIC_1         0xEFu
#define TCP_MAGIC_2         0xBEu
#define CHAN_LIDAR          0x01u
#define CHAN_BOARD          0x02u
#define CHAN_CMD_TO_BOARD   0x03u

// ─── Ring Buffer Sizes ────────────────────────────────────────────────────────
#define LIDAR_RING_SIZE     (16 * 1024)
#define BOARD_RING_SIZE     ( 2 * 1024)

// ─── Shared Ring Buffer Handles (defined in main.cpp) ─────────────────────────
extern RingbufHandle_t xRingBuf_Lidar;
extern RingbufHandle_t xRingBuf_Board;