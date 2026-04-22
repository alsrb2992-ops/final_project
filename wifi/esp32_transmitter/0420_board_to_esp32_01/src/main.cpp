#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

#include "config.h"
#include "wifi_tcp.h"
#include "uart_tasks.h"
#include "tcp_tasks.h"

// Ring buffer 실체 (config.h 에서 extern 선언됨)
RingbufHandle_t xRingBuf_Lidar = NULL;
RingbufHandle_t xRingBuf_Board = NULL;

void setup() {
    Serial.begin(115200);
    Serial.println("\n[System] ESP32 Dual-Channel Bridge v3.0");

    // LiDAR: RX only (TX=-1, 전송 불필요)
    Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, -1);
    Serial.printf("[UART2] LiDAR  RX=GPIO%d (TX unused) BAUD=%d\n",
                  LIDAR_RX_PIN, LIDAR_BAUD);

    // Board (FPGA)
    Serial1.begin(BOARD_BAUD, SERIAL_8N1, BOARD_RX_PIN, BOARD_TX_PIN);
    pinMode(BOARD_RX_PIN, INPUT_PULLUP);  // GPIO18 플로팅 방지
    Serial.printf("[UART1] Board  RX=GPIO%d TX=GPIO%d BAUD=%d\n",
                  BOARD_RX_PIN, BOARD_TX_PIN, BOARD_BAUD);

    // WiFi 초기 연결
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Connecting");
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(WiFi.status() == WL_CONNECTED
        ? "\n[WiFi] OK: " + WiFi.localIP().toString()
        : "\n[WiFi] Boot timeout (retry in tasks)");

    // Ring Buffer 생성
    xRingBuf_Lidar = xRingbufferCreate(LIDAR_RING_SIZE, RINGBUF_TYPE_BYTEBUF);
    xRingBuf_Board = xRingbufferCreate(BOARD_RING_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!xRingBuf_Lidar || !xRingBuf_Board) {
        Serial.println("[ERROR] Ring buffer alloc failed! Halting.");
        while (true) delay(1000);
    }
    Serial.printf("[RingBuf] Lidar=%uB  Board=%uB\n",
                  LIDAR_RING_SIZE, BOARD_RING_SIZE);

    // Core 1: UART 수신 (높은 우선순위로 FIFO 오버플로 방지)
    xTaskCreatePinnedToCore(uartLidarTask, "LIDAR_RX", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(uartBoardTask, "BOARD_RX", 4096, NULL, 5, NULL, 1);

    // Core 0: TCP 송수신
    xTaskCreatePinnedToCore(tcpSendTask,    "TCP_TX",  8192, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(tcpReceiveTask, "TCP_RX",  8192, NULL, 3, NULL, 0);

    Serial.println("[System] All 4 tasks launched.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}