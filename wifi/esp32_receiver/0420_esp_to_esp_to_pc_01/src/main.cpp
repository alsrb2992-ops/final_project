#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "wifi_server.h"
#include "tcp_rx_task.h"

void setup() {
    // ── 데이터 UART (UART0 / USB) ─────────────────────────────────────────────
    // 이 시리얼에는 디버그 출력 일절 금지 — PC 파서가 raw 프레임 스트림으로 읽음
    Serial.setTxBufferSize(PC_UART_TX_BUF);
    Serial.begin(PC_UART_BAUD);

    // ── 디버그 UART (UART2 / GPIO16=RX, GPIO17=TX) ────────────────────────────
    Serial2.begin(DBG_UART_BAUD, SERIAL_8N1, DBG_RX_PIN, DBG_TX_PIN);
    Serial2.println("\n[System] ESP32 Receiver Bridge v1.0");
    Serial2.printf("[System] Data UART0 @ %d baud\n", PC_UART_BAUD);

    // ── Status LED ────────────────────────────────────────────────────────────
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // ── SoftAP + TCP 서버 시작 ────────────────────────────────────────────────
    startAP();

    // ── 포워딩 태스크 (Core 0) ────────────────────────────────────────────────
    xTaskCreatePinnedToCore(tcpForwardTask, "TCP_FWD", 8192, NULL, 5, NULL, 0);
    Serial2.println("[System] tcpForwardTask launched. Waiting for ESP32 #1...");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}