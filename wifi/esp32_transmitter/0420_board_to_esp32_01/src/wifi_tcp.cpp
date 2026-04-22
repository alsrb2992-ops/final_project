#include "wifi_tcp.h"
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

WiFiClient tcpClient;

void ensureWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.println("[WiFi] Reconnecting...");
    WiFi.reconnect();
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED)
        Serial.println("\n[WiFi] Reconnected: " + WiFi.localIP().toString());
}

bool ensureTCP() {
    if (tcpClient.connected()) return true;
    Serial.printf("[TCP] Connecting to %s:%d\n", PC_IP, PC_PORT);
    if (tcpClient.connect(PC_IP, PC_PORT)) {
        tcpClient.setNoDelay(true);   // Nagle 비활성화 — 소형 패킷 지연 방지
        Serial.println("[TCP] Connected!");
        return true;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    return false;
}

bool tcpSendWrapped(uint8_t chan, const uint8_t* payload, uint16_t len) {
    uint8_t hdr[5] = {
        TCP_MAGIC_1, TCP_MAGIC_2, chan,
        static_cast<uint8_t>(len & 0xFFu),
        static_cast<uint8_t>((len >> 8) & 0xFFu)
    };

    // write() 반환값이 0이면 소켓 완전 종료, 부분 전송은 허용
    // (TCP 백프레셔 상황에서 부분 전송은 정상 동작)
    int w1 = tcpClient.write(hdr, 5);
    if (w1 == 0) return false;

    int w2 = tcpClient.write(payload, len);
    if (w2 == 0) return false;

    return true;
}

