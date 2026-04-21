#include "wifi_tcp.h"
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

WiFiClient   tcpClient;
portMUX_TYPE tcpMux = portMUX_INITIALIZER_UNLOCKED;

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
        Serial.println("[TCP] Connected!");
        return true;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    return false;
}

// [0xEF][0xBE][chan][len_lo][len_hi][payload...]
bool tcpSendWrapped(uint8_t chan, const uint8_t* payload, uint16_t len) {
    uint8_t hdr[5] = {
        TCP_MAGIC_1, TCP_MAGIC_2, chan,
        static_cast<uint8_t>(len & 0xFFu),
        static_cast<uint8_t>((len >> 8) & 0xFFu)
    };
    // portENTER_CRITICAL 제거 — tcpSendTask 단일 호출자이므로 불필요
    // critical section 내 TCP write는 WDT 리셋 유발
    int w1 = tcpClient.write(hdr, 5);
    int w2 = tcpClient.write(payload, len);
    return (w1 == 5 && w2 == static_cast<int>(len));
}