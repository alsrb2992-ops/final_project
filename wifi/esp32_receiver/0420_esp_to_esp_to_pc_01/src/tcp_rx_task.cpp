#include "tcp_rx_task.h"
#include "wifi_server.h"
#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static uint8_t fwdBuf[FWD_BUF_SIZE];

void tcpForwardTask(void* /*pv*/) {
    Serial2.printf("[FWD] Task on Core %d\n", xPortGetCoreID());

    while (true) {
        WiFiClient client = tcpServer.available();
        if (!client) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        client.setNoDelay(true);
        Serial2.printf("[FWD] Connected: %s\n",
                       client.remoteIP().toString().c_str());
        digitalWrite(STATUS_LED_PIN, HIGH);

        // 재연결 시 Serial TX 버퍼 잔여물 제거
        Serial.flush();

        while (client.connected()) {
            int avail = client.available();
            if (avail > 0) {
                int n = client.read(fwdBuf,
                            min(avail, static_cast<int>(FWD_BUF_SIZE)));
                if (n > 0) {
                    // TX 버퍼 여유 확인 후 write — 블로킹 방지
                    int writable = Serial.availableForWrite();
                    if (writable >= n) {
                        Serial.write(fwdBuf, static_cast<size_t>(n));
                    } else if (writable > 0) {
                        // 쓸 수 있는 만큼만 write, 나머지는 드롭
                        // LiDAR 스트림 특성상 일부 드롭이 전체 블로킹보다 낫습니다
                        Serial.write(fwdBuf, static_cast<size_t>(writable));
                    }
                    // writable == 0: 이번 청크 드롭, 다음 루프에서 재시도
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        client.stop();
        digitalWrite(STATUS_LED_PIN, LOW);
        Serial2.println("[FWD] Client disconnected");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// #include "tcp_rx_task.h"
// #include "wifi_server.h"
// #include "config.h"
// #include <Arduino.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

// static uint8_t fwdBuf[FWD_BUF_SIZE];

// void tcpForwardTask(void* /*pv*/) {
//     Serial2.printf("[FWD] Task on Core %d\n", xPortGetCoreID());

//     while (true) {
//         // ── 클라이언트 연결 대기 ──────────────────────────────────────────────
//         WiFiClient client = tcpServer.available();
//         if (!client) {
//             vTaskDelay(pdMS_TO_TICKS(10));
//             continue;
//         }

//         // Nagle 비활성화: 소형 패킷 지연 방지
//         client.setNoDelay(true);

//         Serial2.printf("[FWD] Connected: %s\n",
//                        client.remoteIP().toString().c_str());
//         digitalWrite(STATUS_LED_PIN, HIGH);

//         // ── 릴레이 루프 ───────────────────────────────────────────────────────
//         // 파싱 없이 수신 바이트를 Serial(PC) 로 그대로 전달.
//         // 프레이밍(0xEF 0xBE + 채널 + 길이 + 페이로드)은 ESP32 #1 이 보장.
//         while (client.connected()) {
//             int avail = client.available();
//             if (avail > 0) {
//                 int n = client.read(fwdBuf,
//                             min(avail, static_cast<int>(FWD_BUF_SIZE)));
//                 if (n > 0) {
//                     Serial.write(fwdBuf, static_cast<size_t>(n));
//                 }
//             } else {
//                 vTaskDelay(pdMS_TO_TICKS(1));
//             }
//         }

//         // ── 연결 종료 ─────────────────────────────────────────────────────────
//         client.stop();
//         digitalWrite(STATUS_LED_PIN, LOW);
//         Serial2.println("[FWD] Client disconnected — waiting for reconnect");
//         vTaskDelay(pdMS_TO_TICKS(500)); // ← 소켓 정리 대기
//     }
// }