// uart_tasks.cpp
#include "uart_tasks.h"
#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

void uartLidarTask(void* /*pvParameters*/) {
    uint8_t  buf[256];
    uint32_t totalBytes = 0;          // ← 함수 내부 선언
    uint32_t lastLog    = millis();   // ← 함수 내부 선언

    Serial.printf("[LIDAR] Task on Core %d\n", xPortGetCoreID());

    while (true) {
        int avail = Serial2.available();
        if (avail > 0) {
            int n = Serial2.readBytes(buf,
                        min(avail, static_cast<int>(sizeof(buf))));
            if (n > 0) {
                totalBytes += n;
                if (xRingbufferSend(xRingBuf_Lidar, buf, n,
                                    pdMS_TO_TICKS(5)) != pdTRUE) {
                    size_t dsz;
                    void* dump = xRingbufferReceiveUpTo(
                                     xRingBuf_Lidar, &dsz, 0, 256);
                    if (dump) vRingbufferReturnItem(xRingBuf_Lidar, dump);
                    xRingbufferSend(xRingBuf_Lidar, buf, n, 0);
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // 5초마다 LiDAR 수신 바이트 수 출력
        if (millis() - lastLog >= 5000) {
            Serial.printf("[LIDAR] RX %u B/5s (%.1f B/s)\n",
                          totalBytes, totalBytes / 5.0f);
            totalBytes = 0;
            lastLog    = millis();
        }
    }
}

void uartBoardTask(void* /*pvParameters*/) {
    uint8_t buf[64];
    Serial.printf("[BOARD] Task on Core %d\n", xPortGetCoreID());

    while (true) {
        int avail = Serial1.available();
        if (avail > 0) {
            int n = Serial1.readBytes(buf,
                        min(avail, static_cast<int>(sizeof(buf))));
            if (n > 0) {
                if (xRingbufferSend(xRingBuf_Board, buf, n,
                                    pdMS_TO_TICKS(10)) != pdTRUE) {
                    Serial.println("[BOARD] Ring overflow, drop");
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}