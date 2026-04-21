#include "uart_tasks.h"
#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

// ─── Task 1: LiDAR UART (Serial2, Core 1, Priority 5) ───────────────────────
// 128 kbaud ≈ 12800 B/s. buf[256]로 Serial 내부 64B FIFO를 빠르게 비움.
void uartLidarTask(void* /*pvParameters*/) {
    uint8_t buf[256];
    Serial.printf("[LIDAR] Task on Core %d\n", xPortGetCoreID());

    while (true) {
        int avail = Serial2.available();
        if (avail > 0) {
            int n = Serial2.readBytes(buf,
                        min(avail, static_cast<int>(sizeof(buf))));
            if (n > 0) {
                if (xRingbufferSend(xRingBuf_Lidar, buf, n,
                                    pdMS_TO_TICKS(5)) != pdTRUE) {
                    // 오버플로 시 가장 오래된 청크 드롭 후 재시도
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
    }
}

// ─── Task 2: Board UART (Serial1, Core 1, Priority 5) ───────────────────────
// FPGA: 최소 1Hz SHT40, 비동기 RPi/CNN 이벤트
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