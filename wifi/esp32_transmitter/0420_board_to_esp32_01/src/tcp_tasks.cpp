#include "tcp_tasks.h"
#include "config.h"
#include "wifi_tcp.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

// ─── Task 3: TCP 전송 (Core 0, Priority 4) ───────────────────────────────────
void tcpSendTask(void* /*pvParameters*/) {
    Serial.printf("[TCP_TX] Task on Core %d\n", xPortGetCoreID());

    bool wasConnected = false;

    while (true) {
        ensureWiFi();
        if (!ensureTCP()) {
            wasConnected = false;
            continue;
        }

        // 재연결 직후 1회: Board 링버퍼 stale 데이터 드레인
        // SHT40 과거 누적값 + 소멸된 RPi 펄스 제거 → 버스트 방지
        if (!wasConnected) {
            size_t dsz;
            void* dump;
            while ((dump = xRingbufferReceiveUpTo(
                        xRingBuf_Board, &dsz, 0, 64)) != NULL) {
                vRingbufferReturnItem(xRingBuf_Board, dump);
            }
            Serial.println("[TCP_TX] Board buf drained on reconnect");
            wasConnected = true;
        }

        bool sent = false;

        // Priority 1: Board (FPGA) channel
        {
            size_t sz = 0;
            void* item = xRingbufferReceiveUpTo(
                             xRingBuf_Board, &sz, 0, 64);
            if (item && sz > 0) {
                if (!tcpSendWrapped(CHAN_BOARD,
                                    static_cast<uint8_t*>(item),
                                    static_cast<uint16_t>(sz))) {
                    Serial.println("[TCP_TX] Board write failed, reconnect");
                    tcpClient.stop();
                    wasConnected = false;
                }
                vRingbufferReturnItem(xRingBuf_Board, item);
                sent = true;
            }
        }

        // Priority 2: LiDAR channel (batch up to TCP MSS)
        // LiDAR는 드레인 없이 이어서 전송
        {
            size_t sz = 0;
            void* item = xRingbufferReceiveUpTo(
                             xRingBuf_Lidar, &sz, pdMS_TO_TICKS(5), 1460);
            if (item && sz > 0) {
                if (!tcpSendWrapped(CHAN_LIDAR,
                                    static_cast<uint8_t*>(item),
                                    static_cast<uint16_t>(sz))) {
                    Serial.println("[TCP_TX] LiDAR write failed, reconnect");
                    tcpClient.stop();
                    wasConnected = false;
                }
                vRingbufferReturnItem(xRingBuf_Lidar, item);
                sent = true;
            }
        }

        if (!sent) vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ─── Task 4: TCP 수신 (Core 0, Priority 3) ───────────────────────────────────
void tcpReceiveTask(void* /*pvParameters*/) {
    Serial.printf("[TCP_RX] Task on Core %d\n", xPortGetCoreID());

    enum RxState : uint8_t {
        RX_MAGIC1 = 0, RX_MAGIC2, RX_CHAN,
        RX_LEN_L, RX_LEN_H, RX_PAYLOAD
    };

    RxState  rxState = RX_MAGIC1;
    uint8_t  rxChan  = 0;
    uint16_t rxLen = 0, rxCnt = 0;
    uint8_t  rxBuf[512];

    while (true) {
        ensureWiFi();
        if (!ensureTCP()) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        while (tcpClient.available() > 0) {
            uint8_t b = static_cast<uint8_t>(tcpClient.read());

            switch (rxState) {
                case RX_MAGIC1:
                    rxState = (b == TCP_MAGIC_1) ? RX_MAGIC2 : RX_MAGIC1;
                    break;
                case RX_MAGIC2:
                    if      (b == TCP_MAGIC_2) rxState = RX_CHAN;
                    else if (b == TCP_MAGIC_1) rxState = RX_MAGIC2;
                    else                       rxState = RX_MAGIC1;
                    break;
                case RX_CHAN:
                    rxChan = b; rxState = RX_LEN_L;
                    break;
                case RX_LEN_L:
                    rxLen = b; rxState = RX_LEN_H;
                    break;
                case RX_LEN_H:
                    rxLen |= (static_cast<uint16_t>(b) << 8);
                    rxCnt = 0;
                    rxState = (rxLen == 0 || rxLen > sizeof(rxBuf))
                              ? RX_MAGIC1 : RX_PAYLOAD;
                    if (rxLen > sizeof(rxBuf))
                        Serial.printf("[TCP_RX] Oversize %u, discard\n", rxLen);
                    break;
                case RX_PAYLOAD:
                    rxBuf[rxCnt++] = b;
                    if (rxCnt == rxLen) {
                        if (rxChan == CHAN_CMD_TO_BOARD) {
                            Serial1.write(rxBuf, rxLen);
                            Serial.printf("[TCP_RX] CMD→Board %u B\n", rxLen);
                        }
                        rxState = RX_MAGIC1;
                    }
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// #include "tcp_tasks.h"
// #include "config.h"
// #include "wifi_tcp.h"
// #include <Arduino.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <freertos/ringbuf.h>

// // ─── Task 3: TCP 전송 (Core 0, Priority 4) ───────────────────────────────────
// // Board 채널 우선(저속·저지연), LiDAR 채널 MSS 단위 배치
// void tcpSendTask(void* /*pvParameters*/) {
//     Serial.printf("[TCP_TX] Task on Core %d\n", xPortGetCoreID());

//     while (true) {
//         ensureWiFi();
//         if (!ensureTCP()) continue;

//         bool sent = false;

//         // Priority 1: Board (FPGA) channel
//         {
//             size_t sz = 0;
//             void* item = xRingbufferReceiveUpTo(
//                              xRingBuf_Board, &sz, 0, 64);
//             if (item && sz > 0) {
//                 if (!tcpSendWrapped(CHAN_BOARD,
//                                     static_cast<uint8_t*>(item),
//                                     static_cast<uint16_t>(sz))) {
//                     Serial.println("[TCP_TX] Board write failed, reconnect");
//                     tcpClient.stop();
//                 }
//                 vRingbufferReturnItem(xRingBuf_Board, item);
//                 sent = true;
//             }
//         }

//         // Priority 2: LiDAR channel (batch up to TCP MSS)
//         {
//             size_t sz = 0;
//             void* item = xRingbufferReceiveUpTo(
//                              xRingBuf_Lidar, &sz, pdMS_TO_TICKS(5), 1460);
//             if (item && sz > 0) {
//                 if (!tcpSendWrapped(CHAN_LIDAR,
//                                     static_cast<uint8_t*>(item),
//                                     static_cast<uint16_t>(sz))) {
//                     Serial.println("[TCP_TX] LiDAR write failed, reconnect");
//                     tcpClient.stop();
//                 }
//                 vRingbufferReturnItem(xRingBuf_Lidar, item);
//                 sent = true;
//             }
//         }

//         if (!sent) vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

// // ─── Task 4: TCP 수신 (Core 0, Priority 3) ───────────────────────────────────
// // PC → ESP32 → Serial1(Basys3) 커맨드 포워딩
// void tcpReceiveTask(void* /*pvParameters*/) {
//     Serial.printf("[TCP_RX] Task on Core %d\n", xPortGetCoreID());

//     enum RxState : uint8_t {
//         RX_MAGIC1 = 0, RX_MAGIC2, RX_CHAN,
//         RX_LEN_L, RX_LEN_H, RX_PAYLOAD
//     };

//     RxState  rxState = RX_MAGIC1;
//     uint8_t  rxChan  = 0;
//     uint16_t rxLen = 0, rxCnt = 0;
//     uint8_t  rxBuf[512];

//     while (true) {
//         ensureWiFi();
//         if (!ensureTCP()) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

//         while (tcpClient.available() > 0) {
//             uint8_t b = static_cast<uint8_t>(tcpClient.read());

//             switch (rxState) {
//                 case RX_MAGIC1:
//                     rxState = (b == TCP_MAGIC_1) ? RX_MAGIC2 : RX_MAGIC1;
//                     break;
//                 case RX_MAGIC2:
//                     if      (b == TCP_MAGIC_2) rxState = RX_CHAN;
//                     else if (b == TCP_MAGIC_1) rxState = RX_MAGIC2;
//                     else                       rxState = RX_MAGIC1;
//                     break;
//                 case RX_CHAN:
//                     rxChan = b; rxState = RX_LEN_L;
//                     break;
//                 case RX_LEN_L:
//                     rxLen = b; rxState = RX_LEN_H;
//                     break;
//                 case RX_LEN_H:
//                     rxLen |= (static_cast<uint16_t>(b) << 8);
//                     rxCnt = 0;
//                     rxState = (rxLen == 0 || rxLen > sizeof(rxBuf))
//                               ? RX_MAGIC1 : RX_PAYLOAD;
//                     if (rxLen > sizeof(rxBuf))
//                         Serial.printf("[TCP_RX] Oversize %u, discard\n", rxLen);
//                     break;
//                 case RX_PAYLOAD:
//                     rxBuf[rxCnt++] = b;
//                     if (rxCnt == rxLen) {
//                         if (rxChan == CHAN_CMD_TO_BOARD) {
//                             Serial1.write(rxBuf, rxLen);
//                             Serial.printf("[TCP_RX] CMD→Board %u B\n", rxLen);
//                         }
//                         rxState = RX_MAGIC1;
//                     }
//                     break;
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(5));
//     }
// }