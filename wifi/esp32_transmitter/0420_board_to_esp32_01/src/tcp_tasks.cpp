//가장 최신 버전, 0421_오후 7:36
#include "tcp_tasks.h"
#include "config.h"
#include "wifi_tcp.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

void tcpSendTask(void* /*pvParameters*/) {
    Serial.printf("[TCP_TX] Task on Core %d\n", xPortGetCoreID());

    bool wasConnected = false;

    while (true) {
        ensureWiFi();
        if (!ensureTCP()) {
            wasConnected = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 재연결 직후 1회: Board + LiDAR 링버퍼 stale 데이터 전부 드레인 오후 8시 경 수정
        // if (!wasConnected) {
        //     size_t dsz;
        //     void* dump;
        //     while ((dump = xRingbufferReceiveUpTo(
        //                 xRingBuf_Board, &dsz, 0, 64)) != NULL)
        //         vRingbufferReturnItem(xRingBuf_Board, dump);

        //     while ((dump = xRingbufferReceiveUpTo(
        //                 xRingBuf_Lidar, &dsz, 0, 1460)) != NULL)
        //         vRingbufferReturnItem(xRingBuf_Lidar, dump);

        //     Serial.println("[TCP_TX] Both bufs drained on reconnect");
        //     wasConnected = true;
        // }
        if (!wasConnected) {
            size_t dsz;
            void* dump;
            while ((dump = xRingbufferReceiveUpTo(xRingBuf_Board, &dsz, 0, 64)) != NULL)
                vRingbufferReturnItem(xRingBuf_Board, dump);
            while ((dump = xRingbufferReceiveUpTo(xRingBuf_Lidar, &dsz, 0, 1460)) != NULL)
                vRingbufferReturnItem(xRingBuf_Lidar, dump);
            Serial.println("[TCP_TX] Both bufs drained on reconnect");
            wasConnected = true;
        }

        bool sent = false;

        // Priority 1: Board (FPGA) channel
        {
            size_t sz = 0;
            void* item = xRingbufferReceiveUpTo(xRingBuf_Board, &sz, 0, 64);
            if (item && sz > 0) {
                if (!tcpSendWrapped(CHAN_BOARD,
                                    static_cast<uint8_t*>(item),
                                    static_cast<uint16_t>(sz))) {
                    Serial.println("[TCP_TX] Board write failed, reconnect");
                    vRingbufferReturnItem(xRingBuf_Board, item);
                    tcpClient.stop();
                    wasConnected = false;
                    continue;
                }
                vRingbufferReturnItem(xRingBuf_Board, item);
                sent = true;
            }
        }

        // Priority 2: LiDAR channel
        {
            size_t sz = 0;
            void* item = xRingbufferReceiveUpTo(
                             xRingBuf_Lidar, &sz, pdMS_TO_TICKS(5), 512);
            if (item && sz > 0) {
                bool ok = tcpSendWrapped(CHAN_LIDAR,
                                         static_cast<uint8_t*>(item),
                                         static_cast<uint16_t>(sz));
                if (!ok) {
                    Serial.println("[TCP_TX] LIDAR write FAILED");
                    vRingbufferReturnItem(xRingBuf_Lidar, item);
                    tcpClient.stop();
                    wasConnected = false;
                    continue;
                }
                static uint32_t lidarSentBytes = 0;
                static uint32_t lastLog = 0;
                lidarSentBytes += sz;
                if (millis() - lastLog > 5000) {
                    Serial.printf("[TCP_TX] LIDAR sent %u B/5s\n", lidarSentBytes);
                    lidarSentBytes = 0;
                    lastLog = millis();
                }
                vRingbufferReturnItem(xRingBuf_Lidar, item);
                sent = true;
            }
        }

        if (!sent) vTaskDelay(pdMS_TO_TICKS(1));
    }
}

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
        if (!tcpClient.connected()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            rxState = RX_MAGIC1;
            continue;
        }

        int avail = tcpClient.available();
        if (avail <= 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

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
    }
}

/*근본 원인: tcp_tasks.cpp에서 재연결 시 BOARD 버퍼만 드레인하고 LiDAR 버퍼는 드레인하지 않습니다. 아래를 추가하면 됩니다:*/
//가장 최신버전 04_21_16:20
// #include "tcp_tasks.h"
// #include "config.h"
// #include "wifi_tcp.h"
// #include <Arduino.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <freertos/ringbuf.h>

// // ─── Task 3: TCP 전송 (Core 0, Priority 4) ───────────────────────────────────
// // TCP 연결 생명주기 단독 소유 — stop/connect 여기서만 호출
// void tcpSendTask(void* /*pvParameters*/) {
//     Serial.printf("[TCP_TX] Task on Core %d\n", xPortGetCoreID());

//     bool wasConnected = false;

//     while (true) {
//         ensureWiFi();
//         if (!ensureTCP()) {
//             wasConnected = false;
//             vTaskDelay(pdMS_TO_TICKS(100));
//             continue;
//         }

//         // 재연결 직후 1회: Board 링버퍼 stale 데이터 드레인
//         if (!wasConnected) {
//             size_t dsz;
//             void* dump;
//             while ((dump = xRingbufferReceiveUpTo(
//                         xRingBuf_Board, &dsz, 0, 64)) != NULL) {
//                 vRingbufferReturnItem(xRingBuf_Board, dump);
//             }
//             Serial.println("[TCP_TX] Board buf drained on reconnect");
//             wasConnected = true;
//         }

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
//                     vRingbufferReturnItem(xRingBuf_Board, item);
//                     tcpClient.stop();
//                     wasConnected = false;
//                     continue;   // 바로 루프 상단으로 — 재연결 시도
//                 }
//                 vRingbufferReturnItem(xRingBuf_Board, item);
//                 sent = true;
//             }
//         }

//         // Priority 2: LiDAR channel (batch up to TCP MSS)
//         {
//             size_t sz = 0;
//             void* item = xRingbufferReceiveUpTo(
//                              xRingBuf_Lidar, &sz, pdMS_TO_TICKS(5), 512); // 1460→512
//             if (item && sz > 0) {
//                 if (!tcpSendWrapped(CHAN_LIDAR,
//                                     static_cast<uint8_t*>(item),
//                                     static_cast<uint16_t>(sz))) {
//                     Serial.println("[TCP_TX] LiDAR write failed, reconnect");
//                     vRingbufferReturnItem(xRingBuf_Lidar, item);
//                     tcpClient.stop();
//                     wasConnected = false;
//                     continue;   // 바로 루프 상단으로 — 재연결 시도
//                 }
//                 vRingbufferReturnItem(xRingBuf_Lidar, item);
//                 sent = true;
//             }
//         }

//         if (!sent) vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

// // ─── Task 4: TCP 수신 (Core 0, Priority 3) ───────────────────────────────────
// // 연결 관리 완전 제거 — tcpSendTask가 소유한 소켓을 읽기만 함
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
//         // 연결 확인만 — stop/connect 절대 호출 안 함
//         if (!tcpClient.connected()) {
//             vTaskDelay(pdMS_TO_TICKS(100));
//             rxState = RX_MAGIC1;  // 재연결 시 FSM 리셋
//             continue;
//         }

//         int avail = tcpClient.available();
//         if (avail <= 0) {
//             vTaskDelay(pdMS_TO_TICKS(5));
//             continue;
//         }

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
//     }
// }


// 버전2(04_21_실패_tcpSendTask와 tcpReceiveTask 두 태스크가 동일한 tcpClient 객체를 동시에 제어하고 있습니다.)
// #include "tcp_tasks.h"
// #include "config.h"
// #include "wifi_tcp.h"
// #include <Arduino.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <freertos/ringbuf.h>

// // ─── Task 3: TCP 전송 (Core 0, Priority 4) ───────────────────────────────────
// void tcpSendTask(void* /*pvParameters*/) {
//     Serial.printf("[TCP_TX] Task on Core %d\n", xPortGetCoreID());

//     bool wasConnected = false;

//     while (true) {
//         ensureWiFi();
//         if (!ensureTCP()) {
//             wasConnected = false;
//             continue;
//         }

//         // 재연결 직후 1회: Board 링버퍼 stale 데이터 드레인
//         // SHT40 과거 누적값 + 소멸된 RPi 펄스 제거 → 버스트 방지
//         if (!wasConnected) {
//             size_t dsz;
//             void* dump;
//             while ((dump = xRingbufferReceiveUpTo(
//                         xRingBuf_Board, &dsz, 0, 64)) != NULL) {
//                 vRingbufferReturnItem(xRingBuf_Board, dump);
//             }
//             Serial.println("[TCP_TX] Board buf drained on reconnect");
//             wasConnected = true;
//         }

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
//                     wasConnected = false;
//                 }
//                 vRingbufferReturnItem(xRingBuf_Board, item);
//                 sent = true;
//             }
//         }

//         // Priority 2: LiDAR channel (batch up to TCP MSS)
//         // LiDAR는 드레인 없이 이어서 전송
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
//                     wasConnected = false;
//                 }
//                 vRingbufferReturnItem(xRingBuf_Lidar, item);
//                 sent = true;
//             }
//         }

//         if (!sent) vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }

// // ─── Task 4: TCP 수신 (Core 0, Priority 3) ───────────────────────────────────
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