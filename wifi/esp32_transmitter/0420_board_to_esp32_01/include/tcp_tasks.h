#pragma once

// Core 0에 핀: Ring Buffer → TCP 전송 / PC 명령 수신 → Serial1
void tcpSendTask(void* pvParameters);
void tcpReceiveTask(void* pvParameters);