#pragma once

// Core 0: ESP32 #1 로부터 TCP 수신 → Serial(UART0) 릴레이
void tcpForwardTask(void* pvParameters);