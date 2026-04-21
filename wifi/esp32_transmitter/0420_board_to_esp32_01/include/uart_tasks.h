#pragma once

// Core 1에 핀: LiDAR(RX only) + Board UART 수신 → Ring Buffer push
void uartLidarTask(void* pvParameters);
void uartBoardTask(void* pvParameters);