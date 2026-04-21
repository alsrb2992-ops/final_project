#pragma once
#include <WiFi.h>

extern WiFiServer tcpServer;

// SoftAP 시작 + TCP 서버 바인드
void startAP();