#pragma once
#include <Arduino.h>
#include <WiFi.h>

// tcpSendTask 단일 호출자 → critical section 불필요, tcpMux 제거
extern WiFiClient tcpClient;

void ensureWiFi();
bool ensureTCP();
bool tcpSendWrapped(uint8_t chan, const uint8_t* payload, uint16_t len);