#include "wifi_server.h"
#include "config.h"
#include <Arduino.h>

WiFiServer tcpServer(TCP_PORT);

void startAP() {
    WiFi.mode(WIFI_AP);
    // channel, hidden=false, max_connection=1
    bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS);
    if (!ok) {
        Serial2.println("[AP] softAP() failed — check password length (>=8)");
    } else {
        Serial2.printf("[AP] SSID=%s  GW=%s\n",
                       AP_SSID,
                       WiFi.softAPIP().toString().c_str());
    }

    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial2.printf("[TCP] Server listening on port %d\n", TCP_PORT);
}