# ESP32 Receiver (COM15)

Transmitter ESP32로부터 WiFi 데이터를 수신하여 PC로 전달하는 브리지 역할을 합니다.

## 상세 사양
- **플랫폼**: PlatformIO (Arduino Framework)
- **통신 포트**: COM15 (기본값)
- **주요 역할**:
  - WiFi를 통해 들어오는 TCP 데이터 수신
  - 수신된 데이터를 USB-UART(PC)로 고속 전송 (Baudrate: 460800 권장)

## 프로젝트 구조
- `src/main.cpp`: WiFi 서버 설정 및 데이터 포워딩
- `src/tcp_rx_task.cpp`: 네트워크 데이터 수신 처리
- `src/wifi_server.cpp`: WiFi AP/Station 모드 설정

## 사용 방법
1. PlatformIO IDE에서 프로젝트를 엽니다.
2. `platformio.ini`에서 보드 설정을 확인합니다.
3. 빌드 후 업로드하면 Transmitter와 자동으로 페어링되어 데이터를 수신합니다.