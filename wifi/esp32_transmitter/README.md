# ESP32 Transmitter (COM14)

Lidar로부터 UART 데이터를 수신하여 WiFi를 통해 Receiver ESP32로 전달하는 펌웨어입니다.

## 상세 사양
- **플랫폼**: PlatformIO (Arduino Framework)
- **통신 포트**: COM14 (기본값)
- **주요 역할**:
  - Lidar 하드웨어로부터 UART 데이터 수신
  - 수신된 데이터를 TCP/IP 기반 WiFi 통신으로 전송
  - 데이터의 무결성을 위한 래퍼(Wrapper) 처리

## 프로젝트 구조
- `src/main.cpp`: WiFi 연결 및 태스크 스케줄링
- `src/uart_tasks.cpp`: Lidar 데이터 수신 전용 태스크
- `src/wifi_tcp.cpp`: TCP 클라이언트 통신 로직

## 사용 방법
1. PlatformIO IDE에서 프로젝트를 엽니다.
2. `platformio.ini` 설정을 확인한 후 빌드 및 업로드(Upload)를 진행합니다.
3. `.pio` 및 `.vscode` 폴더는 환경에 따라 자동 생성되므로 포함되어 있지 않습니다.