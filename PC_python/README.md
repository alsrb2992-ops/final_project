# PC Receiver Script (Python)

ESP32 Receiver로부터 들어오는 로그 데이터를 실시간으로 파싱하고 출력하는 테스트용 스크립트입니다.

## 파일 정보
- `pc_receiver_esp32.py`: 메인 실행 스크립트

## 기능
- **패킷 파싱**: 0xEF, 0xBE 매직 넘버를 기반으로 패킷의 시작을 감지합니다.
- **채널 구분**:
  - `CHAN 0x01`: Lidar 데이터 (YDLiDAR X4PRO)
  - `CHAN 0x02`: FPGA 보드 데이터 (RPi/CNN/SHT40)
- **로그 출력**: 수신된 데이터를 터미널에 가독성 있는 형식으로 출력합니다.

## 실행 방법
```bash
# 기본 실행 (포트: COM3, 보레이트: 460800)
python pc_receiver_esp32.py

# 특정 포트 지정 실행
python pc_receiver_esp32.py COM15 460800