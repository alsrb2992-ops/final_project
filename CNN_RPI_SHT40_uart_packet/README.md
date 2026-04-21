# CNN RPi SHT40 UART Packet (FPGA/HDL)

이 폴더는 Basys3 FPGA 보드에서 동작하는 SystemVerilog 소스 코드를 포함하고 있습니다.

## 주요 기능
- **SHT40 제어**: I2C 통신을 통해 SHT40 온습도 센서 데이터를 읽어옵니다.
- **CNN UART 패키지**: RPi(라즈베리 파이) 및 하위 시스템과 통신하기 위한 고유 패키지 구조를 생성합니다.
- **UART 전송**: 수집된 센서 데이터 및 상태 정보를 정해진 프로토콜에 맞춰 외부(ESP32)로 전송합니다.

## 주요 모듈 구성
- `UART_RPI_CNN_TOP.sv`: 전체 시스템의 최상위 모듈
- `sht40_i2c.sv`: SHT40 센서 I2C 인터페이스 로직
- `packet_builder.sv`: 데이터를 전송 규격에 맞게 패킷화하는 로직
- `uart_tx.sv / uart_rx.sv`: UART 통신 물리 계층 구현

## 비고
- Vivado 프로젝트에서 `sources_1` 및 `constrs_1`을 참조하여 빌드하십시오.