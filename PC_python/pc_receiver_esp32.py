#!/usr/bin/env python3
"""
pc_receiver_esp32.py
  PC ← USB/UART(460800) ← ESP32 #2 ← WiFi ← ESP32 #1 ← LiDAR / FPGA

  CHAN=0x01 : YDLiDAR X4PRO scan packets  → [LIDAR] FSA / LSA / hex dump
  CHAN=0x02 : FPGA board packets           → [FPGA]  RPi / CNN / SHT40

Usage:
  python pc_receiver_esp32.py [PORT] [BAUD]

  Windows  기본 PORT : COM3
  Linux    기본 PORT : /dev/ttyUSB0
  기본 BAUD : 460800
"""

import sys
import struct
import serial          # pip install pyserial

# ─── 기본 설정 ────────────────────────────────────────────────────────────────
DEFAULT_PORT = "COM3"           # Linux: "/dev/ttyUSB0"
DEFAULT_BAUD = 460800

# ─── TCP 래퍼 상수 (ESP32 와 동일) ───────────────────────────────────────────
MAGIC1          = 0xEF
MAGIC2          = 0xBE
CHAN_LIDAR      = 0x01
CHAN_BOARD      = 0x02

# ─── FPGA 패킷 상수 ──────────────────────────────────────────────────────────
BOARD_STX1      = 0xAB
BOARD_STX2      = 0xCD
TYPE_RPI        = 0x01
TYPE_CNN        = 0x02
TYPE_SHT        = 0x03


# ─────────────────────────────────────────────────────────────────────────────
# LiDAR 패킷 파서 (X4PRO 개발 매뉴얼 §5 기준)
# ─────────────────────────────────────────────────────────────────────────────
class LidarParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        while self._try_parse():
            pass

    def _try_parse(self) -> bool:
        # PH = 0x55 0xAA 동기화
        while len(self.buf) >= 2:
            if self.buf[0] == 0x55 and self.buf[1] == 0xAA:
                break
            self.buf.pop(0)

        if len(self.buf) < 10:
            return False

        lsn     = self.buf[3]
        pkt_len = 10 + 2 * lsn

        if len(self.buf) < pkt_len:
            return False

        pkt = bytes(self.buf[:pkt_len])
        del self.buf[:pkt_len]

        ct      = pkt[2]
        fsa_raw = struct.unpack_from('<H', pkt, 4)[0]
        lsa_raw = struct.unpack_from('<H', pkt, 6)[0]

        # 1차 각도 계산 (매뉴얼 §5.4)
        angle_fsa = (fsa_raw >> 1) / 64.0
        angle_lsa = (lsa_raw >> 1) / 64.0
        is_start  = bool(ct & 0x01)

        hex_str = ' '.join(f'{b:02X}' for b in pkt)
        if is_start:
            print()
        print(f"[LIDAR]{'[START]' if is_start else '       '} "
              f"FSA={angle_fsa:6.2f}° LSA={angle_lsa:6.2f}° LSN={lsn:3d} | {hex_str}")
        return True


# ─────────────────────────────────────────────────────────────────────────────
# FPGA 보드 패킷 파서
# 패킷: [0xAB][0xCD][TYPE][LEN][PAYLOAD...][XOR_CS]
# ─────────────────────────────────────────────────────────────────────────────
class BoardParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        while self._try_parse():
            pass

    def _try_parse(self) -> bool:
        # STX 동기화
        while len(self.buf) >= 2:
            if self.buf[0] == BOARD_STX1 and self.buf[1] == BOARD_STX2:
                break
            self.buf.pop(0)

        if len(self.buf) < 4:
            return False

        ptype = self.buf[2]
        plen  = self.buf[3]
        total = 4 + plen + 1   # header(4) + payload + CS(1)

        if len(self.buf) < total:
            return False

        payload  = bytes(self.buf[4 : 4 + plen])
        cs_recv  = self.buf[4 + plen]
        del self.buf[:total]

        cs_calc = ptype ^ plen
        for b in payload:
            cs_calc ^= b

        if cs_calc != cs_recv:
            print(f"[FPGA] CS ERROR  type=0x{ptype:02X} "
                  f"expected=0x{cs_calc:02X} got=0x{cs_recv:02X}")
            return True

        self._dispatch(ptype, payload)
        return True

    def _dispatch(self, ptype: int, payload: bytes):
        if ptype == TYPE_RPI:
            print("[FPGA] RPi:   *** RESCUE SIGNAL DETECTED ***")

        elif ptype == TYPE_CNN:
            if payload:
                person = bool(payload[0] & 0x01)
                print(f"[FPGA] CNN:   person={'YES' if person else 'NO '}  "
                      f"(raw=0x{payload[0]:02X})")

        elif ptype == TYPE_SHT:
            if len(payload) >= 4:
                raw_t = struct.unpack_from('>H', payload, 0)[0]
                raw_h = struct.unpack_from('>H', payload, 2)[0]
                temp  = -45.0 + 175.0 * raw_t / 65535.0
                humi  = max(0.0, min(100.0, -6.0 + 125.0 * raw_h / 65535.0))
                print(f"[FPGA] SHT40: Temp={temp:5.1f}°C  Humidity={humi:4.1f}%")

        else:
            hex_str = ' '.join(f'{b:02X}' for b in payload)
            print(f"[FPGA] Unknown type=0x{ptype:02X}  payload={hex_str}")


# ─────────────────────────────────────────────────────────────────────────────
# 메인: Serial 포트 열기 → 래퍼 FSM 파싱 → 채널별 파서 위임
# ─────────────────────────────────────────────────────────────────────────────
def run(port: str, baud: int):
    # timeout=0 : non-blocking read, 데이터 없으면 빈 bytes 반환
    # 빠른 폴링으로 지연 최소화
    ser = serial.Serial(port, baud, bytesize=8, parity='N',
                        stopbits=1, timeout=0.02)

    print(f"[System] Serial receiver: {port} @ {baud} baud")
    print(f"         ESP32 #2 연결 대기 중...\n")

    lidar_p = LidarParser()
    board_p = BoardParser()

    state  = "MAGIC1"
    chan   = 0
    length = 0
    cnt    = 0
    pbuf   = bytearray()

    while True:
        chunk = ser.read(4096)
        if not chunk:
            continue

        for b in chunk:
            if state == "MAGIC1":
                if b == MAGIC1:
                    state = "MAGIC2"

            elif state == "MAGIC2":
                if   b == MAGIC2: state = "CHAN"
                elif b == MAGIC1: state = "MAGIC2"
                else:             state = "MAGIC1"

            elif state == "CHAN":
                chan  = b
                state = "LEN_L"

            elif state == "LEN_L":
                length = b
                state  = "LEN_H"

            elif state == "LEN_H":
                length |= (b << 8)
                cnt = 0
                pbuf.clear()
                state = "PAYLOAD" if length > 0 else "MAGIC1"

            elif state == "PAYLOAD":
                pbuf.append(b)
                cnt += 1
                if cnt == length:
                    payload = bytes(pbuf)
                    if   chan == CHAN_LIDAR: lidar_p.feed(payload)
                    elif chan == CHAN_BOARD: board_p.feed(payload)
                    state = "MAGIC1"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    try:
        run(port, baud)
    except serial.SerialException as e:
        print(f"[ERROR] 포트 열기 실패: {e}")
        print("  사용 가능 포트 확인: python -m serial.tools.list_ports")
    except KeyboardInterrupt:
        print("\n[System] 종료.")


if __name__ == "__main__":
    main()