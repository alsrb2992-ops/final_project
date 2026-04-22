#!/usr/bin/env python3
"""
pc_receiver_esp32_ver11.py
Usage: python pc_receiver_esp32_ver11.py COM15 460800
"""

import sys
import struct
import serial
import time
import math
import threading
import collections

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec

MAGIC1     = 0xEF
MAGIC2     = 0xBE
CHAN_LIDAR = 0x01
CHAN_BOARD = 0x02

BOARD_STX1 = 0xAB
BOARD_STX2 = 0xCD
TYPE_RPI   = 0x01
TYPE_CNN   = 0x02
TYPE_SHT   = 0x03

lock         = threading.Lock()
latest_xy    = ([], [])
scan_updated = False
packet_log   = collections.deque(maxlen=10)
sht_log      = collections.deque(maxlen=5)
conn_status  = {"wifi": False, "scan_no": 0,
                "temp": None, "humi": None}


class LidarParser:
    def __init__(self):
        self.buf     = bytearray()
        self.scan_no = 0
        self._cx     = []
        self._cy     = []

    def feed(self, data: bytes):
        self.buf.extend(data)
        while self._try_parse():
            pass

    def _try_parse(self) -> bool:
        global latest_xy, scan_updated

        while len(self.buf) >= 2:
            if self.buf[0] == 0xAA and self.buf[1] == 0x55:
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

        ct       = pkt[2]
        fsa_raw  = struct.unpack_from('<H', pkt, 4)[0]
        lsa_raw  = struct.unpack_from('<H', pkt, 6)[0]
        is_start = bool(ct & 0x01)

        angle_fsa = (fsa_raw >> 1) / 64.0
        angle_lsa = (lsa_raw >> 1) / 64.0
        diff = angle_lsa - angle_fsa
        if diff < 0:
            diff += 360.0

        if is_start:
            if self._cx:
                with lock:
                    latest_xy    = (self._cx[:], self._cy[:])
                    scan_updated = True
            self._cx.clear()
            self._cy.clear()
            self.scan_no += 1
            with lock:
                conn_status["scan_no"] = self.scan_no
                packet_log.append(
                    f"[START] SCAN#{self.scan_no:5d}  "
                    f"angle={angle_fsa:6.2f} deg"
                )
        else:
            valid = 0
            for i in range(lsn):
                si_raw  = struct.unpack_from('<H', pkt, 10 + 2 * i)[0]
                dist_mm = si_raw >> 2
                if dist_mm > 0:
                    angle = angle_fsa if lsn == 1 else \
                            angle_fsa + diff * i / (lsn - 1)
                    rad = math.radians(angle)
                    self._cx.append(dist_mm * math.cos(rad))
                    self._cy.append(dist_mm * math.sin(rad))
                    valid += 1
            with lock:
                packet_log.append(
                    f"  PKT  FSA={angle_fsa:6.2f}  "
                    f"LSA={angle_lsa:6.2f}  "
                    f"LSN={lsn:2d}  valid={valid:2d}pts"
                )
        return True


class BoardParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        while self._try_parse():
            pass

    def _try_parse(self) -> bool:
        while len(self.buf) >= 2:
            if self.buf[0] == BOARD_STX1 and self.buf[1] == BOARD_STX2:
                break
            self.buf.pop(0)
        if len(self.buf) < 4:
            return False
        ptype = self.buf[2]
        plen  = self.buf[3]
        total = 4 + plen + 1
        if len(self.buf) < total:
            return False
        payload  = bytes(self.buf[4 : 4 + plen])
        cs_recv  = self.buf[4 + plen]
        del self.buf[:total]
        cs_calc = ptype ^ plen
        for b in payload:
            cs_calc ^= b
        if cs_calc != cs_recv:
            return True
        self._dispatch(ptype, payload)
        return True

    def _dispatch(self, ptype, payload):
        if ptype == TYPE_RPI:
            print("[FPGA] RPi: *** RESCUE SIGNAL ***", flush=True)
        elif ptype == TYPE_CNN:
            if payload:
                print(f"[FPGA] CNN: person="
                      f"{'YES' if payload[0]&1 else 'NO '}", flush=True)
        elif ptype == TYPE_SHT:
            if len(payload) >= 4:
                raw_t = struct.unpack_from('>H', payload, 0)[0]
                raw_h = struct.unpack_from('>H', payload, 2)[0]
                temp  = -45.0 + 175.0 * raw_t / 65535.0
                humi  = max(0.0, min(100.0,
                            -6.0 + 125.0 * raw_h / 65535.0))
                ts = time.strftime('%H:%M:%S')
                with lock:
                    conn_status["temp"] = temp
                    conn_status["humi"] = humi
                    sht_log.append(
                        f"[{ts}]  {temp:5.1f}C  {humi:4.1f}%"
                    )
                print(f"[FPGA] SHT40: {temp:5.1f}C  {humi:4.1f}%",
                      flush=True)


def serial_thread(port, baud):
    lidar_p = LidarParser()
    board_p = BoardParser()

    while True:
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
            try:
                ser.set_buffer_size(rx_size=65536, tx_size=65536)
            except Exception:
                pass

            with lock:
                conn_status["wifi"] = True
            print(f"[System] Connected: {port} @ {baud}", flush=True)

            state = "MAGIC1"
            chan = length = cnt = 0
            pbuf = bytearray()

            while True:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                for b in chunk:
                    if state == "MAGIC1":
                        if b == MAGIC1: state = "MAGIC2"
                    elif state == "MAGIC2":
                        if   b == MAGIC2: state = "CHAN"
                        elif b == MAGIC1: state = "MAGIC2"
                        else:             state = "MAGIC1"
                    elif state == "CHAN":
                        chan  = b; state = "LEN_L"
                    elif state == "LEN_L":
                        length = b; state = "LEN_H"
                    elif state == "LEN_H":
                        length |= (b << 8)
                        cnt = 0; pbuf.clear()
                        state = ("PAYLOAD"
                                 if 0 < length <= 4096
                                 else "MAGIC1")
                    elif state == "PAYLOAD":
                        pbuf.append(b); cnt += 1
                        if cnt == length:
                            pl = bytes(pbuf)
                            if   chan == CHAN_LIDAR: lidar_p.feed(pl)
                            elif chan == CHAN_BOARD: board_p.feed(pl)
                            state = "MAGIC1"

        except Exception as e:
            with lock:
                conn_status["wifi"] = False
            print(f"[System] Serial error: {e} -- retrying...",
                  flush=True)
            time.sleep(2)


def main():
    if len(sys.argv) < 3:
        print("Usage: python pc_receiver_esp32_ver11.py <PORT> <BAUD>")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2])

    t = threading.Thread(target=serial_thread,
                         args=(port, baud), daemon=True)
    t.start()

    # ── GUI 레이아웃: 상단바 / 맵 / LiDAR로그 / SHT로그 ─────────────────
    fig = plt.figure(figsize=(9, 12))
    gs  = gridspec.GridSpec(3, 1,
                            height_ratios=[4, 1, 1],
                            hspace=0.4)

    # ── 상단: 2D Map ─────────────────────────────────────────────────────
    ax_map = fig.add_subplot(gs[0])
    ax_map.set_aspect('equal')
    ax_map.set_title("LiDAR 2D Map (mm)", fontsize=12)
    ax_map.set_xlim(-6000, 6000)
    ax_map.set_ylim(-6000, 6000)
    ax_map.grid(True, alpha=0.3)
    ax_map.plot(0, 0, 'r^', markersize=10)
    scat = ax_map.scatter([], [], s=2, c='steelblue', alpha=0.8)

    # 연결 상태 (좌상단)
    status_txt = ax_map.text(
        0.01, 0.99, "Waiting...",
        transform=ax_map.transAxes,
        va='top', ha='left', fontsize=9,
        bbox=dict(boxstyle='round', fc='white', alpha=0.8)
    )

    # 온습도 (우상단)
    sht_txt = ax_map.text(
        0.99, 0.99, "SHT40: -- C  -- %",
        transform=ax_map.transAxes,
        va='top', ha='right', fontsize=11,
        color='navy',
        bbox=dict(boxstyle='round', fc='lightyellow', alpha=0.9)
    )

    # ── 중단: LiDAR 패킷 로그 ────────────────────────────────────────────
    ax_pkt = fig.add_subplot(gs[1])
    ax_pkt.axis('off')
    ax_pkt.set_title("LiDAR Packet Log", fontsize=10, pad=3)
    pkt_txt = ax_pkt.text(
        0.01, 0.99, "Waiting for LiDAR...",
        transform=ax_pkt.transAxes,
        va='top', ha='left', fontsize=8,
        family='monospace',
        bbox=dict(boxstyle='round', fc='#f0f0f0', alpha=0.9)
    )

    # ── 하단: SHT40 로그 ─────────────────────────────────────────────────
    ax_sht = fig.add_subplot(gs[2])
    ax_sht.axis('off')
    ax_sht.set_title("SHT40 Log", fontsize=10, pad=3)
    sht_log_txt = ax_sht.text(
        0.01, 0.99, "Waiting for SHT40...",
        transform=ax_sht.transAxes,
        va='top', ha='left', fontsize=8,
        family='monospace',
        bbox=dict(boxstyle='round', fc='#fff8e1', alpha=0.9)
    )

    def update(_):
        global scan_updated

        with lock:
            wifi   = conn_status["wifi"]
            scan_n = conn_status["scan_no"]
            temp   = conn_status["temp"]
            humi   = conn_status["humi"]
            pkts   = list(packet_log)
            shts   = list(sht_log)
            if scan_updated:
                xs, ys = latest_xy
                scan_updated = False
            else:
                xs, ys = None, None

        # 연결 상태
        status_txt.set_text(
            f"Connected  SCAN#{scan_n}" if wifi else "Waiting..."
        )
        status_txt.set_color('green' if wifi else 'red')

        # 온습도 우상단
        if temp is not None:
            sht_txt.set_text(f"SHT40: {temp:5.1f}C  {humi:4.1f}%")
        else:
            sht_txt.set_text("SHT40: -- C  -- %")

        # LiDAR 포인트
        if xs is not None:
            scat.set_offsets(list(zip(xs, ys)))

        # LiDAR 패킷 로그
        pkt_txt.set_text(
            "\n".join(pkts) if pkts else "Waiting for LiDAR..."
        )

        # SHT40 로그
        sht_log_txt.set_text(
            "\n".join(shts) if shts else "Waiting for SHT40..."
        )

        return scat, status_txt, sht_txt, pkt_txt, sht_log_txt

    ani = animation.FuncAnimation(
        fig, update, interval=150,
        blit=True, cache_frame_data=False
    )

    print(f"[System] GUI started. Port={port}", flush=True)
    plt.show()


if __name__ == "__main__":
    main()