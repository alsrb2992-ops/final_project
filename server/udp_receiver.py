# UDP 소켓 생성 및 데이터 수신 (LiDAR, 엔코더)
# 수신 데이터 -> parser.py (파싱) -> handler.py (처리)
# v1.0.0

import socket
import json
from parser import parse_msg
from handler import handle_msg

HOST = "0.0.0.0"
PORT = 5000

def udp_start():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    print(f"[UDP] 서버 온라인 (포트 {PORT})")

    while True:
        data, addr = sock.recvfrom(65535)
        raw = json.loads(data.decode("utf-8"))
        print(f"[UDP] {addr[0]}:{addr[1]}")

        parsed = parse_msg(raw)
        handle_msg(parsed)
