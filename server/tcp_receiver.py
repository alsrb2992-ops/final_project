# TCP 소켓 생성 및 데이터 수신 (카메라, 마이크)
# 수신 데이터 -> parser.py (파싱) -> handler.py (처리)
# 카메라: 포트 5001
# 마이크: 포트 5002
# v1.0.0

import socket
import json
from parser import parse_msg
from handler import handle_msg

HOST = "0.0.0.0"
CAM_PORT = 5001
VOICE_PORT = 5002

def tcp_handle_client(conn, addr):
    print(f"[TCP] 연결됨 {addr[0]}:{addr[1]}")
    with conn:
        buff = ""
        while True:
            data = conn.recv(4096).decode("utf-8")
            if not data:
                print(f"[TCP] 연결 끊김 {addr[0]}:{addr[1]}")
                break

            buff = buff + data
            while "\n" in buff:
                line, buff = buff.split("\n", 1)
                raw = json.loads(line)
                parsed = parse_msg(raw)
                handle_msg(parsed)

def tcp_start(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, port))
    sock.listen()

    print(f"[TCP] 서버 온라인 (포트 {port})")

    while True:
        conn, addr = sock.accept()
        tcp_handle_client(conn, addr)
