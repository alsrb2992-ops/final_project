# Pi 없이 PC에서 테스트하기 위한 모의 송신기
# LiDAR 데이터  -> UDP (포트 5000)
# 카메라 데이터 -> TCP (포트 5001)
# 마이크 데이터 -> TCP (포트 5002)
# v1.0.0

import socket
import json
import time
import random
import threading

sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_cam = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_cam.connect(("127.0.0.1", 5001))
sock_voice = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_voice.connect(("127.0.0.1", 5002))

print("--------------------------------------------------")
print("                 모의 송신기 시작                  ")
print("--------------------------------------------------")

def send_lidar():
    while True:
        data = {
            "type": "lidar_raw",
            "timestamp": time.time(),
            "scan": [
                {"angle": round(i * 0.43 * 2), "dist_cm": random.randint(50, 300)} for i in range(10)
            ]
        }
        sock_lidar.sendto(json.dumps(data).encode("utf-8"), ("127.0.0.1", 5000))
        print(f"[LiDAR] 스캔={data}")
        time.sleep(1)

def send_cam():
    while True:
        detected = random.random() > 0.5
        data = {
            "type": "cam",
            "timestamp": time.time(),
            "person_detected": detected,
            "confidence": round(random.uniform(0.7, 1.0), 2) if detected else 0.0
        }
        sock_cam.sendall((json.dumps(data) + "\n").encode("utf-8"))
        print(f"[CAM] 사람={'감지' if detected else '없음'} 신뢰도={data['confidence']:.0%}")
        time.sleep(2)

def send_voice():
    keywords = ["살려주세요", "살려줘", "도와주세요"]
    while True:
        triggered = random.random() > 0.7
        data = {
            "type": "voice",
            "timestamp": time.time(),
            "triggered": triggered,
            "keyword": random.choice(keywords) if triggered else None,
            "dirDeg": round(random.uniform(0, 360), 1)
        }
        sock_voice.sendall((json.dumps(data) + "\n").encode("utf-8"))
        print(f"[MIC] 트리거={'됨' if triggered else '없음'} 키워드={data['keyword']}")
        time.sleep(3)

threads = [
    threading.Thread(target=send_lidar),
    threading.Thread(target=send_cam),
    threading.Thread(target=send_voice)
]

for t in threads:
    t.daemon = True
    t.start()

threads[0].join()
