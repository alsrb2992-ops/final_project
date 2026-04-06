# v1.0.0

import threading
from udp_receiver import udp_start
from tcp_receiver import tcp_start

def main():
    print("==================================================")
    print("              구조 RC카 수신 서버 시작              ")
    print("==================================================")

    thread_lidar = threading.Thread(target=udp_start)
    thread_cam = threading.Thread(target=tcp_start, args=(5001,))
    thread_voice = threading.Thread(target=tcp_start, args=(5002,))

    for t in [thread_lidar, thread_cam, thread_voice]:
        t.daemon = True
        t.start()

    print("UDP 서버 가동 (LiDAR/엔코더)")
    print("TCP 서버 가동 (카메라/마이크)")

    thread_lidar.join()

if __name__ == "__main__":
    main()
