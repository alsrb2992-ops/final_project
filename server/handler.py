# 파싱된 데이터 처리 및 출력
# 현재는 수신 데이터 출력만 담당, 추후 판단 로직 추가 예정
# v1.0.0

from models import lidarData, camData, voiceData

def handle_msg(data):
    if data is None: return

    # LiDAR
    if isinstance(data, lidarData):
        print(f"-------------------- LiDAR --------------------")
        print(f"스캔: {len(data.scan)}개 포인트")

    # 카메라
    elif isinstance(data, camData):
        detected = "감지됨" if data.person_detected else "없음"
        print(f"--------------------- CAM ---------------------")
        print(f"사람: {detected} (신뢰도: {data.confidence:.0%})")

    # 마이크
    elif isinstance(data, voiceData):
        triggered = "트리거됨" if data.triggered else "대기 중"
        print(f"--------------------- MIC ---------------------")
        print(f"상태: {triggered}")
        print(f"키워드: {data.keyword}")
        print(f"방향: {data.dirDeg}°")
