# 수신 JSON 데이터를 모델 구조로 파싱
# v1.0.0

from models import scanPoint, lidarData, camData, voiceData

# LiDAR
def parse_lidar(data: dict) -> lidarData:
    return lidarData(
        timestamp   = data["timestamp"],
        scan        = [scanPoint(angle=p["angle"], dist_cm=p["dist_cm"]) for p in data["scan"]]
    )

# 카메라
def parse_cam(data: dict) -> camData:
    return camData(
        timestamp       = data["timestamp"],
        person_detected = data["person_detected"],
        confidence      = data["confidence"]
    )

# 마이크
def parse_voice(data: dict) -> voiceData:
    return voiceData(
        timestamp = data["timestamp"],
        triggered = data["triggered"],
        keyword   = data.get("keyword"),
        dirDeg    = data["dirDeg"]
    )

def parse_msg(data: dict):
    type_ = data["type"]
    if type_ == "lidar_raw": return parse_lidar(data)
    elif type_ == "cam":     return parse_cam(data)
    elif type_ == "voice":   return parse_voice(data)
    else:
        print(f"[WARN] 알 수 없는 타입: {type_}")
        return None
