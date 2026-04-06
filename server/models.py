# 수신 데이터 구조 정의
# scanPoint: LiDAR 스캔 포인트 (각도, 거리)
# lidarData: LiDAR 전체 데이터
# camData:   카메라 데이터
# voiceData: 마이크 데이터
# v1.0.0

from dataclasses import dataclass
from typing import Optional

@dataclass
class scanPoint:
    angle: float
    dist_cm: float

@dataclass
class lidarData:
    timestamp: float
    scan: list[scanPoint]

@dataclass
class camData:
    timestamp: float
    person_detected: bool
    confidence: float

@dataclass
class voiceData:
    timestamp: float
    triggered: bool
    keyword: Optional[str]
    dirDeg: float
