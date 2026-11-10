"""위 카메라 사진에서 블록 찾기.

흐름: BGR 사진 → HSV → 색 범위 마스크 → 잡음 제거 → 윤곽선 → 블록처럼 보이는 것만 거르기 → 중심·각도.
practice/00-imgProcess/RGBD2/의 연습 코드와 같은 흐름이고, 다음을 더했다.
- 잡음 제거(열기·닫기), 블록 크기·모양 거르기(면적, 가로세로 비율, 채움 비율)
- 중심 = 윤곽선 무게중심(moments). 바깥 사각형 가운데보다 덜 흔들린다.
- 각도 = 회전된 최소 사각형(minAreaRect). 정육면체라 90°마다 같으므로 [-45°, 45°)로 맞춘다.
각도는 화면 기준이다. 카메라가 비스듬하면 화면 각도와 테이블 위 각도가 다르므로,
로봇 기준 각도는 calib.py에서 네 꼭짓점을 변환해 다시 구한다.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

# OpenCV HSV: H 0~179 (각도 ÷ 2), S·V 0~255.
# 연습 코드(05.GPT코드.HSV 다중색영역.py)의 값에서 시작한다. 웹캠·조명에 맞춰 다시 조정해야 한다.
HSV_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "green": [((35, 100, 70), (85, 255, 255))],
    "blue": [((100, 120, 70), (130, 255, 255))],
    "purple": [((122, 40, 70), (155, 255, 255))],  # 2026-11-10 위 카메라 실측: 보라 블록 H 125~135, S 60~100 (밝은 곳에서 채도가 낮음)
}


@dataclass(frozen=True)
class BlockFilter:
    """블록처럼 보이는 덩어리만 남기는 기준. 면적은 카메라 높이·해상도에 따라 바뀌므로 현장에서 맞춘다."""

    min_area_px: float = 300.0     # 이보다 작으면 잡음
    max_area_px: float = 20000.0   # 이보다 크면 블록이 아님(배경, 큰 물건)
    max_aspect: float = 1.5        # 회전 사각형의 긴 변 ÷ 짧은 변. 위에서 본 정육면체 윗면은 1에 가깝다
    min_fill: float = 0.70         # 윤곽선 면적 ÷ 회전 사각형 면적. 사각형이 꽉 차 있어야 블록


@dataclass(frozen=True)
class Block:
    center_px: tuple[float, float]      # 무게중심 (u, v) 픽셀. u = 오른쪽, v = 아래쪽
    angle_px_deg: float                 # 화면 기준 각도, [-45, 45)
    size_px: tuple[float, float]        # 회전 사각형의 두 변 길이
    area_px: float                      # 윤곽선 면적
    fill: float                         # 채움 비율
    corners_px: np.ndarray              # 회전 사각형 꼭짓점 4개 (4×2)
    contour: np.ndarray


def color_mask(bgr: np.ndarray, color: str, ranges=None) -> np.ndarray:
    """색 범위에 드는 픽셀 = 255, 나머지 = 0인 마스크. 빨강처럼 H가 0과 179에 걸친 색은 범위 두 개를 합친다."""
    ranges = (ranges or HSV_RANGES)[color]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return mask


def clean_mask(mask: np.ndarray, open_k: int = 3, close_k: int = 5) -> np.ndarray:
    """열기 = 작은 점 지우기, 닫기 = 블록 안 작은 구멍 메우기."""
    if open_k > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_k, open_k), np.uint8))
    if close_k > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((close_k, close_k), np.uint8))
    return mask


def normalize_square_angle(angle_deg: float) -> float:
    """정사각형은 90°마다 같은 모양 → 각도를 [-45, 45)로 맞춘다."""
    return (angle_deg + 45.0) % 90.0 - 45.0


def blocks_in_mask(mask: np.ndarray, flt: BlockFilter = BlockFilter()) -> list[Block]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found: list[Block] = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (flt.min_area_px <= area <= flt.max_area_px):
            continue
        (cx_r, cy_r), (w, h), ang = cv2.minAreaRect(c)
        if min(w, h) <= 0:
            continue
        if max(w, h) / min(w, h) > flt.max_aspect:
            continue
        fill = area / (w * h)
        if fill < flt.min_fill:
            continue
        m = cv2.moments(c)
        center = (m["m10"] / m["m00"], m["m01"] / m["m00"]) if m["m00"] else (cx_r, cy_r)
        corners = cv2.boxPoints(((cx_r, cy_r), (w, h), ang)).astype(np.float64)
        found.append(Block(center, normalize_square_angle(ang), (w, h), area, fill, corners, c))
    found.sort(key=lambda b: b.area_px, reverse=True)
    return found


def detect_blocks(bgr: np.ndarray, color: str, flt: BlockFilter = BlockFilter(), ranges=None) -> list[Block]:
    """사진 한 장에서 지정한 색의 블록을 모두 찾는다. 큰 것부터 정렬."""
    return blocks_in_mask(clean_mask(color_mask(bgr, color, ranges)), flt)


def draw_blocks(bgr: np.ndarray, blocks: list[Block], label: str = "") -> np.ndarray:
    """확인용 그림: 회전 사각형(파랑), 중심(빨강 +), 각도 글자."""
    out = bgr.copy()
    for i, b in enumerate(blocks):
        cv2.polylines(out, [b.corners_px.astype(np.int32)], True, (255, 0, 0), 2)
        u, v = int(round(b.center_px[0])), int(round(b.center_px[1]))
        cv2.drawMarker(out, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
        cv2.putText(out, f"{label}{i} ({u},{v}) {b.angle_px_deg:+.1f}deg", (u + 8, v - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    return out
