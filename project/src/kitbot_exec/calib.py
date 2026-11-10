"""화면 픽셀 → 로봇 좌표(mm) 보정.

원리: 블록은 테이블 위 한 층에만 놓인다. 그러면 블록 윗면은 모두 같은 높이의 평면 위에 있고,
그 평면 위의 점과 화면 픽셀은 3×3 변환 하나(평면 대응 변환, homography)로 1:1 대응한다.
카메라가 비스듬해도 같다(문서 스캔 앱이 네 모서리로 종이를 펴는 것과 같은 원리).

보정점 모으는 법 (S02):
  블록을 작업 영역 여러 곳(9곳 권장)에 차례로 놓고,
  ① 위 카메라로 블록 중심 픽셀 (u, v)
  ② 리더암으로 팔로워 그리퍼 끝을 블록 윗면 가운데에 살짝 대고, 그때 관절값으로 계산한 그리퍼 끝 위치 (x, y, z)
  를 한 쌍으로 기록한다.
  블록 윗면에서 짝을 지으므로 블록 높이 때문에 생기는 위치 밀림(시차)이 처음부터 없다.
  로봇 좌표는 기구학 모델(kinematics.py)로 계산한 값이다. 나중에 팔을 보낼 때도 같은 모델의 역기구학을 쓰므로,
  모델의 작은 오차(관절 0점 차이 등)는 보정 영역 안에서 상당 부분 서로 상쇄된다(가정, S02에서 측정으로 확인).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from .vision import Block, normalize_square_angle


@dataclass
class PixelToRobot:
    H: np.ndarray                      # 3×3, 픽셀 (u, v, 1) → 로봇 (x, y, 1) [mm]
    z_plane: np.ndarray                # (a, b, c): 블록 윗면 높이 z = a·x + b·y + c [mm]
    pairs_px: np.ndarray = field(repr=False, default=None)
    pairs_mm: np.ndarray = field(repr=False, default=None)

    # ---------- 적용 ----------
    def to_xy(self, px) -> np.ndarray:
        """픽셀 (N×2 또는 2,) → 로봇 x, y [mm]."""
        p = np.asarray(px, np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, self.H).reshape(-1, 2)

    def z_at(self, xy) -> np.ndarray:
        xy = np.asarray(xy, np.float64).reshape(-1, 2)
        a, b, c = self.z_plane
        return a * xy[:, 0] + b * xy[:, 1] + c

    def block_pose(self, block: Block) -> tuple[float, float, float, float]:
        """블록 → (x, y, 윗면 z [mm], 로봇 기준 각도 [deg, -45~45)).
        각도는 화면 각도를 쓰지 않고, 회전 사각형 꼭짓점 4개를 로봇 평면으로 옮긴 뒤 변의 방향으로 다시 구한다
        (비스듬한 카메라에서는 화면 각도가 왜곡되기 때문)."""
        x, y = self.to_xy(block.center_px)[0]
        z = float(self.z_at([[x, y]])[0])
        return float(x), float(y), z, yaw_from_corners(self.to_xy(block.corners_px))

    # ---------- 품질 ----------
    def residuals_mm(self, px=None, mm=None) -> np.ndarray:
        """각 점의 수평 오차 [mm]. 인자를 주지 않으면 보정에 쓴 점으로 계산(실제보다 작게 나온다)."""
        px = self.pairs_px if px is None else np.asarray(px, np.float64)
        mm = self.pairs_mm if mm is None else np.asarray(mm, np.float64)
        return np.linalg.norm(self.to_xy(px) - mm[:, :2], axis=1)

    # ---------- 저장 ----------
    def save(self, path: str | Path) -> None:
        data = {"H": self.H.tolist(), "z_plane": self.z_plane.tolist(),
                "pairs_px": None if self.pairs_px is None else self.pairs_px.tolist(),
                "pairs_mm": None if self.pairs_mm is None else self.pairs_mm.tolist()}
        Path(path).write_text(json.dumps(data, indent=2))

    @classmethod
    def load(cls, path: str | Path) -> "PixelToRobot":
        d = json.loads(Path(path).read_text())
        arr = lambda k: None if d.get(k) is None else np.asarray(d[k], np.float64)
        return cls(np.asarray(d["H"]), np.asarray(d["z_plane"]), arr("pairs_px"), arr("pairs_mm"))


def fit_pixel_to_robot(pixels, robot_xyz_mm) -> PixelToRobot:
    """보정점 쌍으로 변환을 구한다. 최소 4쌍, 오차를 줄이려면 9쌍 이상을 작업 영역 전체에 고르게."""
    px = np.asarray(pixels, np.float64).reshape(-1, 2)
    mm = np.asarray(robot_xyz_mm, np.float64).reshape(-1, 3)
    if len(px) != len(mm) or len(px) < 4:
        raise ValueError(f"보정점이 4쌍 이상 필요합니다 (받은 수: {len(px)}, {len(mm)})")
    H, _ = cv2.findHomography(px, mm[:, :2], 0)  # 0 = 모든 점으로 최소제곱
    if H is None:
        raise ValueError("변환을 구하지 못했습니다. 점이 한 줄로 늘어서 있지 않은지 확인하세요")
    # 블록 윗면 높이: 테이블이 살짝 기울었을 수 있어 평면으로 맞춘다. 점이 적으면 평균값만.
    if len(mm) >= 3:
        A = np.c_[mm[:, 0], mm[:, 1], np.ones(len(mm))]
        z_plane, *_ = np.linalg.lstsq(A, mm[:, 2], rcond=None)
    else:
        z_plane = np.array([0.0, 0.0, mm[:, 2].mean()])
    return PixelToRobot(H, z_plane, px, mm)


def yaw_from_corners(corners_xy) -> float:
    """사각형 꼭짓점 4개(순서대로)의 변 방향으로 각도를 구한다. 네 변을 90° 주기로 평균."""
    c = np.asarray(corners_xy, np.float64).reshape(4, 2)
    edges = np.roll(c, -1, axis=0) - c
    ang = np.arctan2(edges[:, 1], edges[:, 0])
    # 90° 주기 평균: 각도 × 4 를 원 위에서 평균한 뒤 ÷ 4
    m = np.arctan2(np.sin(4 * ang).mean(), np.cos(4 * ang).mean()) / 4
    return normalize_square_angle(float(np.rad2deg(m)))
