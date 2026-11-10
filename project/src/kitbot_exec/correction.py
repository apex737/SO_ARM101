"""블록 위 접근 위치 보정 (S02-7).

pick --no-place에서 블록 위에 멈춘 뒤, 사용자가 리더암으로 팔로워를 '정확한 위치·각도'로 옮기면
그때 관절값으로 계산한 그리퍼 끝 위치·방향과, 카메라로 계산한 목표의 차이(dx, dy, 각도)를 한 건으로 모은다.
다음 실행부터 pick이 이 차이를 카메라 결과에 더해 쓴다.

- 1~5건: 평균 차이를 모든 위치에 똑같이 더한다(카메라·모델의 일정한 밀림).
- 6건 이상: 위치마다 다른 밀림을 x, y의 1차식으로 맞춘다(dx = a + b·x + c·y, dy도 같은 꼴).
- 각도: 평균 차이를 더한다(90° 주기).
기록: var/approach_corrections.json (지우면 보정 없음)
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

FILE = Path(__file__).resolve().parents[1] / "var" / "approach_corrections.json"
AFFINE_MIN = 6


def wrap90(a):
    """각도를 [-45, 45)로 (정사각형 블록은 90° 돌려도 같다)."""
    return (np.asarray(a, float) + 45.0) % 90.0 - 45.0


def load_samples(path: Path = FILE) -> list[dict]:
    return json.loads(path.read_text()) if path.exists() else []


def add_sample(raw_xyyaw, measured_xyyaw, extra: dict | None = None, path: Path = FILE) -> list[dict]:
    """raw = 카메라·보정만으로 계산한 블록 (x, y, yaw), measured = 리더암으로 맞춘 실제 (x, y, yaw)."""
    samples = load_samples(path)
    rx, ry, ryaw = (float(v) for v in raw_xyyaw)
    mx, my, myaw = (float(v) for v in measured_xyyaw)
    s = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "raw": [round(rx, 2), round(ry, 2), round(ryaw, 2)],
         "measured": [round(mx, 2), round(my, 2), round(myaw, 2)],
         "d": [round(mx - rx, 2), round(my - ry, 2), round(float(wrap90(myaw - ryaw)), 2)]}
    s.update(extra or {})
    samples.append(s)
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(samples, ensure_ascii=False, indent=1))
    return samples


class Correction:
    def __init__(self, samples: list[dict]):
        self.n = len(samples)
        self.mode = "none"
        self.mean = np.zeros(3)
        self.coef = None  # (3, 2): [1, x, y] → (dx, dy)
        if not samples:
            return
        raw = np.array([s["raw"] for s in samples], float)
        d = np.array([s["d"] for s in samples], float)
        self.mean = np.r_[d[:, :2].mean(axis=0), float(np.mean(wrap90(d[:, 2])))]
        self.spread_mm = float(np.max(np.linalg.norm(d[:, :2] - self.mean[:2], axis=1)))
        self.mode = "mean"
        if self.n >= AFFINE_MIN:
            A = np.c_[np.ones(self.n), raw[:, :2]]
            self.coef, *_ = np.linalg.lstsq(A, d[:, :2], rcond=None)
            self.mode = "affine"

    def offset(self, x, y) -> tuple[float, float, float]:
        if self.mode == "affine":
            dx, dy = np.array([1.0, x, y]) @ self.coef
            return float(dx), float(dy), float(self.mean[2])
        return float(self.mean[0]), float(self.mean[1]), float(self.mean[2])

    def apply(self, x, y, yaw) -> tuple[float, float, float]:
        dx, dy, dyaw = self.offset(x, y)
        return x + dx, y + dy, float(wrap90(yaw + dyaw))

    def summary(self) -> str:
        if self.mode == "none":
            return "접근 보정 없음 (기록 0건)"
        how = "위치별 1차식" if self.mode == "affine" else "평균을 모든 위치에 더함"
        return (f"접근 보정 {self.n}건, {how}: 평균 dx {self.mean[0]:+.1f} dy {self.mean[1]:+.1f} mm, "
                f"각도 {self.mean[2]:+.1f}°, 건마다 흩어짐 최대 {self.spread_mm:.1f}mm")
