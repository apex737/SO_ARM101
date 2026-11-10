"""손목 카메라 사진으로 접근 오차를 잰다 (줄자 없이). 팔·카메라를 직접 다루지 않는 계산만 모았다.

생각의 바탕
- 손목 카메라는 그리퍼에 고정되어 있다. 그래서 "블록이 두 부리 가운데 바로 아래에 있을 때" 블록이 사진의 어디에,
  어떤 각도·크기로 보이는지 한 번 기록해 두면(기준 = WristRef), 이후에는 사진만 보고 블록이 부리 가운데에서
  몇 mm·몇 도 벗어났는지, 접근 높이가 기준보다 몇 mm 다른지 계산할 수 있다.
- mm/픽셀 비율과 방향은 팔을 그리퍼 좌표 방향으로 ±10mm 움직이며 블록이 사진에서 얼마나 움직이는지로 구한다(J).
- 높이는 블록 크기로 잰다. 핀홀 카메라에서 크기는 거리에 반비례하므로, 기준 높이와 20mm 위에서의 크기 비율로
  카메라-블록 거리(d_ref)를 구할 수 있다.

부호 약속
- tool 좌표 = 그리퍼 회전 행렬 R의 x, y 축을 테이블 평면에 투영한 2축. 로봇 좌표 변환은 R[:2, :2] @ v.
- J: 블록이 가만히 있고 그리퍼가 tool 좌표로 Δt [mm] 움직이면 블록 픽셀이 Δpx = J @ Δt 만큼 움직인다.
- 블록이 부리 가운데에서 tool 좌표로 e [mm] 벗어나 있으면 Δpx = −J @ e (그리퍼가 −e 움직인 것과 같다).
- k_yaw: 블록이 가만히 있고 그리퍼가 +θ 돌면 블록 사진 각도가 k·θ 변한다(보통 k ≈ −1 또는 +1).
  블록이 그리퍼 대비 +φ 돌아가 있으면 사진 각도는 −k·φ 변한다.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from .correction import wrap90
from .vision import BlockFilter, detect_blocks

REF_FILE = Path(__file__).resolve().parents[1] / "var" / "wrist_ref.json"

# 손목 카메라는 블록이 가깝고 크게 보이며 부리에 일부 가려질 수 있어 위 카메라보다 느슨하게 거른다 (추정값, 현장 조정)
WRIST_FILTER = BlockFilter(min_area_px=1500.0, max_area_px=200000.0, max_aspect=2.0, min_fill=0.5)


@dataclass
class WristObs:
    center_px: tuple[float, float]
    angle_deg: float          # 사진 기준, [-45, 45)
    size_px: float            # 회전 사각형 두 변의 평균
    n: int                    # 블록을 찾은 사진 수

    def as_dict(self) -> dict:
        return {"center_px": [round(self.center_px[0], 2), round(self.center_px[1], 2)],
                "angle_deg": round(self.angle_deg, 2), "size_px": round(self.size_px, 2), "n": self.n}


@dataclass
class WristRef:
    c_ref: tuple[float, float]          # 블록이 부리 가운데 바로 아래(접근 높이)일 때 블록 중심 픽셀
    angle_ref: float                    # 그때 블록 사진 각도
    size_ref: float                     # 그때 블록 크기 [px]
    J: list                             # 2×2, px per tool-mm
    k_yaw: float                        # 사진 각도 변화 / 그리퍼 각도 변화
    d_ref_mm: float                     # 접근 높이에서 손목 카메라 ~ 블록 윗면 거리 (블록 크기로 계산)
    meta: dict = field(default_factory=dict)

    def save(self, path: Path = REF_FILE) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), ensure_ascii=False, indent=1))

    @classmethod
    def load(cls, path: Path = REF_FILE) -> "WristRef":
        d = json.loads(Path(path).read_text())
        return cls(tuple(d["c_ref"]), d["angle_ref"], d["size_ref"], d["J"], d["k_yaw"], d["d_ref_mm"], d.get("meta", {}))


# ---------- 사진 → 관측 ----------
def mean_square_angle(angles_deg) -> float:
    """정사각형 각도(90° 주기)의 원형 평균, [-45, 45)."""
    a = np.deg2rad(4.0 * np.asarray(angles_deg, float))
    return float(wrap90(np.rad2deg(np.arctan2(np.sin(a).mean(), np.cos(a).mean())) / 4.0))


def pick_center_block(blocks, shape):
    """여러 덩어리가 보이면 사진 가운데에 가장 가까운 것 (목표 블록은 부리 아래 = 사진 가운데 근처)."""
    if not blocks:
        return None
    h, w = shape[:2]
    return min(blocks, key=lambda b: (b.center_px[0] - w / 2) ** 2 + (b.center_px[1] - h / 2) ** 2)


def observe(frames, color: str, flt: BlockFilter = WRIST_FILTER, min_found: int = 3) -> WristObs | None:
    """사진 여러 장에서 블록을 찾아 중앙값(위치·크기)과 원형 평균(각도)을 낸다. 못 찾으면 None."""
    cs, angs, sizes = [], [], []
    for f in frames:
        if f is None:
            continue
        b = pick_center_block(detect_blocks(f, color, flt), f.shape)
        if b is None:
            continue
        cs.append(b.center_px)
        angs.append(b.angle_px_deg)
        sizes.append(float(np.mean(b.size_px)))
    if len(cs) < min_found:
        return None
    c = np.median(np.asarray(cs, float), axis=0)
    return WristObs((float(c[0]), float(c[1])), mean_square_angle(angs), float(np.median(sizes)), len(cs))


# ---------- 기준 만들기 ----------
def fit_J(d_tool_mm, d_px) -> tuple[np.ndarray, float]:
    """그리퍼 이동량 Δt (N×2, tool mm) 와 블록 픽셀 이동량 Δpx (N×2) → J (2×2), 맞춤 잔차 RMS [px]."""
    T = np.asarray(d_tool_mm, float)
    P = np.asarray(d_px, float)
    JT, *_ = np.linalg.lstsq(T, P, rcond=None)
    rms = float(np.sqrt(np.mean(np.sum((T @ JT - P) ** 2, axis=1))))
    return JT.T, rms


def d_from_two_heights(size_low_px: float, size_high_px: float, dz_mm: float) -> float:
    """기준 높이(크기 size_low)와 dz 위(크기 size_high)에서 블록 크기 → 기준 높이의 카메라-블록 거리 [mm].
    크기 ∝ 1/거리 → size_low/size_high = (d + dz)/d."""
    ratio = size_low_px / size_high_px
    if ratio <= 1.0:
        raise ValueError(f"위로 올렸는데 블록이 작아지지 않음 (비율 {ratio:.3f})")
    return float(dz_mm / (ratio - 1.0))


def yaw_gain(angle_before: float, angle_after: float, tool_yaw_before: float, tool_yaw_after: float) -> float:
    d_tool = float(wrap90(tool_yaw_after - tool_yaw_before))
    if abs(d_tool) < 1.0:
        raise ValueError("그리퍼 각도가 거의 바뀌지 않음")
    return float(wrap90(angle_after - angle_before)) / d_tool


# ---------- 시행 측정 ----------
def offset_tool_mm(obs: WristObs, ref: WristRef) -> np.ndarray:
    """블록이 부리 가운데에서 tool 좌표로 몇 mm 벗어났나 (e = −J⁻¹ Δpx)."""
    dpx = np.asarray(obs.center_px, float) - np.asarray(ref.c_ref, float)
    return -np.linalg.solve(np.asarray(ref.J, float), dpx)


def offset_yaw_deg(obs: WristObs, ref: WristRef) -> float:
    """블록이 그리퍼 대비 몇 도 돌아가 있나 (φ = −Δ사진각도 / k)."""
    return float(wrap90(-float(wrap90(obs.angle_deg - ref.angle_ref)) / ref.k_yaw))


def height_offset_mm(obs: WristObs, ref: WristRef) -> float:
    """실제 접근 높이 − 기준 접근 높이 [mm]. + = 기준보다 높이 떠 있음 (블록이 작게 보임)."""
    return float(ref.d_ref_mm * (ref.size_ref / obs.size_px - 1.0))


def tool_to_robot(R, v_tool) -> np.ndarray:
    return np.asarray(R, float)[:2, :2] @ np.asarray(v_tool, float)


def robot_to_tool(R, v_robot) -> np.ndarray:
    return np.linalg.solve(np.asarray(R, float)[:2, :2], np.asarray(v_robot, float))


# ---------- 결과 요약 ----------
def group_of(name: str) -> str:
    """'middle_0_r2' → 'middle_0' (같은 위치 반복을 묶는다)."""
    head, sep, tail = name.rpartition("_r")
    return head if sep and tail.isdigit() else name


def summarize(trials: list[dict], tol_mm: float = 10.0, tol_yaw: float = 10.0) -> dict:
    """approach 시행 기록(JSONL 줄들) → 판정에 쓰는 숫자. 계획 실패·미검출도 분모에 넣는다."""
    out: dict = {"n_trials": len(trials)}
    out["by_result"] = {}
    for t in trials:
        out["by_result"][t["result"]] = out["by_result"].get(t["result"], 0) + 1
    meas = [t for t in trials if t.get("wrist") and t["wrist"].get("err_robot_mm") is not None]
    moved = [t for t in trials if t.get("diag")]
    out["n_measured"] = len(meas)
    out["wrist_visible"] = f"{sum(1 for t in moved if t.get('wrist'))}/{len(moved)}"
    if moved:
        out["settle_timeout"] = f"{sum(1 for t in moved if t['diag']['settle_timeout'])}/{len(moved)}"
        dq = np.array([t["diag"]["target_minus_read_deg"] for t in moved], float)
        out["target_minus_read_deg_mean"] = np.round(dq.mean(axis=0), 2).tolist()
        out["target_minus_read_deg_maxabs"] = np.round(np.abs(dq).max(axis=0), 2).tolist()
    if not meas:
        return out
    e = np.array([t["wrist"]["err_robot_mm"] for t in meas], float)
    yaw = np.array([t["wrist"]["err_yaw_deg"] for t in meas], float)
    dz = np.array([t["wrist"]["height_offset_mm"] for t in meas], float)
    mag = np.linalg.norm(e, axis=1)
    ok = (mag <= tol_mm) & (np.abs(yaw) <= tol_yaw)
    out["pass"] = f"{int(ok.sum())}/{len(trials)}"           # 분모 = 전체 시행 (미측정은 불통과)
    out["pass_rate"] = round(float(ok.sum()) / max(len(trials), 1), 3)
    out["err_mm_mean_xy"] = np.round(e.mean(axis=0), 1).tolist()
    out["err_mm_abs"] = {"mean": round(float(mag.mean()), 1), "max": round(float(mag.max()), 1)}
    out["err_yaw_deg"] = {"mean": round(float(yaw.mean()), 1), "maxabs": round(float(np.abs(yaw).max()), 1)}
    out["height_offset_mm"] = {"mean": round(float(dz.mean()), 1), "min": round(float(dz.min()), 1),
                               "max": round(float(dz.max()), 1)}
    # 같은 위치 반복: 평균에서 가장 먼 시행까지 거리
    groups: dict[str, list[int]] = {}
    for i, t in enumerate(meas):
        groups.setdefault(group_of(t["name"]), []).append(i)
    rep = {}
    for g, idx in groups.items():
        if len(idx) < 2:
            continue
        eg = e[idx]
        rep[g] = {"n": len(idx), "mean_mm": np.round(eg.mean(axis=0), 1).tolist(),
                  "spread_mm": round(float(np.max(np.linalg.norm(eg - eg.mean(axis=0), axis=1))), 1),
                  "spread_yaw_deg": round(float(np.max(np.abs(yaw[idx] - yaw[idx].mean()))), 1),
                  "spread_height_mm": round(float(dz[idx].max() - dz[idx].min()), 1)}
    out["repeat"] = rep
    # 받침대에서 멀수록 바깥으로 밀리나: 바깥 방향 성분 = a + b × 거리
    tgt = np.array([t["target_xy"] for t in meas], float)
    r = np.linalg.norm(tgt, axis=1)
    if len(meas) >= 4 and np.ptp(r) > 20.0:
        radial = np.sum(e * (tgt / r[:, None]), axis=1)
        b, a = np.polyfit(r, radial, 1)
        out["radial_fit"] = {"a_mm": round(float(a), 1), "b_mm_per_100mm": round(float(b) * 100, 2),
                             "r_range_mm": [round(float(r.min())), round(float(r.max()))]}
    return out


def table_summary(points: list[dict]) -> dict:
    """테이블 짚기 기록 → 모델 높이가 일정한가(상수 차이) 아니면 거리에 따라 변하나(모델·0점 오차)."""
    xyz = np.array([p["tip_mm"] for p in points], float)
    r = np.linalg.norm(xyz[:, :2], axis=1)
    z = xyz[:, 2]
    out = {"n": len(points), "z_mean": round(float(z.mean()), 1), "z_range": round(float(np.ptp(z)), 1)}
    if len(points) >= 3 and np.ptp(r) > 20.0:
        b, a = np.polyfit(r, z, 1)
        out["z_vs_distance_mm_per_100mm"] = round(float(b) * 100, 2)
    if all("calib_table_z" in p for p in points):
        dz = z - np.array([p["calib_table_z"] for p in points], float)
        out["model_minus_calib_table_z"] = {"mean": round(float(dz.mean()), 1), "range": round(float(np.ptp(dz)), 1)}
    return out
