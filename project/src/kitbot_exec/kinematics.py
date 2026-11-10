"""SO-101 팔 기구학: 관절값(도) ↔ 그리퍼 끝 위치(mm).

- 모델: project/src/assets/so101/so101_new_calib.urdf (TheRobotStudio/SO-ARM100, 2026-10-06 커밋 a758567).
  "new calib" = LeRobot의 새 캘리브레이션처럼 각 관절의 0°가 가동 범위 가운데인 모델.
- 계산 라이브러리: placo (conda lerobot 환경에 설치되어 있음). .venv에는 없다.
- 그리퍼 끝 = URDF의 gripper_frame_link. 이 점의 z축이 손가락이 향하는 방향이다(관절 모두 0°일 때 앞쪽).

확인하지 않은 가정 (S02에서 실물로 확인):
1. LeRobot이 읽는 관절 각도(도)와 이 모델의 관절 각도가 0점·방향이 같다.
   캘리브레이션 때 '가운데 자세'를 손으로 잡았으므로 관절마다 몇 도씩 다를 수 있다.
2. gripper_frame_link 위치가 실제 손가락 끝 가운데와 같다.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
DEFAULT_URDF = Path(__file__).resolve().parents[1] / "assets" / "so101" / "so101_new_calib.urdf"
TOOL_FRAME = "gripper_frame_link"


@dataclass(frozen=True)
class IKResult:
    ok: bool
    q_deg: np.ndarray | None      # 관절 5개 [deg], ARM_JOINTS 순서
    pos_err_mm: float             # 목표 위치와 계산된 그리퍼 끝 사이 거리
    tilt_deg: float               # 그리퍼 방향이 수직 아래에서 기운 각도
    yaw_err_deg: float            # 손목 회전 후 블록 각도와의 차이 (90° 주기)
    reason: str = ""


def _wrap90(a: float) -> float:
    return (a + 45.0) % 90.0 - 45.0


class SO101Kinematics:
    def __init__(self, urdf_path: str | Path = DEFAULT_URDF, joint_margin_deg: float = 3.0):
        import placo  # conda lerobot 환경에서만 있음

        self._placo = placo
        self.robot = placo.RobotWrapper(str(urdf_path))
        self.margin = np.deg2rad(joint_margin_deg)
        self._urdf_limits = _read_urdf_limits(Path(urdf_path))
        # 계산기에게도 여유를 둔 한계를 준다. 그래야 한계에 딱 붙은 답 대신 여유 안의 답(필요하면 약간 기운 자세)을 찾는다.
        for j, (lo, hi) in self._urdf_limits.items():
            self.robot.set_joint_limits(j, lo + self.margin, hi - self.margin)
        self.robot.set_joint("gripper", 0.0)

    # ---------- 정기구학 ----------
    def fk(self, q_deg) -> tuple[np.ndarray, np.ndarray]:
        """관절 5개 [deg] → (그리퍼 끝 위치 [mm], 회전 행렬 3×3)."""
        self._set_q(q_deg)
        T = self.robot.get_T_world_frame(TOOL_FRAME)
        return T[:3, 3] * 1000.0, T[:3, :3]

    # ---------- 역기구학: 위에서 아래로 집는 자세 ----------
    def ik_top_down(self, target_mm, yaw_deg: float = 0.0, q_init_deg=None, *,
                    z_min_mm: float = 5.0, tol_mm: float = 1.0, tol_tilt_deg: float = 2.0, max_tilt_deg: float | None = None,
                    max_iter: int = 300, roll_pref_deg: float | None = None) -> IKResult:
        """그리퍼 끝을 target_mm (x, y, z)에 두고 손가락이 수직 아래를 향하며,
        손목 회전으로 그리퍼를 블록 각도 yaw_deg(로봇 기준, 90° 주기)에 맞추는 관절값을 구한다.

        안전 검사: z가 z_min_mm 아래인 목표, 닿지 않는 목표, 관절 한계(여유 joint_margin_deg)를 넘는 결과는 거부한다.
        max_tilt_deg: 수직 자세가 관절 한계(손목 굽힘 ±95°) 때문에 안 될 때 이 각도까지 기울어진 그리퍼를 허용한다.
            None이면 tol_tilt_deg(거의 수직)만 허용. 모델에서 수직 집기가 되는 범위는 좁다
            (높이 65mm에서 앞쪽 약 135~255mm, 2026-11-10 계산).
        roll_pref_deg: 블록 각도는 90° 주기라 손목 값 후보가 여럿이다. 기본은 0°에 가장 가까운 후보,
            값을 주면 그 값에 가장 가까운 후보를 고른다(반복 계산·경로 중에 후보가 바뀌어 튀지 않게).
        이 함수는 계산만 하고 팔을 움직이지 않는다.
        """
        target = np.asarray(target_mm, np.float64) / 1000.0
        if target[2] * 1000.0 < z_min_mm:
            return IKResult(False, None, np.nan, np.nan, np.nan, f"목표 높이 {target[2]*1000:.1f}mm < 최소 {z_min_mm}mm")

        seeds = []
        if q_init_deg is not None:
            seeds.append(np.asarray(q_init_deg, np.float64))
        pan = np.rad2deg(np.arctan2(-target[1], target[0]))  # 모델에서 어깨 회전 + 방향 = 로봇 기준 -y 쪽
        seeds += [np.array([pan, 0, 0, 80, 0.0]), np.array([pan, -40, 40, 85, 0.0]),
                  np.array([pan, 30, -30, 90, 0.0]), np.zeros(5)]

        result = self._ik_pass(target, yaw_deg, seeds, max_iter, tol_mm, tol_tilt_deg, down_weight=0.5,
                               roll_pref=roll_pref_deg)
        if result.ok or max_tilt_deg is None or max_tilt_deg <= tol_tilt_deg:
            return result
        # 두 번째 시도: 위치는 반드시 맞추고(hard), 기울기는 가능한 만큼만 줄인다.
        relaxed = self._ik_pass(target, yaw_deg, seeds, max_iter, tol_mm, max_tilt_deg, down_weight=0.5, hard_pos=True,
                                roll_pref=roll_pref_deg)
        return relaxed if relaxed.ok else result

    def _ik_pass(self, target, yaw_deg, seeds, max_iter, tol_mm, tol_tilt_deg, down_weight, hard_pos=False,
                 roll_pref=None) -> IKResult:
        best = None
        for seed in seeds:
            # 그리퍼 끝 점이 손목 회전축에서 조금 벗어나 있어서, 손목을 돌리면 끝 위치도 몇 mm 움직인다.
            # → (손목 고정하고 위치·방향 맞추기) → (블록 각도로 손목 값 정하기)를 손목 값이 멈출 때까지 반복한다.
            q = np.asarray(seed, np.float64).copy()
            q[4] = 0.0
            roll, yaw_err = 0.0, np.nan
            for _ in range(6):
                q, err, tilt = self._solve_position_down(target, q, max_iter, tol_mm, tol_tilt_deg, down_weight, hard_pos)
                new_roll, yaw_err = self._roll_for_yaw(q, yaw_deg, roll_pref)
                if new_roll is None:
                    break
                moved = abs(new_roll - q[4])
                q[4] = new_roll
                if moved < 0.05:
                    break
            q, err, tilt = self._solve_position_down(target, q, max_iter, tol_mm, tol_tilt_deg, down_weight, hard_pos)
            if new_roll is not None:
                yaw_err = abs(_wrap90(self._tool_yaw(q) - yaw_deg))
            cand = (q, err, tilt, new_roll is not None, yaw_err)
            if best is None or (cand[3], -cand[1]) > (best[3], -best[1]):
                best = cand
            if cand[3] and err <= tol_mm and tilt <= tol_tilt_deg:
                break
        q, err, tilt, roll_ok, yaw_err = best
        if not roll_ok:
            return IKResult(False, None, err, tilt, np.nan, "손목 회전 한계 안에서 블록 각도를 맞출 수 없음")
        if err > tol_mm or tilt > tol_tilt_deg:
            return IKResult(False, None, err, tilt, yaw_err, f"닿지 않음: 위치 오차 {err:.1f}mm, 기울기 {tilt:.1f}°")
        bad = self._limit_violation(q)
        if bad:
            return IKResult(False, None, err, tilt, yaw_err, f"관절 한계 초과: {bad}")
        return IKResult(True, q, err, tilt, yaw_err)

    # ---------- 내부 ----------
    def _set_q(self, q_deg):
        q = np.deg2rad(np.asarray(q_deg, np.float64)[:5])
        for j, v in zip(ARM_JOINTS, q):
            self.robot.set_joint(j, float(v))
        self.robot.update_kinematics()

    def _solve_position_down(self, target, seed_deg, max_iter, tol_mm, tol_tilt_deg, down_weight=0.5, hard_pos=False):
        placo = self._placo
        seed = np.asarray(seed_deg, np.float64).copy()
        self._set_q(seed)  # 손목 회전(seed[4])은 이 값으로 고정하고 나머지 4개 관절만 푼다
        solver = placo.KinematicsSolver(self.robot)
        solver.mask_fbase(True)
        solver.enable_joint_limits(True)
        solver.mask_dof("gripper")
        solver.mask_dof("wrist_roll")
        pos = solver.add_position_task(TOOL_FRAME, target)
        pos.configure("pos", "hard" if hard_pos else "soft", 1.0)
        down = solver.add_axisalign_task(TOOL_FRAME, np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0]))
        down.configure("down", "soft", down_weight)
        solver.add_regularization_task(1e-4)
        err = tilt = np.inf
        for _ in range(max_iter):
            try:
                solver.solve(True)
            except RuntimeError:  # hard 조건을 만족할 수 없음 = 닿지 않음
                break
            self.robot.update_kinematics()
            T = self.robot.get_T_world_frame(TOOL_FRAME)
            err = float(np.linalg.norm(T[:3, 3] - target) * 1000.0)
            tilt = float(np.rad2deg(np.arccos(np.clip(-T[2, 2], -1.0, 1.0))))
            if err < tol_mm * 0.3 and tilt < tol_tilt_deg * 0.3:
                break
        q = np.rad2deg([self.robot.get_joint(j) for j in ARM_JOINTS])
        return np.asarray(q, np.float64), err, tilt

    def _tool_yaw(self, q_deg) -> float:
        _, R = self.fk(q_deg)
        x = R[:, 0]  # 그리퍼 x축을 테이블 위에서 본 방향
        return float(np.rad2deg(np.arctan2(x[1], x[0])))

    def _roll_for_yaw(self, q_deg, yaw_deg, roll_pref=None):
        """손목 회전값을 바꿔 가며 그리퍼 방향이 블록 각도(90° 주기)와 맞는 값을 찾는다.
        회전 방향 부호를 가정하지 않으려고 모델로 직접 확인한다."""
        q = np.asarray(q_deg, np.float64).copy()
        q[4] = 0.0
        y0 = self._tool_yaw(q)
        q[4] = 10.0
        s = np.sign(_wrap90(self._tool_yaw(q) - y0)) or 1.0  # 손목 +10° 때 그리퍼 방향이 도는 쪽
        base = _wrap90((yaw_deg - y0) * s)
        lo, hi = self._urdf_limits["wrist_roll"]
        lo, hi = np.rad2deg(lo + self.margin), np.rad2deg(hi - self.margin)
        cands = [base + k * 90.0 for k in range(-3, 4) if lo <= base + k * 90.0 <= hi]
        if not cands:
            return None, np.nan
        pref = 0.0 if roll_pref is None else float(roll_pref)
        roll = min(cands, key=lambda c: abs(c - pref))
        q[4] = roll
        return roll, abs(_wrap90(self._tool_yaw(q) - yaw_deg))

    def _limit_violation(self, q_deg) -> str:
        bad = []
        for j, v in zip(ARM_JOINTS, np.deg2rad(q_deg)):
            lo, hi = self._urdf_limits[j]
            eps = np.deg2rad(0.05)  # 계산기가 한계에 딱 붙여 낸 답은 통과시킨다
            if not (lo + self.margin - eps <= v <= hi - self.margin + eps):
                bad.append(f"{j}={np.rad2deg(v):.1f}°")
        return ", ".join(bad)


def _read_urdf_limits(urdf: Path) -> dict[str, tuple[float, float]]:
    import xml.etree.ElementTree as ET

    out = {}
    for j in ET.parse(urdf).getroot().iter("joint"):
        lim = j.find("limit")
        if j.get("name") in ARM_JOINTS and lim is not None:
            out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


def q_from_observation(obs: dict) -> np.ndarray:
    """LeRobot 팔로워 관측값({'shoulder_pan.pos': deg, ...}) → 관절 5개 [deg]."""
    return np.array([obs[f"{j}.pos"] for j in ARM_JOINTS], np.float64)
