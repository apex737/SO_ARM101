"""ACT 시작 자세 만들기: 블록 위로 접근(선택: 목표 흔들기·더 내려가기·리더암 자동 맞춤)한 뒤 토크를 유지하며 끝낸다.

    cd project/src
    python -m kitbot_exec.approach --color purple --step                          # 보정 없이 40mm 접근
    python -m kitbot_exec.approach --color purple --handoff-clear 15              # 40mm 접근 → 블록 윗면 위 15mm(명령)까지 수직 하강
    python -m kitbot_exec.approach --color purple --jitter-mm 15 --jitter-deg 15  # 접근 목표를 일부러 흔들기 (ACT 시연용)
    python -m kitbot_exec.approach --color purple --align-leader --teleop-s 10    # 실험: 리더암을 팔로워 자세로 자동 이동 → 원격 조종 10초

- 기본은 접근 보정 없음(S02 통과·ACT 인계 조건과 같음). 어제 리더암 보정을 쓰려면 --corr default.
- 자동 HOME 복귀 없음. 끝나면 팔로워는 마지막 자세에서 힘을 유지한다(풀기: python -m kitbot_exec.release).
- 시행마다 시작 조건을 var/approach_log.jsonl에 한 줄로 남긴다.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

# 파일 경로로 직접 실행하는 경우에도 같은 패키지 모듈을 사용한다.
if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "kitbot_exec"

import numpy as np

from .correction import Correction, load_samples, wrap90
from .kinematics import ARM_JOINTS
from .pick import CALIB, PickParams, detect_block_pose, outside_mm, plan_pick

SRC = Path(__file__).resolve().parents[1]
LOG = SRC / "var" / "approach_log.jsonl"
DEFAULT_CORR = SRC / "var" / "approach_corrections.json"

ARRIVAL_TOL_DEG = 5.0     # 모든 팔 관절의 |목표 − 현재|가 이 값 미만이면 도착 (평소 팔꿈치 2.2~2.6°)
STILL_WINDOW_S = 0.3      # 이 시간 동안 관절 변화가 STILL_TOL_DEG 이하이면 멈춘 것으로 본다
STILL_TOL_DEG = 0.2
STILL_TIMEOUT_S = 1.5
MIN_HANDOFF_CLEAR_MM = 5.0  # 블록 윗면 아래로 내려가지 않게 (실제는 명령보다 몇 mm 낮음)


def _r(v, n=2):
    return np.round(np.asarray(v, float), n).tolist()


# ---------- 도착 확인 ----------
def report_arrival(kin, q_target, q_read, jaw_center_mm):
    """목표−현재 오차를 출력한다. 위치는 관절값 기반 모델 추정이다."""
    target, current = np.asarray(q_target, float), np.asarray(q_read, float)
    if target.shape != (5,) or current.shape != (5,) or not np.isfinite([target, current]).all():
        raise RuntimeError("도착 진단 실패: 관절값이 유효하지 않습니다.")
    delta = target - current
    print("[도착 진단] 차이 = 목표 − 현재 (양수: 각도를 더 증가, 음수: 더 감소)", flush=True)
    for joint, goal, actual, diff in zip(ARM_JOINTS, target, current, delta):
        status = "허용 범위 밖" if abs(diff) >= ARRIVAL_TOL_DEG else "허용 범위 안"
        print(f"  {joint}: 목표 {goal:.2f}°, 현재 {actual:.2f}°, 차이 {diff:+.2f}° [{status}]")
    tip_target, rotation_target = kin.fk(target)
    tip_read, rotation_read = kin.fk(current)
    xy_target = tip_target[:2] + rotation_target[:2, :2] @ np.asarray(jaw_center_mm)
    xy_read = tip_read[:2] + rotation_read[:2, :2] @ np.asarray(jaw_center_mm)
    offset = np.r_[xy_target - xy_read, tip_target[2] - tip_read[2]]
    print("  모델 기준 집게 중심 XY / 집게 끝 Z [mm]:")
    print(f"    목표: x={xy_target[0]:.2f}, y={xy_target[1]:.2f}, z={tip_target[2]:.2f}")
    print(f"    현재: x={xy_read[0]:.2f}, y={xy_read[1]:.2f}, z={tip_read[2]:.2f}")
    print(f"    목표−현재: dx={offset[0]:+.2f}, dy={offset[1]:+.2f}, dz={offset[2]:+.2f}")
    print("    로봇 좌표축 기준입니다. dz가 양수면 현재 집게 끝이 목표보다 낮습니다.")
    print("    카메라 실측이 아닌 모델 추정이며, 관절별 오차의 원인까지 판정하지는 않습니다.", flush=True)
    return float(np.max(np.abs(delta)))


def wait_still(arm, timeout_s=None, window_s=None, tol_deg=None):
    """관절값이 window_s 동안 tol_deg 넘게 변하지 않을 때까지(최대 timeout_s) 기다린 뒤 마지막 값을 돌려준다.
    move_joints는 목표 근처(5°)에 들어오는 순간 끝나므로, 아직 움직이는 자세를 시작 자세로 남기지 않으려는 것."""
    timeout_s = STILL_TIMEOUT_S if timeout_s is None else timeout_s
    window_s = STILL_WINDOW_S if window_s is None else window_s
    tol_deg = STILL_TOL_DEG if tol_deg is None else tol_deg
    t0 = time.monotonic()
    hist = []
    while True:
        q, g = arm.read()
        now = time.monotonic()
        hist.append((now, np.asarray(q, float)))
        hist = [(t, v) for t, v in hist if now - t <= window_s]
        span = now - hist[0][0]
        moving = np.max(np.ptp(np.array([v for _, v in hist]), axis=0)) if len(hist) > 1 else np.inf
        if span >= window_s * 0.9 and moving <= tol_deg:
            return np.asarray(q, float), g, True
        if now - t0 >= timeout_s:
            return np.asarray(q, float), g, False
        time.sleep(0.03)


# ---------- 목표 흔들기 ----------
def jitter_target(rng, x, y, yaw, jitter_mm: float, jitter_deg: float):
    """접근 목표를 반지름 jitter_mm 원 안(면적 균등)·각도 ±jitter_deg(균등)로 흔든다. (x, y, yaw, dx, dy, dyaw)."""
    if jitter_mm <= 0 and jitter_deg <= 0:
        return x, y, yaw, 0.0, 0.0, 0.0
    r = jitter_mm * np.sqrt(rng.uniform())
    th = rng.uniform(0, 2 * np.pi)
    dx, dy = r * np.cos(th), r * np.sin(th)
    dyaw = rng.uniform(-jitter_deg, jitter_deg) if jitter_deg > 0 else 0.0
    return x + dx, y + dy, float(wrap90(yaw + dyaw)), float(dx), float(dy), float(dyaw)


# ---------- 접근 ----------
def approach_block(arm, cap, gains, kin, calib, poses, correction, *, color="purple", step=None,
                   jitter_mm=0.0, jitter_deg=0.0, rng=None, handoff_clear_mm=None, max_jitter_tries=5, nearest=False):
    """연결된 Arm을 사용한다. 연결 해제는 호출자가 담당한다. 끝난 자세의 기록(dict, "plan" 포함)을 돌려준다.

    성공은 관절 목표 도달을 뜻하며, 영상으로 실제 중심 정렬을 검증하지는 않는다.
    """
    step = step or (lambda message: print(message, flush=True))
    p = PickParams()
    q_home = np.asarray(poses["home"]["q_deg"], float)
    g_open = float(poses["gripper_pregrasp"]["gripper"])
    g_closed = float(poses["gripper_closed"]["gripper"])
    if q_home.shape != (5,) or not np.isfinite(q_home).all():
        raise ValueError("HOME 관절값은 유효한 각도 5개여야 합니다.")
    if not all(np.isfinite(g) and 0 <= g <= 100 for g in (g_open, g_closed)):
        raise ValueError("그리퍼 값은 0~100 범위여야 합니다.")
    if handoff_clear_mm is not None and handoff_clear_mm < MIN_HANDOFF_CLEAR_MM:
        raise ValueError(f"--handoff-clear는 {MIN_HANDOFF_CLEAR_MM:.0f}mm 이상이어야 합니다(블록 윗면 아래로 내려가지 않게).")
    rng = rng or np.random.default_rng()
    rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "color": color, "correction": correction.mode,
           "correction_n": correction.n}

    step("집게를 닫으면서 HOME으로 이동")
    if not arm.move_joints(q_home, g_closed, p.joint_speed_dps, settle_tol_deg=ARRIVAL_TOL_DEG):
        q_read, _ = arm.read()
        report_arrival(kin, q_home, q_read, p.jaw_center_mm)
        raise RuntimeError("HOME 도착 시간 초과: 접근을 중단합니다.")
    # HOME 이동 전에 버퍼에 남아 있던 영상을 비운다.
    for _ in range(5):
        cap.read()
    pose, _, note = detect_block_pose(cap, gains, color, calib, nearest=nearest)
    print(note, flush=True)
    if pose is None:
        raise RuntimeError(f"블록을 검출하지 못했습니다: {note}")
    if not np.isfinite(pose).all():
        raise RuntimeError("블록 좌표가 유효하지 않습니다.")
    x0, y0, z_top, yaw0 = (float(v) for v in pose)
    rec["block_camera"] = _r([x0, y0, z_top, yaw0], 1)
    out_cal = outside_mm(calib, x0, y0)
    rec["outside_calib_mm"] = None if not np.isfinite(out_cal) else round(max(float(out_cal), 0.0), 1)   # 0 = 보정점 범위 안
    xc, yc, yawc = correction.apply(x0, y0, yaw0)
    if not np.isfinite([xc, yc, z_top, yawc]).all():
        raise RuntimeError("보정된 목표가 유효하지 않습니다.")

    plan = None
    for _ in range(max_jitter_tries):
        x, y, yaw, dx, dy, dyaw = jitter_target(rng, xc, yc, yawc, jitter_mm, jitter_deg)
        plan = plan_pick(kin, x, y, z_top, yaw, q_home, p)
        if plan.ok or (jitter_mm <= 0 and jitter_deg <= 0):
            break
        print(f"흔든 목표 계획 실패({plan.reason}) → 다시 뽑음", flush=True)
    if not plan.ok:
        raise RuntimeError(f"이동 계획 실패: {plan.reason}")
    rec.update(target=_r([x, y, yaw], 1), jitter=_r([dx, dy, dyaw], 1), jitter_range=[jitter_mm, jitter_deg])
    if jitter_mm > 0 or jitter_deg > 0:
        print(f"목표 흔들기: dx {dx:+.1f}, dy {dy:+.1f} mm, 각도 {dyaw:+.1f}°", flush=True)

    step(f"집게를 벌리면서 블록 윗면 위 {p.approach_mm:.0f}mm로 접근")
    settled = arm.move_joints(plan.q_pre, g_open, p.joint_speed_dps, settle_tol_deg=ARRIVAL_TOL_DEG)
    q_read, _, still = wait_still(arm)
    error = report_arrival(kin, plan.q_pre, q_read, p.jaw_center_mm)
    if not settled:
        raise RuntimeError("접근 도착 시간 초과: 위 진단을 확인하세요. 현재 자세를 유지합니다.")
    if error >= ARRIVAL_TOL_DEG:
        raise RuntimeError(f"도착 확인 실패: 최대 관절 오차 {error:.2f}°")
    q_final, q_goal = q_read, np.asarray(plan.q_pre, float)

    if handoff_clear_mm is not None:
        from .exp_s02 import line_path  # 접근 자세에서 수직 직선 하강 (exp_s02 straddle과 같은 계산)

        z_pre = float(kin.fk(plan.q_pre)[0][2])
        z_min = float(calib.z_at([[x, y]])[0]) - p.block_mm + p.table_margin_mm
        path = line_path(kin, x, y, yaw, z_pre, z_top + handoff_clear_mm, plan.q_pre, p, z_min, float(plan.q_pre[4]))
        step(f"수직으로 내려가기: 집게 끝이 블록 윗면 위 {handoff_clear_mm:.0f}mm (명령 기준, 실제는 몇 mm 더 낮을 수 있음)")
        arm.follow(path, g_open)
        q_read, _, still = wait_still(arm)
        q_goal = np.asarray(path[-1], float)
        error = report_arrival(kin, q_goal, q_read, p.jaw_center_mm)
        if error >= ARRIVAL_TOL_DEG:
            raise RuntimeError(f"하강 도착 확인 실패: 최대 관절 오차 {error:.2f}°")
        q_final = q_read
        rec["handoff_clear_mm"] = handoff_clear_mm

    tip, _ = kin.fk(q_final)
    rec.update(q_goal=_r(q_goal), q_read=_r(q_final), max_joint_err_deg=round(error, 2), still=bool(still),
               tip_z_read_mm=round(float(tip[2]), 1), clear_read_mm_model=round(float(tip[2] - z_top), 1),
               gripper_open=g_open)
    print(f"이동이 끝났습니다. (최대 관절 오차 {error:.2f}°, 멈춤 확인 {'예' if still else '시간 초과'})", flush=True)
    rec["plan"] = plan
    return rec


# ---------- 리더암 자동 맞춤 (실험) ----------
def align_leader(leader, q_target, g_target, *, speed_dps=20.0, torque_limit=400, hz=30.0,
                 confirm=None, settle_s=1.0, sleep=time.sleep):
    """리더암 서보에 잠깐 힘을 넣어 팔로워 자세(q_target, g_target)로 천천히 옮긴다.
    사람이 리더암을 손으로 잡고 확인하면 힘을 풀고, 남은 관절 차이를 돌려준다.
    리더암 서보 힘은 torque_limit(0~1000)으로 낮춰 두고, 끝나면 원래 값으로 되돌린다."""
    confirm = confirm or (lambda: input("  리더암을 손으로 잡으세요. Enter = 리더암 힘 풀기 "))
    bus = leader.teleop.bus
    names = list(ARM_JOINTS) + ["gripper"]
    target = {n: float(v) for n, v in zip(names, list(q_target) + [g_target])}
    present = bus.sync_read("Present_Position")
    start = {n: float(present[n]) for n in names}
    gap_before = {n: round(target[n] - start[n], 2) for n in names}
    old_limits = bus.sync_read("Torque_Limit", normalize=False)
    out = {"gap_before_deg": gap_before, "torque_limit": torque_limit}
    torque_on = False
    try:
        bus.sync_write("Goal_Position", start)               # 지금 자리를 목표로 둔 채 힘을 켜야 튀지 않는다
        bus.sync_write("Torque_Limit", {n: int(torque_limit) for n in names}, normalize=False)
        bus.enable_torque()
        torque_on = True
        span = max(abs(target[n] - start[n]) for n in names)
        steps = max(int(span / speed_dps * hz), 2)
        for i in range(1, steps + 1):
            s = 0.5 - 0.5 * np.cos(np.pi * i / steps)
            bus.sync_write("Goal_Position", {n: start[n] + (target[n] - start[n]) * s for n in names})
            sleep(1.0 / hz)
        sleep(settle_s)
        held = bus.sync_read("Present_Position")
        out["gap_held_deg"] = {n: round(target[n] - float(held[n]), 2) for n in names}
        confirm()
    finally:
        if torque_on:
            bus.disable_torque()
        try:
            bus.sync_write("Torque_Limit", {n: int(old_limits[n]) for n in names}, normalize=False)
        except Exception as e:  # 힘은 이미 풀었으므로 경고만
            print("  리더암 Torque_Limit 복원 실패(전원을 껐다 켜면 원래 값):", e)
    sleep(0.5)
    rel = bus.sync_read("Present_Position")
    out["gap_released_deg"] = {n: round(target[n] - float(rel[n]), 2) for n in names}
    out["max_gap_released_deg"] = round(max(abs(v) for k, v in out["gap_released_deg"].items() if k != "gripper"), 2)
    return out


def teleop_probe(arm, leader, seconds: float, hz=30.0):
    """리더 → 팔로워 원격 조종을 seconds초 동안 하며, 처음 0.5초 안에 팔로워가 튀었는지 잰다."""
    q0, _ = arm.read()
    t0 = time.monotonic()
    first_jump = 0.0
    while time.monotonic() - t0 < seconds:
        arm.send_action(leader.action())
        q, _ = arm.read()
        if time.monotonic() - t0 <= 0.5:
            first_jump = max(first_jump, float(np.max(np.abs(q - q0))))
        time.sleep(1.0 / hz)
    return {"teleop_s": seconds, "first_0_5s_max_joint_change_deg": round(first_jump, 2)}


# ---------- 실행 ----------
def build_correction(corr_arg: str | None) -> Correction:
    """기본 = 보정 없음(S02 통과 기준). 'default' = var/approach_corrections.json, 그 밖 = 파일 경로."""
    if corr_arg in (None, "", "none"):
        return Correction([])
    return Correction(load_samples(DEFAULT_CORR if corr_arg == "default" else Path(corr_arg)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    from .vision import HSV_RANGES
    parser.add_argument("--color", default="purple", choices=sorted(HSV_RANGES),
                        help=f"접근할 블록 색. 가능: {', '.join(sorted(HSV_RANGES))}")
    parser.add_argument("--step", action="store_true", help="이동 전 Enter 대기")
    parser.add_argument("--corr", default="none",
                        help="접근 보정: none(기본, S02 통과 기준) / default(var/approach_corrections.json) / 파일 경로")
    parser.add_argument("--no-correction", action="store_true", help="(예전 옵션, 지금은 기본이 보정 없음)")
    parser.add_argument("--jitter-mm", type=float, default=0.0, help="접근 목표를 이 반지름 안에서 흔든다 [mm]. 시연용 15")
    parser.add_argument("--jitter-deg", type=float, default=0.0, help="손목 각도를 ±이 값 안에서 흔든다 [°]. 시연용 15")
    parser.add_argument("--seed", type=int, help="흔들기 난수 씨앗 (같은 값이면 같은 흔들기)")
    parser.add_argument("--handoff-clear", type=float, help="40mm 접근 뒤 집게 끝이 블록 윗면 위 이 높이까지 수직 하강 [mm, 명령]. 5 이상")
    parser.add_argument("--align-leader", action="store_true", help="(실험) 끝난 뒤 리더암을 팔로워 자세로 자동 이동")
    parser.add_argument("--leader-torque", type=int, default=400, help="리더암 자동 이동 때 서보 힘 한도 0~1000 (기본 400)")
    parser.add_argument("--teleop-s", type=float, default=0.0, help="(실험) 리더암 맞춤 뒤 원격 조종을 이 시간만큼 하며 튐을 잰다")
    parser.add_argument("--log", default=str(LOG), help="시작 조건 기록 파일 (JSON 한 줄씩)")
    args = parser.parse_args(argv)
    if args.teleop_s > 0 and not args.align_leader:
        parser.error("--teleop-s는 --align-leader와 함께 써야 합니다 (리더암을 맞추지 않으면 팔로워가 튑니다)")

    from .arm import Arm, Leader, load_poses
    from .calib import PixelToRobot
    from .camera import open_role
    from .kinematics import SO101Kinematics

    poses = load_poses()
    for key in ("home", "gripper_pregrasp", "gripper_closed"):
        if key not in poses:
            raise RuntimeError(f"poses.json에 {key}가 없습니다. teach로 저장하세요.")
    calib = PixelToRobot.load(CALIB)
    kin = SO101Kinematics()
    correction = build_correction("none" if args.no_correction else args.corr)
    print(correction.summary())
    arm = cap = leader = None
    rec = None

    def step(message):
        print(message, flush=True)
        if args.step:
            input("Enter = 진행, Ctrl+C = 중단: ")

    try:
        cap, gains = open_role("top")
        arm = Arm(hold_on_exit=True)
        rec = approach_block(arm, cap, gains, kin, calib, poses, correction, color=args.color, step=step,
                             jitter_mm=args.jitter_mm, jitter_deg=args.jitter_deg,
                             rng=np.random.default_rng(args.seed), handoff_clear_mm=args.handoff_clear)
        if args.align_leader:
            leader = Leader()
            q, g = arm.read()
            step(f"리더암 자동 맞춤(실험): 리더암 서보 힘 {args.leader_torque}/1000으로 팔로워 자세까지 천천히 이동")
            rec["leader"] = align_leader(leader, q, g, torque_limit=args.leader_torque)
            lg = rec["leader"]
            print(f"  맞추기 전 관절 차이 최대 {max(abs(v) for v in lg['gap_before_deg'].values()):.1f}°, "
                  f"힘 켠 채 {max(abs(v) for v in lg['gap_held_deg'].values()):.1f}°, "
                  f"힘 푼 뒤(그리퍼 제외) {lg['max_gap_released_deg']:.1f}°")
            if args.teleop_s > 0:
                step(f"원격 조종 {args.teleop_s:.0f}초 (처음 0.5초 동안 팔로워가 튀는지 잰다)")
                rec["leader"].update(teleop_probe(arm, leader, args.teleop_s))
                print(f"  원격 조종 처음 0.5초 팔로워 관절 변화 최대 {rec['leader']['first_0_5s_max_joint_change_deg']:.1f}°")
    finally:
        if rec is not None:
            rec.pop("plan", None)
            log = Path(args.log)
            log.parent.mkdir(parents=True, exist_ok=True)
            with log.open("a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            print("기록:", log)
        try:
            if leader is not None:
                leader.disconnect()
        finally:
            try:
                if cap is not None:
                    cap.release()
            finally:
                if arm is not None:
                    arm.disconnect()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("중단했습니다. 자동 HOME 복귀는 하지 않습니다.")
        raise SystemExit(130)
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"접근 실패: {exc}", file=sys.stderr)
        raise SystemExit(1)
