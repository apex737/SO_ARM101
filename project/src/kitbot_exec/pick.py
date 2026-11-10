"""학습 없이 한 바퀴: 블록 찾기 → 블록 위 접근 + 손목 회전 → 내려가 집기 → 들어 올리기 → 바구니에 놓기 → HOME (conda lerobot).

    cd project/src
    python -m kitbot_exec.pick --color red --dry      # 팔을 움직이지 않고 계산·검사만
    python -m kitbot_exec.pick --color red --step     # 단계마다 Enter를 눌러야 다음으로 (처음엔 이걸로)
    python -m kitbot_exec.pick --color red            # 한 번에

필요한 것: calib_top.json(collect_calib), poses.json의 home·basket(teach). 그리퍼 값은 teach의 g·c로 저장한 값을 쓴다.
높이는 모두 로봇 모델 좌표(mm). 블록 윗면 높이 z_top은 보정점에서 구한 평면, 테이블 = z_top − 블록 한 변.
모든 목표는 움직이기 전에 계산·검사하고, 하나라도 실패하면 팔을 움직이지 않는다.
결과는 var/pick_log.jsonl에 한 줄씩 남는다(S02-7 판정 실험 기록용).
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .correction import Correction, add_sample, load_samples, wrap90
from .kinematics import ARM_JOINTS, SO101Kinematics

CALIB = Path(__file__).with_name("calib_top.json")
LOG = Path(__file__).resolve().parents[1] / "var" / "pick_log.jsonl"


@dataclass
class PickParams:
    block_mm: float = 25.0          # 블록 한 변 (measurements.json M01, 실측)
    approach_mm: float = 40.0       # 블록 윗면 위 접근 높이 (사용자 확정 30~50mm)
    grasp_depth_mm: float = 12.0    # 블록 윗면에서 그리퍼 끝을 얼마나 내릴지 (블록 가운데 높이쯤)
    lift_mm: float = 40.0           # 집은 뒤 블록 윗면 기준으로 올릴 높이 (60은 받침대 가까운 곳에서 닿지 않음, 2026-11-10)
    table_margin_mm: float = 3.0    # 그리퍼 끝이 테이블 위 이만큼 아래로는 못 가게
    max_tilt_deg: float = 10.0      # 수직이 안 되는 위치에서 허용할 그리퍼 기울기
    line_step_mm: float = 3.0       # 직선 이동 경로 간격
    line_speed_mms: float = 30.0    # 직선 이동 속도
    joint_speed_dps: float = 40.0   # 관절 이동 속도
    # 두 부리 가운데(벌렸을 때)가 모델 '그리퍼 끝'(고정 부리 쪽)에서 그리퍼 좌표로 얼마나 떨어져 있나 [mm].
    # 2026-11-10 접근 기록 7건에서 22.2mm로 맞춰짐(= 벌림 45mm의 절반). 블록 중심에 이 점을 맞춘다.
    # 이 값이 없으면 손목 방향에 따라 22mm 차이가 다른 쪽으로 생겨 평균 보정으로는 못 잡는다.
    jaw_center_mm: tuple = (-22.0, 0.0)


@dataclass
class PickPlan:
    ok: bool
    reason: str = ""
    block: dict = field(default_factory=dict)      # x, y, z_top, yaw
    q_pre: np.ndarray | None = None                 # 블록 위 접근 자세
    down: list = field(default_factory=list)        # 접근 → 집기 높이 관절 경로
    up: list = field(default_factory=list)          # 집기 높이 → 들어 올린 높이 관절 경로
    info: dict = field(default_factory=dict)


def jaw_offset_xy(kin: SO101Kinematics, q_deg, jaw_center_mm) -> np.ndarray:
    """관절값 q에서 '두 부리 가운데 − 모델 그리퍼 끝'의 수평 차이 (로봇 좌표 x, y mm)."""
    _, R = kin.fk(q_deg)
    return R[:2, :2] @ np.asarray(jaw_center_mm, float)


def jaw_center_xy(kin: SO101Kinematics, q_deg, jaw_center_mm) -> np.ndarray:
    """관절값 q에서 두 부리 가운데의 로봇 좌표 x, y [mm]."""
    p, _ = kin.fk(q_deg)
    return p[:2] + jaw_offset_xy(kin, q_deg, jaw_center_mm)


def plan_pick(kin: SO101Kinematics, x, y, z_top, yaw, q_seed, p: PickParams = PickParams()) -> PickPlan:
    """블록 위치·각도 → 관절 경로. 팔을 움직이지 않는다."""
    z_table = z_top - p.block_mm
    z_min = z_table + p.table_margin_mm
    z_pre, z_grasp, z_lift = z_top + p.approach_mm, z_top - p.grasp_depth_mm, z_top + p.lift_mm
    block = {"x": round(float(x), 1), "y": round(float(y), 1), "z_top": round(float(z_top), 1), "yaw": round(float(yaw), 1)}
    if z_grasp < z_min:
        return PickPlan(False, f"집기 높이 {z_grasp:.1f} < 테이블 + 여유 {z_min:.1f}", block)

    roll_keep = [None]  # 처음 고른 손목 값 후보를 끝까지 유지 (90° 주기 후보가 바뀌면 22mm 차이 방향도 돌아감)

    def ik(z, seed):
        # 두 부리 가운데가 (x, y)에 오도록 모델 그리퍼 끝 목표를 옮긴다. 옮길 방향은 손목 방향에 달려 있어 몇 번 반복.
        off = np.zeros(2)
        for _ in range(6):
            r = kin.ik_top_down((x - off[0], y - off[1], z), yaw, seed, z_min_mm=z_min, max_tilt_deg=p.max_tilt_deg,
                                roll_pref_deg=roll_keep[0])
            if not r.ok:
                return r
            if roll_keep[0] is None:
                roll_keep[0] = float(r.q_deg[4])
            new = jaw_offset_xy(kin, r.q_deg, p.jaw_center_mm)
            if np.linalg.norm(new - off) < 0.2:
                return r
            off, seed = new, r.q_deg
        c = jaw_center_xy(kin, r.q_deg, p.jaw_center_mm)
        miss = float(np.hypot(c[0] - x, c[1] - y))
        if miss > 1.5:
            return type(r)(False, None, miss, r.tilt_deg, r.yaw_err_deg, f"두 부리 가운데를 맞추지 못함 ({miss:.1f}mm)")
        return r

    pre = ik(z_pre, q_seed)
    if not pre.ok:
        return PickPlan(False, f"접근 자세: {pre.reason}", block)

    def line(z0, z1, seed):
        n = max(int(abs(z1 - z0) / p.line_step_mm), 1)
        path, q, worst = [], seed, 0.0
        for z in np.linspace(z0, z1, n + 1)[1:]:
            r = ik(z, q)
            if not r.ok:
                return None, f"높이 {z:.1f}mm: {r.reason}", worst
            # 경로 중간에 관절이 크게 튀면(다른 자세 해로 넘어감) 거부
            jump = float(np.max(np.abs(r.q_deg - q)))
            worst = max(worst, jump)
            if jump > 15.0:
                return None, f"높이 {z:.1f}mm에서 관절이 {jump:.0f}° 튐", worst
            path.append(r.q_deg)
            q = r.q_deg
        return path, "", worst

    down, why, j1 = line(z_pre, z_grasp, pre.q_deg)
    if down is None:
        return PickPlan(False, f"내려가기: {why}", block)
    up, why, j2 = line(z_grasp, z_lift, down[-1])
    if up is None:
        return PickPlan(False, f"들어 올리기: {why}", block)
    # 직선 이동 속도에 맞게 경로를 늘린다 (HZ=30 기준)
    per_step = p.line_step_mm / p.line_speed_mms * 30.0
    stretch = lambda path: [q for q in path for _ in range(max(int(round(per_step)), 1))]
    info = {"z_pre": round(float(z_pre), 1), "z_grasp": round(float(z_grasp), 1), "z_lift": round(float(z_lift), 1),
            "tilt_pre_deg": round(pre.tilt_deg, 1), "pos_err_pre_mm": round(pre.pos_err_mm, 2),
            "max_joint_jump_deg": round(max(j1, j2), 1)}
    return PickPlan(True, "", block, pre.q_deg, stretch(down), stretch(up), info)


def detect_block_pose(cap, gains, color, calib, n_frames=15):
    """여러 장에서 같은 블록을 찾아 평균. 블록이 정확히 하나여야 함(가장 큰 것을 고르지 않는다: 잘못 집는 것 방지)."""
    from .camera import correct_wb
    from .vision import detect_blocks

    poses, frame = [], None
    for _ in range(n_frames * 2):
        ok, frame = cap.read()
        if not ok:
            continue
        frame = correct_wb(frame, gains)
        blocks = detect_blocks(frame, color)
        if len(blocks) == 1:
            poses.append(calib.block_pose(blocks[0]))
        if len(poses) >= n_frames:
            break
    if len(poses) < n_frames // 2:
        return None, frame, f"'{color}' 블록이 정확히 1개 보이지 않음"
    P = np.asarray(poses)
    yaw = np.rad2deg(np.arctan2(np.sin(np.deg2rad(4 * P[:, 3])).mean(), np.cos(np.deg2rad(4 * P[:, 3])).mean()) / 4)
    pose = (*P[:, :3].mean(axis=0), float(yaw))
    spread = float(np.max(np.linalg.norm(P[:, :2] - P[:, :2].mean(axis=0), axis=1)))
    return pose, frame, f"블록 위치 흔들림 최대 {spread:.1f}mm ({len(P)}장)"


def screen_axes(calib, x, y) -> np.ndarray:
    """블록 위치에서 '화면 오른쪽'·'화면 위쪽' 1mm가 로봇 좌표로 어느 방향인지 (2×2 열벡터)."""
    import cv2
    px = cv2.perspectiveTransform(np.array([[[x, y]]], float), np.linalg.inv(calib.H)).reshape(2)
    p0 = calib.to_xy(px)[0]
    right = calib.to_xy(px + [10.0, 0.0])[0] - p0
    up = calib.to_xy(px + [0.0, -10.0])[0] - p0
    return np.c_[right / np.linalg.norm(right), up / np.linalg.norm(up)]


def measure_with_leader(arm, leader, kin, q_ref, axes, jaw_center_mm=PickParams.jaw_center_mm,
                        max_gap_deg: float = 15.0):
    """리더암으로 팔로워를 '정확한 위치·각도'로 옮기게 하고 그때 관절값을 돌려준다. 취소하면 None.

    ① 리더암을 지금 팔로워 자세에 맞춘다(관절 차이 표시). 차이가 max_gap_deg 안이어야 넘어간다.
       (차이가 크면 연결하는 순간 팔로워가 리더 자세로 휙 움직인다)
    ② 팔로워가 리더 자세로 천천히 옮겨 간 뒤 원격 조종. 목표 대비 어긋남을 실시간 표시.
    ③ Enter → 그 순간 관절값 기록.
    """
    import threading

    names = [f"{j}.pos" for j in ARM_JOINTS]

    def leader_q():
        a = leader.action()
        return np.array([a[n] for n in names], float), float(a["gripper.pos"])

    def live(fn, period=0.2):
        stop = threading.Event()

        def run():
            while not stop.is_set():
                fn()
                time.sleep(period)
        th = threading.Thread(target=run, daemon=True)
        th.start()
        return stop, th

    # ① 리더 맞추기
    print("\n[리더암 맞추기] 리더암을 지금 팔로워암 자세와 똑같이 맞추세요 (손목 회전 포함).")
    print(f"  아래 '관절 차이'가 모두 {max_gap_deg:.0f}° 안이 되면 Enter. 취소: x Enter")
    while True:
        def show_gap():
            gap = np.abs(leader_q()[0] - q_ref)
            print(f"\r  관절 차이 최대 {gap.max():5.1f}°  각 관절 {np.round(gap).astype(int).tolist()}      ", end="", flush=True)
        stop, th = live(show_gap)
        cmd = input().strip().lower()
        stop.set(); th.join()
        if cmd == "x":
            return None
        ql, gl = leader_q()
        gap = np.abs(ql - q_ref)
        if gap.max() <= max_gap_deg:
            break
        print(f"  아직 {gap.max():.0f}° 차이 (가장 큰 관절: {ARM_JOINTS[int(gap.argmax())]}). 더 맞추고 Enter")

    # ② 원격 조종으로 정확한 위치에 맞추기
    arm.move_joints(ql, gl, 20.0)
    p_ref, _ = kin.fk(q_ref)
    c_ref = jaw_center_xy(kin, q_ref, jaw_center_mm)  # 두 부리 가운데 기준
    yaw_ref = kin._tool_yaw(q_ref)
    lock = threading.Lock()

    def teleop_and_show():
        with lock:
            for _ in range(6):  # 0.2초 동안 30Hz
                arm.send_action(leader.action())
                time.sleep(1.0 / 30)
            q, _ = arm.read()
        p, R = kin.fk(q)
        d = jaw_center_xy(kin, q, jaw_center_mm) - c_ref
        r, u = np.linalg.solve(axes, d)
        tilt = np.rad2deg(np.arccos(np.clip(-R[2, 2], -1, 1)))
        dyaw = float(wrap90(kin._tool_yaw(q) - yaw_ref))
        print(f"\r  처음 위치 대비: 화면 오른쪽 {r:+6.1f} 위 {u:+6.1f} mm | 회전 {dyaw:+5.1f}° | "
              f"높이 {p[2] - p_ref[2]:+6.1f} mm | 기울기 {tilt:4.1f}°   ", end="", flush=True)

    print("\n[정확한 위치로] 리더암으로 두 부리 가운데를 블록 중심 바로 위(1cm쯤)에, 부리 방향을 블록 면과 나란히 맞추세요.")
    print("  그리퍼는 수직에 가깝게(기울기 10° 안). 맞췄으면 Enter, 취소: x Enter")
    stop, th = live(teleop_and_show, 0.0)  # 원격 조종은 끊기지 않게 쉬지 않고 돈다
    cmd = input().strip().lower()
    stop.set(); th.join()
    q, _ = arm.read()
    print()
    return None if cmd == "x" else q


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--color", default="red")
    ap.add_argument("--dry", action="store_true", help="팔을 움직이지 않고 계산·검사만")
    ap.add_argument("--step", action="store_true", help="단계마다 Enter")
    ap.add_argument("--no-place", action="store_true",
                    help="블록 위 접근까지만. 멈춘 뒤 리더암으로 정확한 위치에 맞추면 어긋남을 기록해 다음부터 자동 보정")
    ap.add_argument("--no-correction", action="store_true", help="기록된 접근 보정을 쓰지 않음")
    args = ap.parse_args()

    from .arm import Arm, Leader, load_poses
    from .calib import PixelToRobot
    from .camera import open_role

    if not CALIB.exists():
        raise SystemExit("calib_top.json이 없습니다. 먼저 python -m kitbot_exec.collect_calib")
    poses = load_poses()
    # 바구니 자세는 놓을 때만 필요하다 (--no-place는 블록 위 접근까지만)
    for need in ("home",) if args.no_place else ("home", "basket"):
        if need not in poses:
            raise SystemExit(f"poses.json에 {need}가 없습니다. 먼저 python -m kitbot_exec.teach")
    g_open = poses.get("gripper_pregrasp", {}).get("gripper")
    g_closed = poses.get("gripper_closed", {}).get("gripper")
    if g_open is None or g_closed is None:
        raise SystemExit("그리퍼 값이 없습니다. teach에서 g(집기 직전 벌림)와 c(닫힘)를 저장하세요")

    params = PickParams()
    kin = SO101Kinematics()
    calib = PixelToRobot.load(CALIB)
    q_home = np.array(poses["home"]["q_deg"])
    log = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "color": args.color, "dry": args.dry}

    def stage(name):
        print(f"\n[{name}]")
        if args.step and not args.dry:
            input("  Enter = 진행, Ctrl+C = 중단 ")

    corr = Correction([] if args.no_correction else load_samples())
    print(corr.summary())
    arm = None if args.dry else Arm()
    leader = Leader() if (args.no_place and arm) else None
    cap, gains = open_role("top")
    try:
        if arm:
            stage("HOME으로 (카메라가 블록을 볼 수 있게 팔을 비킴)")
            arm.move_joints(q_home, g_open, params.joint_speed_dps)
        pose, _, note = detect_block_pose(cap, gains, args.color, calib)
        print("검출:", note)
        if pose is None:
            log.update(result="no_block"); return
        x0, y0, z_top, yaw0 = pose
        x, y, yaw = corr.apply(x0, y0, yaw0)
        if corr.mode != "none":
            print(f"보정 적용: 카메라 ({x0:.1f}, {y0:.1f}, {yaw0:+.1f}°) → ({x:.1f}, {y:.1f}, {yaw:+.1f}°)")
        plan = plan_pick(kin, x, y, z_top, yaw, q_home, params)
        log.update(block_camera=[round(x0, 1), round(y0, 1), round(yaw0, 1)], correction=corr.mode,
                   block=plan.block, plan_ok=plan.ok, plan_reason=plan.reason, plan_info=plan.info)
        print("블록 (로봇 좌표):", plan.block)
        if not plan.ok:
            print("계획 실패 → 움직이지 않음:", plan.reason)
            log.update(result="plan_failed"); return
        print("계획:", plan.info)
        if args.dry:
            log.update(result="dry_ok"); return

        stage(f"블록 위 {params.approach_mm:.0f}mm로 이동 + 손목 회전")
        arm.move_joints(plan.q_pre, g_open, params.joint_speed_dps)
        if args.no_place:
            q_user = measure_with_leader(arm, leader, kin, plan.q_pre, screen_axes(calib, x, y), params.jaw_center_mm)
            if q_user is not None:
                p_user = jaw_center_xy(kin, q_user, params.jaw_center_mm)  # 두 부리 가운데
                yaw_user = yaw + float(wrap90(kin._tool_yaw(q_user) - kin._tool_yaw(plan.q_pre)))
                samples = add_sample((x0, y0, yaw0), (p_user[0], p_user[1], yaw_user),
                                     {"applied": corr.mode})
                d = samples[-1]["d"]
                r, u = np.linalg.solve(screen_axes(calib, x, y), np.array([p_user[0] - x, p_user[1] - y]))
                print(f"이번 어긋남 (보정 적용 후): 화면 오른쪽 {r:+.1f} 위 {u:+.1f} mm, 회전 {wrap90(yaw_user - yaw):+.1f}°")
                print(f"카메라 계산 대비 전체 차이 기록: dx {d[0]:+.1f} dy {d[1]:+.1f} mm, 각도 {d[2]:+.1f}°")
                print("→", Correction(samples).summary())
                log.update(approach_error_screen_mm=[round(float(r), 1), round(float(u), 1)],
                           approach_error_yaw_deg=round(float(wrap90(yaw_user - yaw)), 1))
            else:
                print("기록 취소")
            stage("HOME으로 (블록 위 접근 높이를 거쳐서)")
            arm.move_joints(plan.q_pre, g_open, params.joint_speed_dps)
            arm.move_joints(q_home, g_open, params.joint_speed_dps)
            log.update(result="approach_measured" if q_user is not None else "approach_only"); return
        stage("내려가기")
        arm.follow(plan.down, g_open)
        stage("집기")
        g_now = arm.set_gripper(g_closed, 1.2)
        stage("들어 올리기")
        arm.follow(plan.up, g_closed)
        g_now = arm.read()[1]
        # 블록을 잡았으면 그리퍼가 끝까지 닫히지 못한다
        held = abs(g_now - g_closed) > 3.0
        log.update(gripper_after_lift=round(g_now, 1), gripper_closed_ref=g_closed, held=held)
        print(f"그리퍼 값 {g_now:.1f} (빈손 닫힘 {g_closed:.1f}) → {'잡음' if held else '빈손'}")
        if not held:
            stage("빈손 → 그리퍼 열고 HOME")
            arm.set_gripper(g_open, 0.5)
            arm.move_joints(q_home, g_open, params.joint_speed_dps)
            log.update(result="empty"); return
        q_basket = np.array(poses["basket"]["q_deg"])
        q_lift = arm.read()[0]
        # 관절을 한꺼번에 움직이면 도중에 팔이 낮게 휘어 바구니 테두리를 칠 수 있다.
        # ① 어깨 회전은 그대로 두고 나머지 관절을 바구니 자세로(작업 영역 위에서 높이를 맞춤)
        # ② 그 높이에서 어깨만 돌려 바구니 위로. BASKET은 테두리보다 높게 기록해 둔다.
        stage("바구니 자세 높이로 (작업 영역 위에서)")
        arm.move_joints(np.r_[q_lift[0], q_basket[1:]], g_closed, params.joint_speed_dps)
        stage("어깨만 돌려 바구니 위로")
        arm.move_joints(q_basket, g_closed, params.joint_speed_dps)
        stage("놓기")
        arm.set_gripper(g_open, 0.8)
        stage("HOME으로 (어깨 먼저 돌리고 나머지)")
        arm.move_joints(np.r_[q_home[0], q_basket[1:]], g_open, params.joint_speed_dps)
        arm.move_joints(q_home, g_open, params.joint_speed_dps)
        log.update(result="placed")
    finally:
        cap.release()
        if leader:
            leader.disconnect()
        if arm:
            arm.disconnect()
        LOG.parent.mkdir(exist_ok=True)
        with LOG.open("a") as f:
            f.write(json.dumps(log, ensure_ascii=False) + "\n")
        print("기록:", log.get("result"))


if __name__ == "__main__":
    main()
