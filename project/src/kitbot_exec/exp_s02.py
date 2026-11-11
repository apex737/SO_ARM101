"""S02 최종 실험 도구 (conda lerobot, project/src에서). 줄자 없이 손목 카메라·블록 크기(25mm)·팔 모델로 잰다.

    python -m kitbot_exec.exp_s02 check                         # 팔을 움직이지 않음: 파일·카메라·검출 점검
    python -m kitbot_exec.exp_s02 table                         # 실험 1: 닫은 부리 끝으로 테이블 5곳 이상 짚기 (리더암)
    python -m kitbot_exec.exp_s02 wristref                      # 실험 2: 손목 카메라 기준 만들기 (리더암으로 블록 한 번 잡기)
    python -m kitbot_exec.exp_s02 approach --name middle_0 --repeat 3 --step   # 실험 3·5: 자동 접근 + 사진 측정
    python -m kitbot_exec.exp_s02 approach --name c01 --collect --corr none    # 실험 4: 보정 기록을 자동으로 모으기
    python -m kitbot_exec.exp_s02 straddle --name s01 --step                    # 손목 기준 확인: 블록 윗면 바로 위까지 내려가 멈춤
    python -m kitbot_exec.exp_s02 general --name 01 --step                      # 일반화 실험: 접근 오차 + 내려가 보기 + 자리 종류 (commands.md 2-3)
    python -m kitbot_exec.exp_s02 report                        # 결과 요약 → 실행 폴더의 report.json

모든 기록은 실행 폴더(--run, 기본 ../evidence/<오늘>_s02)에 trials.jsonl·table_touch.json·wrist_ref.json으로 남는다.
사진은 var/exp_s02/frames/에 저장한다(git 제외). 실험 3·5(평가)는 보정 파일을 읽기만 하고 바꾸지 않는다.
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np

from . import wrist_measure as wm
from .correction import Correction, add_sample, load_samples, wrap90
from .kinematics import ARM_JOINTS
from .pick import CALIB, PickParams, detect_block_pose, jaw_center_xy, jaw_offset_xy, plan_pick

SRC = Path(__file__).resolve().parents[1]
FRAMES = SRC / "var" / "exp_s02" / "frames"
DEFAULT_CORR = SRC / "var" / "approach_corrections.json"
WRIST_CORR = SRC / "var" / "approach_corrections_wrist.json"
SETTLE_TOL_DEG = 2.0   # Arm.move_joints 기본 멈춤 기준. 접근 직후 (목표 − 실제)가 이 이상이면 시간 초과로 끝난 것


def default_run_dir() -> Path:
    return SRC.parent / "evidence" / f"{time.strftime('%Y-%m-%d')}_s02"


def append_jsonl(path: Path, rec: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def read_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _r(v, n=2):
    return np.round(np.asarray(v, float), n).tolist()


# ---------- 카메라 ----------
def grab(cap, gains, n: int, flush: int = 5):
    """버퍼에 남은 옛 사진을 버린 뒤 n장. 팔이 막 멈췄을 때 이전 장면이 섞이지 않게."""
    from .camera import correct_wb

    for _ in range(flush):
        cap.read()
    out = []
    for _ in range(n):
        ok, f = cap.read()
        if ok:
            out.append(correct_wb(f, gains))
    return out


def save_frame(frame, name: str) -> str | None:
    if frame is None:
        return None
    import cv2

    FRAMES.mkdir(parents=True, exist_ok=True)
    p = FRAMES / f"{time.strftime('%H%M%S')}_{name}.jpg"
    cv2.imwrite(str(p), frame)
    return str(p.relative_to(SRC))


# ---------- 역기구학 (두 부리 가운데 기준) ----------
def ik_jaw(kin, x, y, z, yaw, seed, p: PickParams, z_min: float, roll_pref=None):
    """두 부리 가운데가 (x, y), 모델 그리퍼 끝 높이가 z가 되는 수직 자세. plan_pick 안의 계산과 같다."""
    off = np.zeros(2)
    r = None
    for _ in range(6):
        r = kin.ik_top_down((x - off[0], y - off[1], z), yaw, seed, z_min_mm=z_min, max_tilt_deg=p.max_tilt_deg,
                            roll_pref_deg=roll_pref)
        if not r.ok:
            return r
        new = jaw_offset_xy(kin, r.q_deg, p.jaw_center_mm)
        if np.linalg.norm(new - off) < 0.2:
            return r
        off, seed = new, r.q_deg
    return r


def line_path(kin, x, y, yaw, z0, z1, seed, p: PickParams, z_min: float, roll_pref=None):
    """높이 z0 → z1 수직 직선 경로 (3mm 간격). 관절이 15° 넘게 튀면 거부."""
    n = max(int(abs(z1 - z0) / p.line_step_mm), 1)
    path, q = [], np.asarray(seed, float)
    for z in np.linspace(z0, z1, n + 1)[1:]:
        r = ik_jaw(kin, x, y, z, yaw, q, p, z_min, roll_pref)
        if not r.ok:
            raise RuntimeError(f"높이 {z:.1f}mm: {r.reason}")
        if np.max(np.abs(r.q_deg - q)) > 15.0:
            raise RuntimeError(f"높이 {z:.1f}mm에서 관절이 튐")
        path.append(r.q_deg)
        q = r.q_deg
    per = max(int(round(p.line_step_mm / p.line_speed_mms * 30.0)), 1)
    return [qq for qq in path for _ in range(per)]


# ---------- 리더암 원격 조종 ----------
class Teleop:
    """리더암 → 팔로워. 버스를 두 스레드가 동시에 쓰지 않도록, 읽기 전에는 반드시 pause()."""

    def __init__(self, arm, leader):
        self.arm, self.leader = arm, leader
        self._stop = threading.Event()
        self._th = None
        self.names = [f"{j}.pos" for j in ARM_JOINTS]

    def leader_q(self):
        a = self.leader.action()
        return np.array([a[n] for n in self.names], float), float(a["gripper.pos"])

    def align(self, max_gap_deg: float = 15.0) -> bool:
        q_f, _ = self.arm.read()
        print(f"\n[리더암 맞추기] 리더암을 지금 팔로워 자세와 비슷하게(관절 차이 {max_gap_deg:.0f}° 안) 맞추고 Enter. 취소: x Enter")
        while True:
            if input("  > ").strip().lower() == "x":
                return False
            ql, gl = self.leader_q()
            gap = np.abs(ql - q_f)
            if gap.max() <= max_gap_deg:
                self.arm.move_joints(ql, gl, 20.0)
                return True
            print(f"  아직 {gap.max():.0f}° 차이 (가장 큰 관절: {ARM_JOINTS[int(gap.argmax())]})")

    def start(self):
        self._stop.clear()

        def run():
            while not self._stop.is_set():
                self.arm.send_action(self.leader.action())
                time.sleep(1.0 / 30)
        self._th = threading.Thread(target=run, daemon=True)
        self._th.start()

    def pause(self):
        self._stop.set()
        if self._th:
            self._th.join()
        time.sleep(0.3)  # 팔로워가 마지막 명령에서 멈출 시간


# ---------- 공통 준비 ----------
class Ctx:
    def __init__(self, args, need_arm=True, need_leader=False, need_top=False, need_wrist=False):
        from .arm import load_poses
        from .calib import PixelToRobot
        from .kinematics import SO101Kinematics

        self.args = args
        self.run = Path(args.run) if args.run else default_run_dir()
        self.run.mkdir(parents=True, exist_ok=True)
        self.p = PickParams()
        self.kin = SO101Kinematics()
        self.calib = PixelToRobot.load(CALIB)
        self.poses = load_poses()
        for need in ("home", "gripper_pregrasp", "gripper_closed"):
            if need not in self.poses:
                raise SystemExit(f"poses.json에 {need}가 없습니다. python -m kitbot_exec.teach")
        self.q_home = np.array(self.poses["home"]["q_deg"], float)
        self.g_open = float(self.poses["gripper_pregrasp"]["gripper"])
        self.g_closed = float(self.poses["gripper_closed"]["gripper"])
        self.arm = self.leader = self.top = self.wrist = None
        self.top_gains = self.wrist_gains = None
        if need_top:
            from .camera import open_role
            self.top, self.top_gains = open_role("top")
        if need_wrist:
            from .camera import open_role
            self.wrist, self.wrist_gains = open_role("wrist")
        if need_arm:
            from .arm import Arm
            self.arm = Arm()
        if need_leader:
            from .arm import Leader
            self.leader = Leader()

    def z_table(self, x, y) -> float:
        return float(self.calib.z_at([[x, y]])[0]) - self.p.block_mm

    def step(self, msg: str):
        print(f"\n[{msg}]")
        if getattr(self.args, "step", False):
            input("  Enter = 진행, Ctrl+C = 중단 ")

    def close(self):
        for cap in (self.top, self.wrist):
            if cap is not None:
                cap.release()
        if self.leader is not None:
            self.leader.disconnect()
        if self.arm is not None:
            self.arm.disconnect()


# ---------- check ----------
def cmd_check(args):
    """팔을 움직이지 않는다."""
    from .arm import load_poses
    from .camera import open_role
    from .vision import detect_blocks

    poses = load_poses()
    print("calib_top.json:", "있음" if CALIB.exists() else "없음 → collect_calib 먼저")
    for k in ("home", "gripper_pregrasp", "gripper_closed", "basket"):
        print(f"poses.json {k}:", "있음" if k in poses else "없음")
    for path in (DEFAULT_CORR, WRIST_CORR):
        print(f"{path.name}:", Correction(load_samples(path)).summary())
    print("wrist_ref.json:", "있음" if wm.REF_FILE.exists() else "없음 → 실험 2(wristref)에서 만듦")
    for role in ("top", "wrist"):
        try:
            cap, gains = open_role(role)
            frames = grab(cap, gains, 10)
            cap.release()
        except Exception as e:  # 장치 배정이 바뀐 경우 등
            print(f"카메라 {role}: 열기 실패 ({e})")
            continue
        if role == "top":
            counts = [len(detect_blocks(f, args.color)) for f in frames]
            print(f"카메라 top: {len(frames)}장, '{args.color}' 블록 개수 {counts} (모두 1이어야 함)")
        else:
            obs = wm.observe(frames, args.color)
            print(f"카메라 wrist: {len(frames)}장, 블록", obs.as_dict() if obs else "못 찾음 (팔이 HOME이면 정상일 수 있음)")


# ---------- 실험 1: 테이블 짚기 ----------
def cmd_table(args):
    ctx = Ctx(args, need_leader=True)
    tele = Teleop(ctx.arm, ctx.leader)
    pts = []
    try:
        if not tele.align():
            return
        print("\n[테이블 짚기] 부리를 닫고 수직에 가깝게 세워, 끝을 테이블에 살짝 대세요.")
        print("  가까이·중간·멀리·왼쪽·오른쪽 등 5곳 이상. 댄 상태에서 Enter = 기록, q Enter = 끝")
        tele.start()
        while True:
            cmd = input(f"  ({len(pts)}곳 기록됨) > ").strip().lower()
            tele.pause()
            if cmd == "q":
                break
            q, g = ctx.arm.read()
            tip, R = ctx.kin.fk(q)
            tilt = float(np.rad2deg(np.arccos(np.clip(-R[2, 2], -1, 1))))
            rec = {"q_deg": _r(q), "tip_mm": _r(tip, 1), "tilt_deg": round(tilt, 1),
                   "calib_table_z": round(ctx.z_table(tip[0], tip[1]), 1)}
            pts.append(rec)
            print(f"    끝 위치 (모델) {rec['tip_mm']} mm, 기울기 {tilt:.1f}°, 보정 평면이 예상한 테이블 z {rec['calib_table_z']}")
            if tilt > 10:
                print("    기울기가 10° 넘음: 끝이 아닌 옆면이 닿았을 수 있음. 다시 세워 기록하길 권함")
            tele.start()
    finally:
        tele.pause()
        if pts:
            out = {"points": pts, "summary": wm.table_summary(pts)}
            (ctx.run / "table_touch.json").write_text(json.dumps(out, ensure_ascii=False, indent=1))
            print("\n요약:", json.dumps(out["summary"], ensure_ascii=False))
            print("저장:", ctx.run / "table_touch.json")
        ctx.close()


# ---------- 실험 2: 손목 기준 ----------
def cmd_wristref(args):
    ctx = Ctx(args, need_leader=True, need_wrist=True)
    tele = Teleop(ctx.arm, ctx.leader)
    p, kin, arm = ctx.p, ctx.kin, ctx.arm
    try:
        if not tele.align():
            return
        print("\n[블록 잡기] 리더암으로 블록을 부리 손가락 가운데로 잡고, 블록이 테이블에 닿은 채로 두세요.")
        print("  그리퍼는 수직에 가깝게. 됐으면 Enter (프로그램이 그리퍼를 열고 수직으로 올라갑니다). 취소: x Enter")
        tele.start()
        cmd = input("  > ").strip().lower()
        tele.pause()
        if cmd == "x":
            return
        q0, g_block = arm.read()
        tip0, R0 = kin.fk(q0)
        x, y = jaw_center_xy(kin, q0, p.jaw_center_mm)
        yaw0 = kin._tool_yaw(q0)
        z_top = float(ctx.calib.z_at([[x, y]])[0])
        z_min = ctx.z_table(x, y) + p.table_margin_mm
        # 지금 실제로 블록을 쥐고 있는 자세는 안전하다. 모델 높이가 위치마다 몇 mm 다르므로(테이블 짚기)
        # 시작 높이가 예상 하한보다 낮으면 하한을 시작 높이 아래 1mm로 내린다. 여기서는 위로만 움직인다.
        if tip0[2] - 1.0 < z_min:
            print(f"  시작 높이 {tip0[2]:.1f}mm가 예상 하한 {z_min:.1f}mm보다 낮음 → 하한을 {tip0[2] - 1.0:.1f}mm로 (위로만 움직임)")
            z_min = float(tip0[2]) - 1.0
        z_pre = z_top + p.approach_mm
        span = g_block - ctx.g_closed
        open_est = p.block_mm * (ctx.g_open - ctx.g_closed) / span if abs(span) > 1e-6 else float("nan")
        print(f"  블록 쥔 그리퍼 값 {g_block:.1f} → 집기 직전 벌림 추정 {open_est:.1f}mm "
              f"(닫힘=0mm·선형 가정, 추정)")
        roll = float(q0[4])

        ctx.step("그리퍼 열기 (블록은 테이블에 남음)")
        arm.set_gripper(ctx.g_open, 1.0)
        ctx.step(f"수직으로 올라가기 (모델 높이 {tip0[2]:.0f} → {z_pre:.0f}mm)")
        r = ik_jaw(kin, x, y, tip0[2], yaw0, q0, p, z_min, roll)
        if not r.ok:
            raise SystemExit(f"시작 자세 계산 실패: {r.reason}")
        arm.move_joints(r.q_deg, ctx.g_open, 10.0)
        arm.follow(line_path(kin, x, y, yaw0, tip0[2], z_pre, r.q_deg, p, z_min, roll), ctx.g_open)

        def at(dx_tool=0.0, dy_tool=0.0, dyaw=0.0, dz=0.0, label=""):
            """기준 자세에서 tool 좌표로 옮긴 자세로 가서 사진·관절값을 얻는다."""
            d_rob = wm.tool_to_robot(R0, [dx_tool, dy_tool])
            rr = ik_jaw(kin, x + d_rob[0], y + d_rob[1], z_pre + dz, yaw0 + dyaw, q_c, p, z_min, roll)
            if not rr.ok:
                raise SystemExit(f"{label} 자세 계산 실패: {rr.reason}")
            arm.move_joints(rr.q_deg, ctx.g_open, 10.0)
            time.sleep(0.5)
            qq, _ = arm.read()
            frames = grab(ctx.wrist, ctx.wrist_gains, 12)
            obs = wm.observe(frames, args.color)
            if obs is None:
                save_frame(frames[-1] if frames else None, f"wristref_fail_{label}")
                raise SystemExit(f"{label}: 손목 사진에서 블록을 못 찾음. view_blocks --role wrist로 색·면적 확인")
            shots[label] = {"px": _r(obs.center_px, 1), "size_px": round(obs.size_px, 1),
                            "angle_deg": round(obs.angle_deg, 1), "frame": save_frame(frames[-1], f"wristref_{label}")}
            return qq, obs, frames[-1]

        shots = {}

        q_c = arm.read()[0]
        ctx.step("기준 사진 (접근 높이)")
        q_c, ref_obs, frame = at(label="center")
        print("  기준:", ref_obs.as_dict())
        jaw_c = jaw_center_xy(kin, q_c, p.jaw_center_mm)
        _, Rc = kin.fk(q_c)
        moves, dpx = [], []
        ctx.step("mm/픽셀 재기: 그리퍼 좌표로 ±10mm 네 번 움직임")
        for dx, dy in ((10, 0), (-10, 0), (0, 10), (0, -10)):
            qq, obs, _ = at(dx, dy, label=f"move{dx:+d}_{dy:+d}")
            moves.append(wm.robot_to_tool(Rc, jaw_center_xy(kin, qq, p.jaw_center_mm) - jaw_c))  # 실제 이동(모델)
            dpx.append(np.asarray(obs.center_px) - np.asarray(ref_obs.center_px))
        J, rms = wm.fit_J(moves, dpx)
        resid = np.asarray(dpx) - np.asarray(moves) @ J.T
        print("  이동별: 그리퍼 이동(mm) → 블록 픽셀 이동, 직선에서 벗어난 양(px)")
        for mv, dp, rs in zip(moves, dpx, resid):
            print(f"    ({mv[0]:+6.1f},{mv[1]:+6.1f}) → ({dp[0]:+6.1f},{dp[1]:+6.1f})  벗어남 {np.linalg.norm(rs):.1f}")
        ctx.step("각도 민감도: 손목 +10°")
        qq, obs_yaw, _ = at(dyaw=10.0, label="yaw+10")
        k_yaw = wm.yaw_gain(ref_obs.angle_deg, obs_yaw.angle_deg, kin._tool_yaw(q_c), kin._tool_yaw(qq))
        ctx.step("거리 재기: 20mm 위")
        qq, obs_up, _ = at(dz=20.0, label="up20")
        dz_real = float(kin.fk(qq)[0][2] - kin.fk(q_c)[0][2])
        d_ref = wm.d_from_two_heights(ref_obs.size_px, obs_up.size_px, dz_real)
        _, obs_back, _ = at(label="back")
        back_px = float(np.linalg.norm(np.asarray(obs_back.center_px) - np.asarray(ref_obs.center_px)))
        print(f"  기준 자리로 돌아왔을 때 블록 픽셀 차이 {back_px:.1f}px (크면 서보 유격·블록 밀림)")
        ref = wm.WristRef(ref_obs.center_px, ref_obs.angle_deg, ref_obs.size_px, np.round(J, 4).tolist(),
                          round(k_yaw, 3), round(d_ref, 1),
                          {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "J_fit_rms_px": round(rms, 2),
                           "block_xy_model": _r([x, y], 1), "approach_mm": p.approach_mm,
                           "gripper_open_est_mm": round(open_est, 1), "gripper_block_value": round(g_block, 1),
                           "moves_tool_mm": _r(moves, 2), "moves_px": _r(dpx, 1), "resid_px": _r(resid, 1),
                           "back_minus_ref_px": round(back_px, 1), "shots": shots,
                           "frame": save_frame(frame, "wristref")})
        if wm.REF_FILE.exists():
            old = wm.WristRef.load()
            d_mm = np.linalg.solve(np.asarray(J), np.asarray(ref.c_ref) - np.asarray(old.c_ref))
            print(f"  이전 기준 대비: 중심 {np.linalg.norm(d_mm):.1f}mm, 각도 {float(wrap90(ref.angle_ref - old.angle_ref)):+.1f}°, "
                  f"크기 {100 * (ref.size_ref / old.size_ref - 1):+.1f}% (2mm·2°·3% 넘으면 블록을 잡은 위치가 달랐을 수 있음)")
        ref.save()
        (ctx.run / f"wrist_ref_{time.strftime('%H%M%S')}.json").write_text(
            json.dumps(ref.__dict__, ensure_ascii=False, indent=1, default=list))
        (ctx.run / "wrist_ref.json").write_text(json.dumps(ref.__dict__, ensure_ascii=False, indent=1, default=list))
        px_per_mm = float(np.mean(np.linalg.svd(np.asarray(J))[1]))
        print(f"\n저장: {wm.REF_FILE}  (복사: {ctx.run / 'wrist_ref.json'})")
        print(f"  mm당 픽셀 ≈ {px_per_mm:.2f}, J 맞춤 잔차 {rms:.2f}px, 각도 민감도 k {k_yaw:+.2f}, "
              f"카메라-블록 거리 {d_ref:.0f}mm")
        warn = []
        if rms > 3.0:
            warn.append("J 잔차가 3px 넘음 (팔이 흔들렸거나 블록 검출 불안정)")
        if not 0.7 <= abs(k_yaw) <= 1.3:
            warn.append("각도 민감도 |k|가 0.7~1.3 밖 (검출 각도 불안정 가능)")
        if not 20 <= d_ref <= 300:
            warn.append("카메라-블록 거리가 20~300mm 밖 (크기 측정 불안정 가능)")
        print("  점검:", "; ".join(warn) if warn else "이상 없음")
    finally:
        tele.pause()
        try:
            arm.move_joints(ctx.q_home, ctx.g_open, p.joint_speed_dps)
        except Exception as e:
            print("HOME 복귀 실패:", e)
        ctx.close()


# ---------- 실험 3·4·5: 자동 접근 + 사진 측정 ----------
def approach_trial(ctx, name: str, corr: Correction, ref: wm.WristRef | None, collect_path: Path | None,
                   low_clear_mm: float | None = None, ask: bool = False) -> dict:
    """한 시행: HOME → 위 카메라 검출 → (보정) → 계획 → 접근 → 진단·손목 사진 → HOME. 기록 dict를 돌려준다."""
    p, kin, arm = ctx.p, ctx.kin, ctx.arm
    rec = {"name": name, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "color": ctx.args.color,
           "correction": corr.mode, "correction_n": corr.n, "result": "started"}
    ctx.step("HOME")
    arm.move_joints(ctx.q_home, ctx.g_open, p.joint_speed_dps)
    grab(ctx.top, ctx.top_gains, 0)  # 버퍼 비우기
    pose, frame, note = detect_block_pose(ctx.top, ctx.top_gains, ctx.args.color, ctx.calib)
    rec["top_note"] = note
    if pose is None:
        rec["result"] = "no_block"
        return rec
    x0, y0, z_top, yaw0 = pose
    x, y, yaw = corr.apply(x0, y0, yaw0)
    rec.update(block_camera=_r([x0, y0, z_top, yaw0], 1), target_xy=_r([x, y], 1), target_yaw=round(yaw, 1))
    plan = plan_pick(kin, x, y, z_top, yaw, ctx.q_home, p)
    rec["plan_info"] = plan.info
    if not plan.ok:
        rec.update(result="plan_failed", plan_reason=plan.reason)
        return rec
    ctx.step(f"블록 위 {p.approach_mm:.0f}mm로 접근")
    arm.move_joints(plan.q_pre, ctx.g_open, p.joint_speed_dps)
    q_read, g_read = arm.read()
    q_t = np.asarray(plan.q_pre, float)
    dq = q_t - q_read
    tip_t, _ = kin.fk(q_t)
    tip_r, R = kin.fk(q_read)
    jaw_r = jaw_center_xy(kin, q_read, p.jaw_center_mm)
    rec["diag"] = {
        "target_minus_read_deg": _r(dq), "max_abs_deg": round(float(np.max(np.abs(dq))), 2),
        "settle_timeout": bool(np.max(np.abs(dq)) >= SETTLE_TOL_DEG),
        "jaw_target_minus_read_xy_mm_model": _r(jaw_center_xy(kin, q_t, p.jaw_center_mm) - jaw_r, 1),
        "tip_target_minus_read_z_mm_model": round(float(tip_t[2] - tip_r[2]), 1),
        "gripper_read": round(float(g_read), 1), "gripper_target": round(ctx.g_open, 1)}
    time.sleep(0.5)
    frames = grab(ctx.wrist, ctx.wrist_gains, 12)
    obs = wm.observe(frames, ctx.args.color)
    rec["wrist_frame"] = save_frame(frames[-1] if frames else None, name)
    rec["result"] = "approached"
    if obs is not None:
        w = {"obs": obs.as_dict()}
        if ref is not None:
            e_tool = wm.offset_tool_mm(obs, ref)
            e_rob = wm.tool_to_robot(R, e_tool)               # 블록 − 부리 가운데 (실제, 사진 기준)
            phi = wm.offset_yaw_deg(obs, ref)                 # 블록 − 그리퍼 각도
            block_true = jaw_r + e_rob
            w.update(err_tool_mm=_r(e_tool, 1), err_robot_mm=_r(e_rob, 1), err_mm=round(float(np.linalg.norm(e_rob)), 1),
                     err_yaw_deg=round(phi, 1), height_offset_mm=round(wm.height_offset_mm(obs, ref), 1),
                     block_true_xy_model=_r(block_true, 1),
                     perception_err_xy_mm=_r(block_true - np.array([x, y]), 1))  # 목표가 블록에서 벗어난 양(카메라·보정 쪽)
            if collect_path is not None:
                # 명령값 기준으로 기록한다: "이 위치를 명령했더니 블록이 e만큼 옆에 있었다" → 다음엔 명령 + e.
                # 서보가 덜 가는 양까지 보정에 들어가므로, 실행 단계 서보 보상(arm.py)을 나중에 넣으면 다시 모아야 한다.
                jaw_t = jaw_center_xy(kin, q_t, p.jaw_center_mm)
                cmd_xy = jaw_t + e_rob
                add_sample((x0, y0, yaw0), (cmd_xy[0], cmd_xy[1], float(wrap90(yaw + phi))),
                           {"source": "wrist", "basis": "command", "trial": name,
                            "block_true_read_xy": _r(block_true, 1)}, path=collect_path)
                w["collected_to"] = collect_path.name
        rec["wrist"] = w
    if low_clear_mm is not None:
        rec["low"] = descend_check(ctx, plan, x, y, z_top, yaw, low_clear_mm, name, ask, skip_ok=True)
    ctx.step("HOME (접근 높이를 거쳐서)")
    arm.move_joints(ctx.q_home, ctx.g_open, p.joint_speed_dps)
    return rec


def print_trial(rec: dict):
    print(f"\n=== {rec['name']}: {rec['result']}")
    if "diag" in rec:
        d = rec["diag"]
        print(f"  관절 (목표 − 실제) {d['target_minus_read_deg']}°  최대 {d['max_abs_deg']}°"
              f"{'  ← 멈춤 시간 초과' if d['settle_timeout'] else ''}")
        print(f"  그로 인한 부리 가운데 차이 (모델) {d['jaw_target_minus_read_xy_mm_model']} mm, 높이 {d['tip_target_minus_read_z_mm_model']} mm")
    w = rec.get("wrist")
    if w is None and rec["result"] == "approached":
        print("  손목 사진: 블록을 못 찾음 (가려짐·시야 밖·색 범위) → 사진 확인:", rec.get("wrist_frame"))
    elif w and "err_mm" in w:
        print(f"  접근 오차 (손목 사진): {w['err_mm']} mm  (로봇 x, y {w['err_robot_mm']}), 각도 {w['err_yaw_deg']:+}°, "
              f"높이 {w['height_offset_mm']:+} mm (기준 대비)")
        print(f"  그중 카메라·보정 쪽 (모델) {w['perception_err_xy_mm']} mm")
    elif w:
        print("  손목 사진 블록:", w["obs"], "(wrist_ref 없음 → mm 계산 안 함)")


def cmd_approach(args):
    ctx = Ctx(args, need_top=True, need_wrist=True)
    if args.corr == "none":
        corr = Correction([])
    else:
        corr = Correction(load_samples(Path(args.corr) if args.corr else DEFAULT_CORR))
    print(corr.summary(), "(읽기만 함)" if not args.collect else "")
    ref = wm.WristRef.load() if wm.REF_FILE.exists() else None
    if ref is None:
        print("wrist_ref.json 없음: 진단·사진만 남기고 mm 오차는 계산하지 않습니다 (실험 2 먼저 권장)")
    collect_path = WRIST_CORR if args.collect else None
    if args.collect and ref is None:
        raise SystemExit("--collect에는 wrist_ref.json이 필요합니다 (실험 2)")
    try:
        for i in range(1, args.repeat + 1):
            name = f"{args.name}_r{i}" if args.repeat > 1 else args.name
            if i > 1:
                input(f"\n블록을 그대로 두고 Enter (다음: {name}) ")
            rec = approach_trial(ctx, name, corr, ref, collect_path)
            append_jsonl(ctx.run / "trials.jsonl", rec)
            print_trial(rec)
    finally:
        ctx.close()
    print("\n기록:", ctx.run / "trials.jsonl")


# ---------- 블록 바로 위까지 내려가기 (straddle·general 공통) ----------
JAW_PROMPT = ("  블록에 더 가까운 손가락:  f = 고정(빨강),  m = 움직이는(검정),  c = 비슷함(가운데)\n"
              "  뒤에 대략 틈을 mm로 적어도 됨: 가까운 쪽 틈, 먼 쪽 틈. 예) m 1 15  (검정 쪽 1mm, 빨강 쪽 15mm)")


def descend_check(ctx, plan, x, y, z_top, yaw, clear_mm: float, name: str, ask: bool, skip_ok: bool = False) -> dict:
    """접근 자세(plan.q_pre)에서 벌린 부리 끝이 블록 윗면 위 clear_mm(명령 기준)까지 수직으로 내려가
    관절 진단·손목 사진(·사람 판정)을 남기고, 같은 길로 접근 높이까지 올라온다. 블록 윗면 아래로는 내려가지 않는다."""
    p, kin, arm = ctx.p, ctx.kin, ctx.arm
    roll = float(plan.q_pre[4])
    z_pre = float(kin.fk(plan.q_pre)[0][2])
    z_min = ctx.z_table(x, y) + p.table_margin_mm
    path = line_path(kin, x, y, yaw, z_pre, z_top + clear_mm, plan.q_pre, p, z_min, roll)
    out = {"clear_cmd_mm": clear_mm}
    ctx.step(f"수직으로 내려가기: 부리 끝이 블록 윗면 위 {clear_mm:.0f}mm (명령 기준, 실제는 몇 mm 더 낮을 수 있음)")
    arm.follow(path, ctx.g_open)
    try:
        time.sleep(0.5)
        q_read, _ = arm.read()
        q_t = np.asarray(path[-1], float)
        tip_t, _ = kin.fk(q_t)
        tip_r, _ = kin.fk(q_read)
        out["diag_low"] = {
            "target_minus_read_deg": _r(q_t - q_read),
            "jaw_target_minus_read_xy_mm_model": _r(jaw_center_xy(kin, q_t, p.jaw_center_mm)
                                                    - jaw_center_xy(kin, q_read, p.jaw_center_mm), 1),
            "tip_z_cmd_mm": round(float(tip_t[2]), 1), "tip_z_read_mm": round(float(tip_r[2]), 1),
            "block_top_z_mm": round(float(z_top), 1),
            "clear_read_mm_model": round(float(tip_r[2] - z_top), 1)}
        frames = grab(ctx.wrist, ctx.wrist_gains, 12)
        out["wrist_frame_low"] = save_frame(frames[-1] if frames else None, f"low_{name}")
        obs = wm.observe(frames, ctx.args.color)
        if obs is not None:
            out["wrist_obs_low"] = obs.as_dict()
        d = out["diag_low"]
        print(f"  [내려간 자리] 관절 (목표 − 실제) {d['target_minus_read_deg']}°, 부리 끝 높이 명령 {d['tip_z_cmd_mm']} → "
              f"실제(모델) {d['tip_z_read_mm']} mm, 블록 윗면 {d['block_top_z_mm']} mm → 틈(모델) {d['clear_read_mm_model']} mm")
        print("  손목 사진:", out["wrist_frame_low"])
        if ask:
            print("\n[눈으로 확인] 두 손가락이 나란히 보이는 쪽(그리퍼 넓은 면 정면)에서 보세요.")
            print(JAW_PROMPT)
            if skip_ok:
                print("  Enter만 누르면 눈 판정 없이 넘어감")
            out["human"] = input("  > ").strip()
    finally:
        ctx.step("다시 접근 높이로 올라가기")
        arm.follow(path[::-1], ctx.g_open)
    return out


def place_info(calib, x, y) -> dict:
    """블록 자리가 평가 격자 안 / 보정점 범위 안 / 밖 중 어디인지, 가장 가까운 격자·보정점까지 거리."""
    import cv2

    from .view_blocks import S02_GRID
    g = np.array(list(S02_GRID.values()), float)
    in_grid = g[:, 0].min() <= x <= g[:, 0].max() and g[:, 1].min() <= y <= g[:, 1].max()
    out = {"nearest_grid_mm": round(float(np.min(np.linalg.norm(g - [x, y], axis=1))), 1)}
    in_calib = None
    if calib.pairs_mm is not None and len(calib.pairs_mm) >= 3:
        c = np.asarray(calib.pairs_mm, float)[:, :2]
        hull = cv2.convexHull(c.astype(np.float32))
        in_calib = cv2.pointPolygonTest(hull, (float(x), float(y)), False) >= 0
        out["nearest_calib_mm"] = round(float(np.min(np.linalg.norm(c - [x, y], axis=1))), 1)
    out["region"] = "grid" if in_grid else ("calib" if in_calib else ("outside" if in_calib is False else "unknown"))
    return out


REGION_KO = {"grid": "평가 격자 안", "calib": "격자 밖·보정점 범위 안", "outside": "보정점 범위 밖(추정 범위 밖)",
             "unknown": "범위 모름(보정점 없음)"}


def cmd_straddle(args):
    """보정 없이 접근 → 벌린 부리 끝이 블록 윗면 바로 위(명령 기준 --clear mm)까지 수직으로 내려가 멈춤.
    사람은 앞에서 두 손가락과 블록 사이 틈을 보고, 손목 사진은 부리 끝과 블록을 거의 같은 거리에서 함께 찍는다.
    블록에 닿지 않는 높이까지만 내려간다. 서보가 약 6~7mm 덜 올라가(처져) 있으므로 실제 틈은 명령보다 작다."""
    ctx = Ctx(args, need_top=True, need_wrist=True)
    p, kin, arm = ctx.p, ctx.kin, ctx.arm
    ref = wm.WristRef.load() if wm.REF_FILE.exists() else None
    rec = {"name": args.name, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "kind": "straddle",
           "clear_cmd_mm": args.clear, "result": "started"}
    try:
        ctx.step("HOME")
        arm.move_joints(ctx.q_home, ctx.g_open, p.joint_speed_dps)
        grab(ctx.top, ctx.top_gains, 0)
        pose, _, note = detect_block_pose(ctx.top, ctx.top_gains, args.color, ctx.calib)
        rec["top_note"] = note
        if pose is None:
            rec["result"] = "no_block"
            print("블록을 못 찾음:", note)
            return
        x, y, z_top, yaw = pose
        rec.update(block_camera=_r([x, y, z_top, yaw], 1))
        plan = plan_pick(kin, x, y, z_top, yaw, ctx.q_home, p)
        if not plan.ok:
            rec.update(result="plan_failed", plan_reason=plan.reason)
            print("계획 실패:", plan.reason)
            return
        ctx.step(f"블록 위 {p.approach_mm:.0f}mm로 접근 (보정 없음)")
        arm.move_joints(plan.q_pre, ctx.g_open, p.joint_speed_dps)
        low = descend_check(ctx, plan, x, y, z_top, yaw, args.clear, args.name, ask=True)
        rec.update(diag_low=low["diag_low"], wrist_frame=low["wrist_frame_low"], human=low.get("human"))
        if "wrist_obs_low" in low:
            rec["wrist_obs"] = low["wrist_obs_low"]
        rec["result"] = "checked"
        if ref is not None:
            rec["wrist_ref_time"] = ref.meta.get("time")
    finally:
        try:
            arm.move_joints(ctx.q_home, ctx.g_open, p.joint_speed_dps)
        except Exception as e:
            print("복귀 실패:", e)
        append_jsonl(ctx.run / "straddle.jsonl", rec)
        print("기록:", ctx.run / "straddle.jsonl")
        ctx.close()


# ---------- 일반화 실험: 처음 보는 자리에서 접근 오차 + 내려가 보기 ----------
def cmd_general(args):
    """블록을 아무 데나(평가 격자 밖 포함) 두고: 보정 없는 접근 → 손목 사진으로 오차 자동 측정 →
    블록 바로 위까지 내려가 (선택) 눈 판정 → HOME. 자리 종류(격자 안/보정점 범위 안/밖)를 함께 기록한다.
    기록: trials.jsonl (이름 gen_로 시작), 요약: report --prefix gen_"""
    ctx = Ctx(args, need_top=True, need_wrist=True)
    ref = wm.WristRef.load() if wm.REF_FILE.exists() else None
    if ref is None:
        print("wrist_ref.json 없음: 손목 사진 mm 오차 없이 진단·사진·눈 판정만 남깁니다")
    name = args.name if args.name.startswith("gen_") else f"gen_{args.name}"
    try:
        rec = approach_trial(ctx, name, Correction([]), ref, None,
                             low_clear_mm=None if args.no_descend else args.clear, ask=not args.no_ask)
        rec["kind"] = "general"
        if "block_camera" in rec:
            rec.update(place_info(ctx.calib, rec["block_camera"][0], rec["block_camera"][1]))
        append_jsonl(ctx.run / "trials.jsonl", rec)
        print_trial(rec)
        if "region" in rec:
            print(f"  자리: {REGION_KO[rec['region']]}, 가장 가까운 평가 격자 {rec['nearest_grid_mm']}mm"
                  + (f", 가장 가까운 보정점 {rec['nearest_calib_mm']}mm" if "nearest_calib_mm" in rec else ""))
            if rec["nearest_grid_mm"] < 20:
                print("  주의: 평가 격자 원에서 20mm 안 → 처음 보는 자리로 치지 않음(요약에서는 그대로 셈)")
    finally:
        ctx.close()
    print("\n기록:", ctx.run / "trials.jsonl", " 요약: s02 report --prefix gen_")


# ---------- 요약 ----------
def cmd_report(args):
    run = Path(args.run) if args.run else default_run_dir()
    trials = read_jsonl(run / "trials.jsonl")
    sel = [t for t in trials if not args.prefix or t["name"].startswith(tuple(args.prefix))]
    out = {"run": str(run), "filter": args.prefix, "approach": wm.summarize(sel, args.tol_mm, args.tol_yaw)}
    regions = sorted({t["region"] for t in sel if "region" in t})
    if regions:
        out["by_region"] = {r: wm.summarize([t for t in sel if t.get("region") == r], args.tol_mm, args.tol_yaw)
                            for r in regions}
        out["human_low"] = [[t["name"], t.get("region"), t["low"].get("human")] for t in sel if t.get("low")]
    tt = run / "table_touch.json"
    if tt.exists():
        out["table_touch"] = json.loads(tt.read_text())["summary"]
    wr = run / "wrist_ref.json"
    if wr.exists():
        out["wrist_ref"] = {k: v for k, v in json.loads(wr.read_text()).items() if k != "meta"}
    run.mkdir(parents=True, exist_ok=True)
    name = "report" + ("_" + "_".join(args.prefix) if args.prefix else "") + ".json"
    (run / name).write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print(json.dumps(out, ensure_ascii=False, indent=1))
    print("저장:", run / name)


def main(argv=None):
    ap = argparse.ArgumentParser(description="S02 최종 실험 도구")
    ap.add_argument("--run", help="기록 폴더 (기본 ../evidence/<오늘>_s02)")
    ap.add_argument("--color", default="purple")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    sub.add_parser("table")
    w = sub.add_parser("wristref")
    w.add_argument("--step", action="store_true")
    a = sub.add_parser("approach")
    a.add_argument("--name", required=True, help="시행 이름 (영문·숫자·밑줄). 같은 위치 반복은 --repeat")
    a.add_argument("--repeat", type=int, default=1)
    a.add_argument("--step", action="store_true")
    a.add_argument("--corr", help="보정 파일 경로, 또는 none (기본 var/approach_corrections.json)")
    a.add_argument("--collect", action="store_true", help="손목 사진으로 잰 실제 위치를 var/approach_corrections_wrist.json에 추가")
    st = sub.add_parser("straddle", help="손목 기준 확인: 보정 없이 접근 후 블록 윗면 바로 위까지 내려가 멈춤")
    st.add_argument("--name", required=True)
    st.add_argument("--clear", type=float, default=15.0,
                    help="명령 기준 부리 끝 ~ 블록 윗면 높이 [mm]. 서보 처짐(약 6~7mm) 때문에 실제는 더 낮음. 기본 15")
    st.add_argument("--step", action="store_true")
    ge = sub.add_parser("general", help="일반화 실험: 보정 없는 접근 + 손목 사진 오차 + 블록 바로 위까지 내려가 보기")
    ge.add_argument("--name", required=True, help="시행 이름 (앞에 gen_이 붙음). 예: 01")
    ge.add_argument("--clear", type=float, default=15.0, help="내려갈 높이: 블록 윗면 위 [mm, 명령 기준]. 기본 15")
    ge.add_argument("--no-descend", action="store_true", help="내려가지 않고 접근 높이에서 측정만")
    ge.add_argument("--no-ask", action="store_true", help="눈 판정을 묻지 않음")
    ge.add_argument("--step", action="store_true")
    r = sub.add_parser("report")
    r.add_argument("--prefix", nargs="*", help="이 이름으로 시작하는 시행만 (예: grid)")
    r.add_argument("--tol-mm", type=float, default=10.0)
    r.add_argument("--tol-yaw", type=float, default=10.0)
    args = ap.parse_args(argv)
    {"check": cmd_check, "table": cmd_table, "wristref": cmd_wristref,
     "approach": cmd_approach, "straddle": cmd_straddle, "general": cmd_general, "report": cmd_report}[args.cmd](args)


if __name__ == "__main__":
    main()
