"""ACT 시연을 여러 에피소드 이어서 녹화한다 (S05-1). LeRobot 데이터셋으로 저장 → lerobot-train으로 바로 학습.

    cd project/src
    python -m kitbot_exec.record_act --repo-id local/kitbot_act_v1 --episodes 10
    python -m kitbot_exec.record_act --repo-id local/kitbot_act_v1 --goal 50 --pause-every 25  # 50개까지(25개마다 쉼), 끊겨도 같은 명령으로 이어서

에피소드 하나:
  ① 블록을 새 자리에 놓고 Enter
  ② HOME → 위 카메라로 블록 찾기 → (흔든) 목표로 40mm 접근 → 블록 윗면 위 15mm까지 수직 하강   (approach.py와 같은 계산)
  ③ 인계 판단: 관절 오차 ≤ --max-arrival-deg, 손목 카메라에 블록이 보임. 아니면 이 에피소드를 건너뜀
  ④ 리더암 자동 맞춤(리더 서보 힘 40%로 팔로워 자세까지 이동 → 리더암을 잡고 Enter → 힘 풀림)
     → 손 맞춤: 관절별 차이를 실시간으로 보며 리더암을 직접 맞춤(팔로워는 그대로). 모두 --start-gap 안이면 Enter = 녹화 시작
  ⑤ 녹화: 리더암으로 보정 → 집기 → 바구니 → HOME.  n(또는 →) = 끝내고 저장, r(또는 ←) = 버리고 다시, q(또는 Esc) = 저장하고 전체 종료(한 번 더 물음)
     --episode-s 초가 지나면 자동으로 끝내고 저장
  ⑥ 팔로워를 HOME으로 (녹화 밖)

기록: 데이터셋(~/.cache/huggingface/lerobot/<repo-id>/), 에피소드별 시작 조건 <데이터셋>/kitbot_episode_meta.jsonl
카메라 영상은 위·손목 모두 camera_settings.json의 고정값·색 보정을 적용한 RGB 640×480 30fps (실행 때도 같은 처리를 써야 한다).
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np

from .approach import align_leader, approach_block, build_correction
from .kinematics import ARM_JOINTS

FPS = 30
WIDTH, HEIGHT = 640, 480
NAMES = [f"{j}.pos" for j in list(ARM_JOINTS) + ["gripper"]]
TASK = "pick the block under the gripper and put it in the basket"


# ---------- 카메라: 항상 최신 사진 ----------
class LatestFrame:
    """카메라를 별도 스레드에서 계속 읽어 가장 최근 사진만 둔다. 녹화 루프가 카메라를 기다리지 않게."""

    def __init__(self, cap, gains=None):
        self.cap, self.gains = cap, gains
        self._lock = threading.Lock()
        self._frame, self._count, self._t = None, 0, 0.0
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self):
        while not self._stop.is_set():
            ok, f = self.cap.read()
            if ok and f is not None:
                with self._lock:
                    self._frame, self._count, self._t = f, self._count + 1, time.monotonic()

    def read(self, timeout_s=1.0):
        """cv2.VideoCapture.read()처럼: 새 사진이 올 때까지 기다려 (ok, 원본 BGR)를 돌려준다."""
        with self._lock:
            c0 = self._count
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            with self._lock:
                if self._count > c0:
                    return True, self._frame.copy()
            time.sleep(0.003)
        return False, None

    def latest_rgb(self):
        """녹화용: 가장 최근 사진에 색 보정을 하고 RGB로. (사진, 나이 초)"""
        import cv2

        from .camera import correct_wb

        with self._lock:
            f, t = self._frame, self._t
        if f is None:
            return None, np.inf
        return cv2.cvtColor(correct_wb(f, self.gains), cv2.COLOR_BGR2RGB), time.monotonic() - t

    def close(self):
        self._stop.set()
        self._th.join(timeout=1.0)
        self.cap.release()


# ---------- 데이터셋 ----------
def dataset_features() -> dict:
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.feature_utils import hw_to_dataset_features

    joints = {n: float for n in NAMES}
    cams = {"wrist": (HEIGHT, WIDTH, 3), "top": (HEIGHT, WIDTH, 3)}
    return {**hw_to_dataset_features(joints, ACTION), **hw_to_dataset_features({**joints, **cams}, OBS_STR)}


def open_dataset(repo_id: str, root: str | None, resume: bool, n_cams: int = 2):
    from lerobot.datasets import LeRobotDataset

    root = str(dataset_root(repo_id, root))   # LeRobot 0.6 resume()은 root=None을 거부한다

    kw = dict(image_writer_threads=4 * n_cams)
    if resume:
        return LeRobotDataset.resume(repo_id, root=root, **kw)
    return LeRobotDataset.create(repo_id, FPS, dataset_features(), root=root, robot_type="so101_follower",
                                 use_videos=True, **kw)


def make_frame(features: dict, q, g, action: dict, wrist_rgb, top_rgb) -> dict:
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.feature_utils import build_dataset_frame

    obs = {n: float(v) for n, v in zip(NAMES, list(q) + [g])}
    obs.update(wrist=wrist_rgb, top=top_rgb)
    return {**build_dataset_frame(features, obs, OBS_STR),
            **build_dataset_frame(features, {n: float(action[n]) for n in NAMES}, ACTION), "task": TASK}


# ---------- 녹화 루프 ----------
def record_episode(arm, leader, wrist: LatestFrame, top: LatestFrame, dataset, events: dict, episode_s: float,
                   hz: float = FPS):
    """리더 → 팔로워 원격 조종을 하며 프레임을 쌓는다. (프레임 수, 평균 Hz, 최대 사진 나이 초)."""
    from lerobot.utils.robot_utils import precise_sleep

    n, t0, max_age, clipped = 0, time.perf_counter(), 0.0, 0
    period = 1.0 / hz
    while time.perf_counter() - t0 < episode_s:
        t_loop = time.perf_counter()
        if events["exit_early"]:
            events["exit_early"] = False
            break
        q, g = arm.read()                       # 관찰(팔로워 실제 관절)
        w, aw = wrist.latest_rgb()
        tp, at = top.latest_rgb()
        act = leader.action()                   # 리더 관절
        sent = arm.send_action(act)             # 실제로 보낸 값(안전 제한으로 잘렸을 수 있음) = 학습 목표
        if isinstance(sent, dict) and all(n_ in sent for n_ in NAMES):
            # 그리퍼는 블록을 쥐고 있으면 늘 잘린다(팔로워는 블록 폭에서 멈추고 리더는 더 닫힘) → 팔 관절만 센다
            clipped += any(abs(float(sent[n_]) - float(act[n_])) > 1e-3 for n_ in NAMES[:5])
            act = sent
        if w is None or tp is None:
            continue
        max_age = max(max_age, aw, at)
        dataset.add_frame(make_frame(dataset.features, q, g, act, w, tp))
        n += 1
        precise_sleep(max(period - (time.perf_counter() - t_loop), 0.0))
    dur = time.perf_counter() - t0
    if clipped:
        print(f"  주의: {clipped}프레임에서 팔 관절 명령이 안전 제한(한 번에 8°)으로 잘림 → 녹화에는 실제로 보낸 값이 들어감. 많으면 리더암을 더 천천히")
    return n, (n / dur if dur > 0 else 0.0), max_age


# ---------- 이어서 녹화: 기록 읽기·반반 흔들기·설정 검사 ----------
META_NAME = "kitbot_episode_meta.jsonl"


def dataset_root(repo_id: str, root: str | None) -> Path:
    from lerobot.utils.constants import HF_LEROBOT_HOME

    return Path(root) if root else Path(HF_LEROBOT_HOME) / repo_id


def read_meta(path: Path, n_episodes: int | None = None) -> list[dict]:
    """저장된 에피소드의 시작 조건. n_episodes를 주면 데이터셋에 실제로 있는 에피소드 번호만."""
    if not path.exists():
        return []
    rows = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    if n_episodes is not None:
        rows = [r for r in rows if r.get("episode_index", -1) < n_episodes]
    return rows


def is_jittered(rec: dict) -> bool:
    return any(float(v) > 0 for v in rec.get("jitter_range", [0.0, 0.0]))


def jitter_counts(meta: list[dict]) -> tuple[int, int]:
    """(흔든 수, 안 흔든 수)"""
    j = sum(is_jittered(r) for r in meta)
    return j, len(meta) - j


def choose_jitter(meta: list[dict], mode: str, prob: float, rng) -> bool:
    """balanced: 저장된 에피소드 중 적은 쪽을 고른다(같으면 무작위) → 몇 번에 나눠 녹화해도 차이 ≤ 1.
    random: 확률 prob로 흔든다."""
    if mode == "random":
        return bool(rng.uniform() < prob)
    j, n = jitter_counts(meta)
    if j != n:
        return j < n
    return bool(rng.uniform() < 0.5)


def settings_conflict(meta: list[dict], jitter_mm: float, jitter_deg: float, handoff_clear: float) -> str | None:
    """한 데이터셋 안에서 흔들기 폭·넘기는 높이를 바꾸지 않는다 (commands.md 2-5). 다르면 이유를 돌려준다."""
    for r in meta:
        if is_jittered(r) and [round(float(v), 3) for v in r["jitter_range"]] != [round(jitter_mm, 3), round(jitter_deg, 3)]:
            return f"에피소드 {r.get('episode_index')}의 흔들기 {r['jitter_range']} ≠ 지금 [{jitter_mm}, {jitter_deg}]"
        h = r.get("handoff_clear_mm") or 0.0
        if abs(float(h) - float(handoff_clear or 0.0)) > 1e-6:
            return f"에피소드 {r.get('episode_index')}의 넘기는 높이 {h} ≠ 지금 {handoff_clear}"
    return None


def flush_stdin() -> None:
    """녹화 중 누른 키(n, q 등)는 터미널 입력 칸에도 쌓인다. 다음 질문의 답으로 들어가지 않게 비운다."""
    try:
        import sys
        import termios

        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:  # 터미널이 아닐 때(시험 등)
        pass


def confirm_stop() -> bool:
    """q/Esc를 눌렀을 때 전체 종료를 한 번 더 묻는다 (실수로 누른 경우 대비)."""
    flush_stdin()
    return input("  q/Esc 눌림 → 전체 종료? (Enter = 종료, c = 계속 녹화) ").strip().lower() != "c"


# ---------- 리더암 손 맞춤 (자동 맞춤 뒤, 녹화 전) ----------
SHORT = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow", "wrist_flex": "wflex",
         "wrist_roll": "roll", "gripper": "grip"}


def leader_need(q, g, leader_act: dict) -> dict:
    """맞추려면 리더암을 얼마나 더 움직여야 하나 = 팔로워 − 리더 (관절별, 그리퍼는 0~100 값)."""
    fol = dict(zip(NAMES, list(q) + [g]))
    return {n.split(".")[0]: float(fol[n]) - float(leader_act[n]) for n in NAMES}


def gaps_ok(need: dict, tol_deg: float, tol_grip: float) -> bool:
    return all(abs(v) <= (tol_grip if k == "gripper" else tol_deg) for k, v in need.items())


def format_need(need: dict, tol_deg: float, tol_grip: float) -> str:
    parts = []
    for k, v in need.items():
        bad = abs(v) > (tol_grip if k == "gripper" else tol_deg)
        parts.append(f"{SHORT[k]} {v:+5.1f}{'*' if bad else ' '}")
    return " ".join(parts)


class CbreakKeys:
    """Enter 없이 키 하나씩 읽기 (터미널). 끝나면 터미널 설정을 되돌린다."""

    def __enter__(self):
        import sys
        import termios
        import tty

        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        return self

    def get(self, timeout_s: float):
        import os
        import select

        r, _, _ = select.select([self.fd], [], [], timeout_s)
        return os.read(self.fd, 1).decode(errors="ignore") if r else None

    def __exit__(self, *exc):
        import termios

        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def fine_tune_leader(arm, leader, tol_deg: float, tol_grip: float, keys, *, hz: float = 10.0, auto_start_s: float = 0.0,
                     clock=time.monotonic, show=print) -> tuple[str, dict, float]:
    """팔로워는 그 자리에 둔 채, 사람이 리더암을 손으로 맞추는 동안 관절별 차이를 계속 보여 준다.
    Enter/스페이스 = 녹화 시작(모두 허용 범위 안일 때만), f = 그래도 시작, a = 자동 맞춤 다시, s = 이 에피소드 건너뛰기.
    auto_start_s > 0이면 허용 범위 안을 그 시간 동안 유지할 때 키 없이 시작한다(두 손으로 리더암을 잡은 채).
    (선택, 마지막 차이, 걸린 초)"""
    t0 = clock()
    warned = False
    ok_since = None
    while True:
        q, g = arm.read()
        need = leader_need(q, g, leader.action())
        ok = gaps_ok(need, tol_deg, tol_grip)
        now = clock()
        ok_since = (ok_since if ok_since is not None else now) if ok else None
        if auto_start_s > 0 and ok_since is not None and now - ok_since >= auto_start_s:
            return "auto", need, now - t0
        show("\r  " + format_need(need, tol_deg, tol_grip) + ("  → 맞음, Enter = 녹화 시작 " if ok else "  (* = 더 맞추기) "),
             end="", flush=True)
        k = keys.get(1.0 / hz)
        if k is None:
            continue
        if k in ("\n", "\r", " "):
            if ok:
                return "start", need, clock() - t0
            if not warned:
                show(f"\n  아직 {tol_deg}°(그리퍼 {tol_grip}) 넘는 관절이 있음. 더 맞추거나 f = 그래도 시작")
                warned = True
        elif k.lower() in ("f", "a", "s"):
            return {"f": "force", "a": "realign", "s": "skip"}[k.lower()], need, clock() - t0


def wrist_sees_block(wrist: LatestFrame, color: str) -> bool:
    from . import wrist_measure as wm

    frames = []
    for _ in range(5):
        ok, f = wrist.read()
        if ok:
            from .camera import correct_wb
            frames.append(correct_wb(f, wrist.gains))
    return wm.observe(frames, color) is not None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo-id", required=True, help="데이터셋 이름 (예: local/kitbot_act_v1)")
    ap.add_argument("--root", help="저장 폴더 (기본 ~/.cache/huggingface/lerobot/<repo-id>)")
    ap.add_argument("--resume", action="store_true", help="(생략 가능) 데이터셋 폴더가 이미 있으면 자동으로 이어서 녹화")
    ap.add_argument("--episodes", type=int, help="이번에 녹화할 에피소드 수 (기본: --goal이 있으면 목표까지, 없으면 10)")
    ap.add_argument("--goal", type=int, help="데이터셋 전체 목표 에피소드 수 (예: 50). 다 차면 멈춘다")
    ap.add_argument("--pause-every", type=int, default=0, help="이번에 이만큼 저장할 때마다 쉬는 시간 (0 = 없음)")
    ap.add_argument("--episode-s", type=float, default=30.0, help="에피소드 최대 길이 [초]. n으로 일찍 끝낼 수 있음")
    from .vision import HSV_RANGES
    colors = sorted(HSV_RANGES)
    ap.add_argument("--color", default="purple", choices=colors,
                    help=f"집을 블록 색 (시작 값). 에피소드마다 입력 칸에 색 이름을 쳐서 바꿀 수 있음. 가능: {', '.join(colors)}")
    ap.add_argument("--jitter-mm", type=float, default=10.0, help="흔들기 반지름 [mm] (기본 10, S05 D03)")
    ap.add_argument("--jitter-deg", type=float, default=10.0, help="흔들기 각도 ± [°] (기본 10)")
    ap.add_argument("--jitter-mode", choices=["balanced", "random"], default="balanced",
                    help="balanced(기본) = 저장된 것 기준 흔듦·안 흔듦을 정확히 반반, random = --jitter-prob 확률")
    ap.add_argument("--jitter-prob", type=float, default=0.5, help="random일 때 흔들 확률 0~1")
    ap.add_argument("--handoff-clear", type=float, default=15.0, help="넘기는 높이: 블록 윗면 위 [mm, 명령]. 0이면 40mm 접근에서 넘김")
    ap.add_argument("--max-arrival-deg", type=float, default=4.0, help="인계 판단: 접근 후 관절 오차 최대 [°]")
    ap.add_argument("--leader-torque", type=int, default=400)
    ap.add_argument("--start-gap", type=float, default=3.0,
                    help="손 맞춤: 모든 팔 관절의 리더−팔로워 차이가 이 안이면 녹화 시작 가능 [°]")
    ap.add_argument("--start-grip-gap", type=float, default=3.0, help="손 맞춤: 그리퍼 차이 허용 (0~100 값)")
    ap.add_argument("--auto-start-s", type=float, default=0.0,
                    help="손 맞춤: 허용 범위 안을 이 시간[초] 유지하면 키 없이 녹화 시작 (0 = 끔, 키로만)")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--step", action="store_true", help="이동 전마다 Enter")
    args = ap.parse_args(argv)

    from lerobot.utils.keyboard_input import init_keyboard_listener

    from .arm import Arm, Leader, load_poses
    from .calib import PixelToRobot
    from .camera import open_role
    from .kinematics import SO101Kinematics
    from .pick import CALIB, PickParams

    poses = load_poses()
    q_home = np.asarray(poses["home"]["q_deg"], float)
    g_open = float(poses["gripper_pregrasp"]["gripper"])
    calib = PixelToRobot.load(CALIB)
    kin = SO101Kinematics()
    corr = build_correction("none")
    rng = np.random.default_rng(args.seed)
    p = PickParams()

    root = dataset_root(args.repo_id, args.root)
    resume = args.resume or (root / "meta" / "info.json").exists()
    if args.resume and not root.exists():
        raise SystemExit(f"이어서 녹화할 데이터셋이 없음: {root}")
    old_meta = read_meta(root / META_NAME) if resume else []
    bad = settings_conflict(old_meta, args.jitter_mm, args.jitter_deg, args.handoff_clear)
    if bad:
        raise SystemExit(f"설정이 기존 에피소드와 다름 → 새 데이터셋 이름을 쓰거나 옵션을 맞출 것: {bad}")
    if args.episodes is not None:
        session_max = args.episodes
    elif args.goal is not None:
        session_max = max(args.goal - len(old_meta), 0)
    else:
        session_max = 10
    if session_max <= 0:
        j, n = jitter_counts(old_meta)
        raise SystemExit(f"이미 목표 {args.goal}개 이상 (기록 {len(old_meta)}개, 흔듦 {j} · 안 흔듦 {n}): {root}")

    def step(msg):
        print(msg, flush=True)
        if args.step:
            input("  Enter = 진행, Ctrl+C = 중단 ")

    top = wrist = arm = leader = dataset = None
    saved = 0
    why = "오류 또는 Ctrl+C"
    try:
        top = LatestFrame(*open_role("top", WIDTH, HEIGHT, FPS))
        wrist = LatestFrame(*open_role("wrist", WIDTH, HEIGHT, FPS))
        arm = Arm(hold_on_exit=True)
        leader = Leader()
        dataset = open_dataset(args.repo_id, str(root), resume)   # resume은 폴더를 직접 줘야 함(LeRobot 0.6)
        meta_path = Path(dataset.root) / META_NAME
        meta = read_meta(meta_path, dataset.num_episodes)
        print(f"데이터셋: {dataset.root} ({'이어서' if resume else '새로'}, 지금 {dataset.num_episodes}개)")
        if len(meta) != dataset.num_episodes:
            print(f"  주의: 시작 조건 기록 {len(meta)}개 ≠ 에피소드 {dataset.num_episodes}개 (반반 맞추기는 기록 기준)")

        def progress():
            j, n = jitter_counts(meta)
            g = f"/{args.goal}" if args.goal else ""
            return f"데이터셋 {dataset.num_episodes}{g} (흔듦 {j} · 안 흔듦 {n}), 이번 {saved}/{session_max}"

        color = args.color
        while saved < session_max and not (args.goal and dataset.num_episodes >= args.goal):
            ep = dataset.num_episodes
            jit = choose_jitter(meta, args.jitter_mode, args.jitter_prob, rng)
            print(f"\n{progress()}")
            ans = input(f"[에피소드 {ep}, {'흔듦' if jit else '안 흔듦'}] 블록을 새 자리에 놓고 Enter (지금 색: {color} / "
                        f"색 바꾸기: 색 이름 입력 [{', '.join(colors)}] / q = 끝) ").strip().lower()
            if ans == "q":
                why = "입력 q"
                break
            if ans:
                if ans not in colors:
                    print(f"모르는 색: {ans}. 가능: {', '.join(colors)}")
                    continue
                color = ans
                print(f"이번부터 {color} 블록을 집습니다")
            try:
                rec = approach_block(arm, top, top.gains, kin, calib, poses, corr, color=color, step=step,
                                     jitter_mm=args.jitter_mm if jit else 0.0,
                                     jitter_deg=args.jitter_deg if jit else 0.0, rng=rng,
                                     handoff_clear_mm=args.handoff_clear or None)
            except (RuntimeError, ValueError) as e:
                print("접근 실패 → 이 에피소드 건너뜀:", e)
                arm.move_joints(q_home, g_open, p.joint_speed_dps)
                continue
            rec.pop("plan", None)
            if rec["max_joint_err_deg"] > args.max_arrival_deg:
                print(f"인계 판단 실패: 관절 오차 {rec['max_joint_err_deg']}° > {args.max_arrival_deg}° → 건너뜀")
                arm.move_joints(q_home, g_open, p.joint_speed_dps)
                continue
            if not wrist_sees_block(wrist, color):
                print("인계 판단 실패: 손목 카메라에 블록이 안 보임 → 건너뜀")
                arm.move_joints(q_home, g_open, p.joint_speed_dps)
                continue

            q, g = arm.read()
            choice, n_align = "realign", 0
            while choice == "realign":
                step("리더암 자동 맞춤: 리더암이 스스로 움직입니다 (손 치우기)")
                lg = align_leader(leader, q, g, torque_limit=args.leader_torque)
                n_align += 1
                print(f"  리더암 힘 푼 뒤 관절 차이 최대 {lg['max_gap_released_deg']:.1f}°")
                print(f"  손 맞춤: 숫자 = 리더암을 더 움직여야 할 양(팔로워 − 리더). 허용 {args.start_gap}°, 그리퍼 {args.start_grip_gap}."
                      f"\n  Enter/스페이스 = 녹화 시작, f = 그래도 시작, a = 자동 맞춤 다시, s = 이 에피소드 건너뛰기"
                      + (f"\n  허용 범위 안을 {args.auto_start_s:.1f}초 유지하면 자동 시작" if args.auto_start_s > 0 else ""))
                with CbreakKeys() as keys:
                    choice, need, tune_s = fine_tune_leader(arm, leader, args.start_gap, args.start_grip_gap, keys,
                                                           auto_start_s=args.auto_start_s)
                print()
            lg.update(n_align=n_align, fine_tune_s=round(tune_s, 1), start_choice=choice,
                      gap_at_start={k: round(v, 2) for k, v in need.items()},
                      max_gap_at_start_deg=round(max(abs(v) for k, v in need.items() if k != "gripper"), 2))
            rec["leader"] = lg
            if choice == "skip":
                print("  건너뜀")
                arm.move_joints(q_home, g_open, p.joint_speed_dps)
                continue
            print(f"  시작 때 관절 차이 최대 {lg['max_gap_at_start_deg']:.1f}° (손 맞춤 {tune_s:.0f}초"
                  + (", 허용 범위 넘은 채 시작" if choice == "force" else "") + ")")

            listener, events = init_keyboard_listener()
            print(f"  ● 녹화 중 (최대 {args.episode_s:.0f}초): 보정 → 집기 → 바구니 → HOME.  n = 끝·저장, r = 버리고 다시, q = 저장 후 종료")
            try:
                n, hz, age = record_episode(arm, leader, wrist, top, dataset, events, args.episode_s)
            finally:
                if listener is not None:
                    listener.stop()
                flush_stdin()
            if events["rerecord_episode"] or n == 0:
                dataset.clear_episode_buffer()
                print("  버림 (다시 녹화)")
            else:
                dataset.save_episode()
                saved += 1
                rec.update(episode_index=ep, frames=n, mean_hz=round(hz, 1), max_frame_age_s=round(age, 3),
                           repo_id=args.repo_id)
                with meta_path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                meta.append(rec)
                print(f"  저장: 에피소드 {ep}, {n}프레임, 평균 {hz:.1f}Hz" + ("  ← 30Hz보다 느림" if hz < FPS * 0.9 else ""))
            step("팔로워 HOME으로 (녹화 밖)")
            arm.move_joints(q_home, g_open, p.joint_speed_dps)
            if events["stop_recording"] and confirm_stop():
                why = "녹화 중 q/Esc"
                break
            if (args.pause_every and saved and saved % args.pause_every == 0 and saved < session_max
                    and not (args.goal and dataset.num_episodes >= args.goal)):
                flush_stdin()
                print(f"\n── 쉬는 시간 ({progress()}) ──  리더암·블록·카메라 위치, 조명 확인")
                if input("  Enter = 계속, q = 여기서 끝 ").strip().lower() == "q":
                    why = "쉬는 시간에 q"
                    break
        else:
            why = (f"전체 목표 {args.goal}개 도달" if args.goal and dataset.num_episodes >= args.goal
                   else f"이번 목표 {session_max}개 저장")
    finally:
        if dataset is not None:
            try:
                dataset.clear_episode_buffer()  # 저장하지 않은 남은 프레임
            except Exception:
                pass
            dataset.finalize()
            j, n = jitter_counts(read_meta(Path(dataset.root) / META_NAME, dataset.num_episodes))
            print(f"\n끝난 이유: {why}. 이번에 저장 {saved}개, 데이터셋 전체 {dataset.num_episodes}개"
                  f" (흔듦 {j} · 안 흔듦 {n}): {dataset.root}")
        for c in (top, wrist):
            if c is not None:
                c.close()
        if leader is not None:
            leader.disconnect()
        if arm is not None:
            arm.disconnect()


if __name__ == "__main__":
    main()
