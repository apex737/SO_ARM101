"""ACT 시연을 여러 에피소드 이어서 녹화한다 (S05-1). LeRobot 데이터셋으로 저장 → lerobot-train으로 바로 학습.

    cd project/src
    python -m kitbot_exec.record_act --repo-id local/kitbot_act_v1 --episodes 10
    python -m kitbot_exec.record_act --repo-id local/kitbot_act_v1 --episodes 10 --resume     # 같은 데이터셋에 이어서

에피소드 하나:
  ① 블록을 새 자리에 놓고 Enter
  ② HOME → 위 카메라로 블록 찾기 → (흔든) 목표로 40mm 접근 → 블록 윗면 위 15mm까지 수직 하강   (approach.py와 같은 계산)
  ③ 인계 판단: 관절 오차 ≤ --max-arrival-deg, 손목 카메라에 블록이 보임. 아니면 이 에피소드를 건너뜀
  ④ 리더암 자동 맞춤(리더 서보 힘 40%로 팔로워 자세까지 이동 → 리더암을 잡고 Enter → 힘 풀림)
  ⑤ 녹화: 리더암으로 보정 → 집기 → 바구니 → HOME.  n(또는 →) = 끝내고 저장, r(또는 ←) = 버리고 다시, q(또는 Esc) = 저장하고 전체 종료
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

    n, t0, max_age = 0, time.perf_counter(), 0.0
    period = 1.0 / hz
    while time.perf_counter() - t0 < episode_s:
        t_loop = time.perf_counter()
        if events["exit_early"]:
            events["exit_early"] = False
            break
        q, g = arm.read()                       # 관찰(팔로워 실제 관절)
        w, aw = wrist.latest_rgb()
        tp, at = top.latest_rgb()
        act = leader.action()                   # 동작(리더 관절) = 학습 목표
        arm.send_action(act)
        if w is None or tp is None:
            continue
        max_age = max(max_age, aw, at)
        dataset.add_frame(make_frame(dataset.features, q, g, act, w, tp))
        n += 1
        precise_sleep(max(period - (time.perf_counter() - t_loop), 0.0))
    dur = time.perf_counter() - t0
    return n, (n / dur if dur > 0 else 0.0), max_age


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
    ap.add_argument("--resume", action="store_true", help="같은 데이터셋에 이어서 녹화")
    ap.add_argument("--episodes", type=int, default=10, help="이번에 녹화할 에피소드 수")
    ap.add_argument("--episode-s", type=float, default=30.0, help="에피소드 최대 길이 [초]. n으로 일찍 끝낼 수 있음")
    from .vision import HSV_RANGES
    colors = sorted(HSV_RANGES)
    ap.add_argument("--color", default="purple", choices=colors,
                    help=f"집을 블록 색 (시작 값). 에피소드마다 입력 칸에 색 이름을 쳐서 바꿀 수 있음. 가능: {', '.join(colors)}")
    ap.add_argument("--jitter-mm", type=float, default=15.0)
    ap.add_argument("--jitter-deg", type=float, default=15.0)
    ap.add_argument("--jitter-prob", type=float, default=1.0,
                    help="에피소드마다 흔들 확률 0~1. 0.5면 절반은 흔들지 않은 CV 접근 그대로 (기본 1 = 모두 흔듦)")
    ap.add_argument("--handoff-clear", type=float, default=15.0, help="넘기는 높이: 블록 윗면 위 [mm, 명령]. 0이면 40mm 접근에서 넘김")
    ap.add_argument("--max-arrival-deg", type=float, default=4.0, help="인계 판단: 접근 후 관절 오차 최대 [°]")
    ap.add_argument("--leader-torque", type=int, default=400)
    ap.add_argument("--max-leader-gap", type=float, default=5.0, help="리더암 힘 푼 뒤 관절 차이가 이보다 크면 녹화 전에 다시 묻는다 [°]")
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

    def step(msg):
        print(msg, flush=True)
        if args.step:
            input("  Enter = 진행, Ctrl+C = 중단 ")

    top = wrist = arm = leader = dataset = None
    saved = 0
    try:
        top = LatestFrame(*open_role("top", WIDTH, HEIGHT, FPS))
        wrist = LatestFrame(*open_role("wrist", WIDTH, HEIGHT, FPS))
        arm = Arm(hold_on_exit=True)
        leader = Leader()
        dataset = open_dataset(args.repo_id, args.root, args.resume)
        meta_path = Path(dataset.root) / "kitbot_episode_meta.jsonl"
        print(f"데이터셋: {dataset.root} (지금 {dataset.num_episodes}개)")

        color = args.color
        while saved < args.episodes:
            ep = dataset.num_episodes
            ans = input(f"\n[에피소드 {ep}] 블록을 새 자리에 놓고 Enter (지금 색: {color} / 색 바꾸기: 색 이름 입력 "
                        f"[{', '.join(colors)}] / q = 끝) ").strip().lower()
            if ans == "q":
                break
            if ans:
                if ans not in colors:
                    print(f"모르는 색: {ans}. 가능: {', '.join(colors)}")
                    continue
                color = ans
                print(f"이번부터 {color} 블록을 집습니다")
            jit = rng.uniform() < args.jitter_prob
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
            step("리더암 자동 맞춤: 리더암이 스스로 움직입니다 (손 치우기)")
            lg = align_leader(leader, q, g, torque_limit=args.leader_torque)
            rec["leader"] = lg
            print(f"  리더암 힘 푼 뒤 관절 차이 최대 {lg['max_gap_released_deg']:.1f}°")
            if lg["max_gap_released_deg"] > args.max_leader_gap:
                if input(f"  {args.max_leader_gap}°보다 큼. 그래도 녹화? (y = 녹화, 그 밖 = 건너뜀) ").strip().lower() != "y":
                    arm.move_joints(q_home, g_open, p.joint_speed_dps)
                    continue

            listener, events = init_keyboard_listener()
            print(f"  ● 녹화 중 (최대 {args.episode_s:.0f}초): 보정 → 집기 → 바구니 → HOME.  n = 끝·저장, r = 버리고 다시, q = 저장 후 종료")
            try:
                n, hz, age = record_episode(arm, leader, wrist, top, dataset, events, args.episode_s)
            finally:
                if listener is not None:
                    listener.stop()
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
                print(f"  저장: 에피소드 {ep}, {n}프레임, 평균 {hz:.1f}Hz" + ("  ← 30Hz보다 느림" if hz < FPS * 0.9 else ""))
            step("팔로워 HOME으로 (녹화 밖)")
            arm.move_joints(q_home, g_open, p.joint_speed_dps)
            if events["stop_recording"]:
                break
    finally:
        if dataset is not None:
            try:
                dataset.clear_episode_buffer()  # 저장하지 않은 남은 프레임
            except Exception:
                pass
            dataset.finalize()
            print(f"\n이번에 저장 {saved}개, 데이터셋 전체 {dataset.num_episodes}개: {dataset.root}")
        for c in (top, wrist):
            if c is not None:
                c.close()
        if leader is not None:
            leader.disconnect()
        if arm is not None:
            arm.disconnect()


if __name__ == "__main__":
    main()
