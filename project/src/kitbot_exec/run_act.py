"""학습한 ACT 정책을 실물에서 실행하고 결과를 기록한다 (S05-4).

    cd project/src
    python -m kitbot_exec.run_act --policy <체크포인트>/pretrained_model --color purple --step
    python -m kitbot_exec.run_act --policy ... --name act_v1 --color purple          # 비교 실험 18회 (격자 g01~g09 × 2)

<체크포인트> 예: outputs/train/act_kitbot_v1/checkpoints/last  (lerobot-train --output_dir 아래, commands.md 6절)

에피소드 하나 (녹화 record_act와 같은 접근·인계·카메라 처리):
  ① 블록을 작업 영역 아무 곳에 놓고 Enter (자리는 접근 때 찾은 값으로 자동 기록, 가장 가까운 시연 자리까지 거리 포함)
     --grid면 격자점(g01→g09, 2바퀴)을 안내하고 위 카메라로 확인(멀면 다시 놓기)
     (색 이름 입력 = 색 바꾸기, q = 끝)
  ② HOME → 위 카메라로 블록 찾기 → 40mm 접근 → 블록 윗면 위 15mm까지 수직 하강 (흔들기 없음이 기본)
  ③ 인계 판단: 관절 오차 ≤ --max-arrival-deg, 손목 카메라에 블록이 보임. 아니면 HOME (이것도 한 시행으로 기록)
  ④ 정책 첫 명령이 지금 자세와 --max-first-gap° 넘게 다르면 묻는다
  ⑤ 정책 실행 (30Hz). 아래 중 하나가 되면 멈춘다:
       home    HOME 근처(--home-tol°)에 --home-hold-s초 머묾  → 정상 끝
       empty   그리퍼를 닫으라는 명령인데 끝까지 닫힘(블록 없음) --empty-hold-s초 → 빈손
       dropped 잡았다가 옮기는 도중 끝까지 닫힘 → 떨어뜨림
       floor   명령한 그리퍼 끝이 테이블보다 --floor-mm 넘게 아래 → 그 명령은 보내지 않고 멈춤
       timeout --max-s초가 지남
       key     n / q / Esc / → (q·Esc는 이번 뒤 전체 종료)
  ⑥ 멈춘 뒤: 빈손·떨어뜨림이면 그리퍼 열고 HOME, 그 밖은 Enter 뒤 HOME
  ⑦ 끝 사진(위·손목) 저장 → 사람 판정: 순수 IK(eval_ik.py)와 같은 코드 s/g/d/b/x/? (compare.py 설명)

기록: var/compare/<--name>.jsonl, 사진 var/compare/<--name>/. 비교: python -m kitbot_exec.compare <기록들>
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field

import numpy as np

from .approach import approach_block, build_correction
from .record_act import (FPS, HEIGHT, NAMES, TASK, WIDTH, LatestFrame, confirm_stop, dataset_features,
                         flush_stdin, wrist_sees_block)

HELD_MARGIN = 3.0   # pick.py와 같은 기준: 닫은 뒤 측정값이 빈손 닫힘값보다 이만큼 넘게 크면 블록을 쥔 것


# ---------- 정책 ----------
def load_policy(path: str, device: str | None = None, n_action_steps: int | None = None,
                temporal_ensemble: float | None = None):
    """lerobot-train이 저장한 pretrained_model 폴더 → (정책, 전처리, 후처리, 장치)."""
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies import get_policy_class, make_pre_post_processors

    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    cfg = PreTrainedConfig.from_pretrained(path)
    cfg.pretrained_path = path
    cfg.device = device
    if temporal_ensemble is not None:            # 매 프레임 새로 예측해 겹치는 예측을 평균 (ACT 논문 방식)
        cfg.temporal_ensemble_coeff = temporal_ensemble
        cfg.n_action_steps = 1
    elif n_action_steps is not None:
        cfg.n_action_steps = n_action_steps
    policy = get_policy_class(cfg.type).from_pretrained(path, config=cfg)
    policy.to(device).eval()
    pre, post = make_pre_post_processors(policy_cfg=cfg, pretrained_path=path,
                                         preprocessor_overrides={"device_processor": {"device": device}})
    return policy, pre, post, device


class PolicyRunner:
    """관찰(관절·그리퍼·두 카메라 RGB) → 정책 → 관절 명령 dict. 녹화와 같은 데이터 형식을 쓴다."""

    def __init__(self, policy, pre, post, device, features: dict | None = None):
        self.policy, self.pre, self.post, self.device = policy, pre, post, device
        self.features = features or dataset_features()
        # 정책이 실제로 받는 입력만 넘긴다 (예: 위 카메라를 뺀 '손목만' 정책)
        self.inputs = set(policy.config.input_features)
        missing = self.inputs - set(self.features)
        if missing:
            raise ValueError(f"정책 입력 {sorted(missing)}을(를) 만들 수 없음. 녹화 형식: {sorted(self.features)}")
        n_out = policy.config.output_features["action"].shape[0]
        if n_out != len(NAMES):
            raise ValueError(f"정책 출력 {n_out}개 ≠ 관절 {len(NAMES)}개")

    def reset(self):
        self.policy.reset()
        self.pre.reset()
        self.post.reset()

    def obs_frame(self, q, g, wrist_rgb, top_rgb) -> dict:
        from lerobot.utils.constants import OBS_STR
        from lerobot.utils.feature_utils import build_dataset_frame

        obs = {n: float(v) for n, v in zip(NAMES, list(q) + [g])}
        obs.update(wrist=wrist_rgb, top=top_rgb)
        frame = build_dataset_frame(self.features, obs, OBS_STR)
        return {k: v for k, v in frame.items() if k in self.inputs}

    def act(self, q, g, wrist_rgb, top_rgb) -> dict:
        import torch
        from lerobot.policies import make_robot_action, prepare_observation_for_inference

        obs = self.obs_frame(q, g, wrist_rgb, top_rgb)
        dev = torch.device(self.device)
        with torch.inference_mode():
            obs = prepare_observation_for_inference(obs, dev, TASK, "so101_follower")
            action = self.post(self.policy.select_action(self.pre(obs)))
        return make_robot_action(action, self.features)


# ---------- 끝·실패 판정 ----------
@dataclass
class Monitor:
    """매 프레임 (시간, 관절, 그리퍼 측정값, 그리퍼 명령, 명령한 끝 높이)를 받아 멈출 이유를 돌려준다."""
    q_home: np.ndarray
    g_open: float
    g_closed: float
    max_s: float = 30.0
    home_tol_deg: float = 8.0
    home_hold_s: float = 1.0
    empty_hold_s: float = 0.5
    floor_mm: float = 10.0
    held_hold_s: float = 0.5   # 닫히는 중(측정값이 명령을 따라가는 중)을 잡음으로 보지 않게
    # 기록
    grasped_at: float | None = None
    released_at: float | None = None
    q_release: list | None = None
    min_cmd_clear_mm: float = np.inf
    _since: dict = field(default_factory=dict)

    def _held_for(self, key, cond, t, need_s) -> bool:
        if not cond:
            self._since.pop(key, None)
            return False
        t0 = self._since.setdefault(key, t)
        return t - t0 >= need_s

    def update(self, t: float, q, g_meas: float, g_cmd: float, cmd_clear_mm: float | None = None) -> str | None:
        q = np.asarray(q, float)
        mid = 0.5 * (self.g_open + self.g_closed)
        closing = g_cmd < mid
        # 닫으라는 값(또는 빈손 닫힘값)까지 다 닫힘 = 사이에 아무것도 없음. 블록이 있으면 그 폭에서 멈춘다.
        shut = g_meas <= max(g_cmd, self.g_closed) + HELD_MARGIN
        if cmd_clear_mm is not None:
            self.min_cmd_clear_mm = min(self.min_cmd_clear_mm, cmd_clear_mm)
            if cmd_clear_mm < -self.floor_mm:
                return "floor"
        if self._held_for("held", closing and not shut, t, self.held_hold_s) and self.grasped_at is None:
            self.grasped_at = t
        if self.grasped_at is not None and self.released_at is None and not closing:
            self.released_at, self.q_release = t, [round(float(v), 1) for v in q]
        # 놓은 뒤(바구니) 다시 닫는 것은 빈손·떨어뜨림이 아니다
        if self.released_at is None and self._held_for("shut", closing and shut, t, self.empty_hold_s):
            return "dropped" if self.grasped_at is not None else "empty"
        if self._held_for("home", np.max(np.abs(q - self.q_home)) < self.home_tol_deg, t, self.home_hold_s):
            return "home"
        if t >= self.max_s:
            return "timeout"
        return None

    def summary(self) -> dict:
        r = lambda v: None if v is None else round(float(v), 2)
        return {"grasped_at_s": r(self.grasped_at), "released_at_s": r(self.released_at), "q_release": self.q_release,
                "min_cmd_clear_mm": None if not np.isfinite(self.min_cmd_clear_mm) else round(self.min_cmd_clear_mm, 1)}


def cmd_clearance(kin, calib, action: dict, block_mm: float) -> float:
    """명령한 관절값에서 모델 그리퍼 끝(고정 부리 끝)이 그 자리 테이블보다 얼마나 위인가 [mm]."""
    p, _ = kin.fk([float(action[n]) for n in NAMES[:5]])   # NAMES는 이미 'shoulder_pan.pos' 형태
    return float(p[2] - (calib.z_at([p[:2]])[0] - block_mm))


def run_episode(arm, runner: PolicyRunner, monitor: Monitor, wrist: LatestFrame, top: LatestFrame, events: dict,
                clearance=None, hz: float = FPS) -> dict:
    """정책을 실행하다 Monitor가 멈추라 하면(또는 키) 멈춘다. 멈출 때 보내지 않은 명령은 없다(floor는 그 명령 전에 멈춤)."""
    from lerobot.utils.robot_utils import precise_sleep

    period = 1.0 / hz
    n, clipped, max_age, slow = 0, 0, 0.0, 0
    reason = None
    t0 = time.perf_counter()
    while reason is None:
        t_loop = time.perf_counter()
        if events["exit_early"]:
            reason = "key"
            break
        q, g = arm.read()
        w, aw = wrist.latest_rgb()
        tp, at = top.latest_rgb()
        if w is None or tp is None:
            time.sleep(0.005)
            continue
        max_age = max(max_age, aw, at)
        act = runner.act(q, g, w, tp)
        c = clearance(act) if clearance else None
        reason = monitor.update(time.perf_counter() - t0, q, g, float(act["gripper.pos"]), c)
        if reason == "floor":
            break
        sent = arm.send_action(act)
        if isinstance(sent, dict) and all(k in sent for k in NAMES):
            clipped += any(abs(float(sent[k]) - float(act[k])) > 1e-3 for k in NAMES[:5])  # 그리퍼는 쥐면 늘 잘림
        n += 1
        spent = time.perf_counter() - t_loop
        slow += spent > period * 1.5
        precise_sleep(max(period - spent, 0.0))
    dur = time.perf_counter() - t0
    return {"end": reason, "frames": n, "duration_s": round(dur, 2), "mean_hz": round(n / dur, 1) if dur > 0 else 0.0,
            "clipped_frames": clipped, "slow_frames": slow, "max_frame_age_s": round(max_age, 3), **monitor.summary()}


END_KO = {"home": "HOME 도착", "empty": "빈손 (닫았는데 블록 없음)", "dropped": "옮기다 떨어뜨림",
          "floor": "테이블 아래 명령 → 멈춤", "timeout": "시간 초과", "key": "키로 멈춤"}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", required=True, help="pretrained_model 폴더 (예: outputs/train/act_kitbot_v1/checkpoints/last/pretrained_model)")
    ap.add_argument("--name", default=time.strftime("run_%Y%m%d_%H%M"), help="기록 이름 → var/compare/<name>.jsonl")
    ap.add_argument("--trials", type=int, default=18, help="시행 수 (평가 격자 g01~g09 × 2 = 18)")
    ap.add_argument("--grid", action="store_true", help="평가 격자 g01~g09를 순서대로 안내·확인 (기본: 아무 자리)")
    ap.add_argument("--start", type=int, default=0, help="몇 번째 시행부터 (끊긴 뒤 이어서: 이미 기록된 수)")
    ap.add_argument("--place-tol", type=float, default=15.0, help="--grid일 때 격자점 거리 허용 [mm]")
    from .vision import HSV_RANGES
    colors = sorted(HSV_RANGES)
    ap.add_argument("--color", default="purple", choices=colors)
    ap.add_argument("--device", help="cuda / cpu (기본: GPU 있으면 cuda)")
    ap.add_argument("--n-action-steps", type=int, help="한 번 예측한 묶음에서 몇 개를 실행하고 다시 예측할지 (기본: 학습 설정값)")
    ap.add_argument("--temporal-ensemble", type=float, metavar="K",
                    help="매 프레임 예측 + 겹친 예측을 지수 가중 평균 (예: 0.01). 더 부드럽지만 계산이 많다")
    ap.add_argument("--num-inference-steps", type=int, help="디퓨전 전용: 노이즈 제거 단계 수 (기본: 학습 설정값). 줄이면 빠르지만 동작이 달라짐")
    ap.add_argument("--jitter-mm", type=float, default=0.0, help="접근 흔들기 [mm] (기본 0 = 평가. 강인성 시험만)")
    ap.add_argument("--jitter-deg", type=float, default=0.0)
    ap.add_argument("--seed", type=int)
    ap.add_argument("--handoff-clear", type=float, default=15.0, help="넘기는 높이 (녹화와 같게)")
    ap.add_argument("--max-arrival-deg", type=float, default=4.0)
    ap.add_argument("--max-first-gap", type=float, default=10.0, help="정책 첫 명령과 지금 자세 차이가 이보다 크면 묻는다 [°]")
    ap.add_argument("--max-step-deg", type=float, default=8.0,
                    help="한 프레임에 관절이 움직일 수 있는 최대 [°]. 녹화와 같은 8 (낮추면 쥐는 힘·속도가 시연과 달라짐)")
    ap.add_argument("--max-s", type=float, default=30.0, help="정책 실행 최대 [초]")
    ap.add_argument("--home-tol", type=float, default=8.0)
    ap.add_argument("--home-hold-s", type=float, default=1.0)
    ap.add_argument("--empty-hold-s", type=float, default=0.5)
    ap.add_argument("--floor-mm", type=float, default=10.0, help="명령한 그리퍼 끝이 테이블보다 이만큼 아래면 멈춤 [mm]")
    ap.add_argument("--step", action="store_true", help="이동·정책 시작 전마다 Enter")
    ap.add_argument("--queue", help='큐: "orange 2, yellow 1, blue" = 주황 최대 2개 → 노랑 최대 1개 → 파랑은 안 보일 때까지. '
                                    "그 색이 안 보이면 다음 명령으로. 묻지 않고 이어서 실행(사람 판정 없음). "
                                    "같은 색이 여러 개면 로봇에서 가까운 것부터")
    args = ap.parse_args(argv)

    from lerobot.utils.keyboard_input import init_keyboard_listener

    from .arm import Arm, load_poses
    from .calib import PixelToRobot
    from .camera import open_role
    import cv2

    from . import compare as cp
    from .exp_s02 import REGION_KO
    from .kinematics import SO101Kinematics
    from .pick import CALIB, PickParams

    print("정책 불러오는 중:", args.policy)
    policy, pre, post, device = load_policy(args.policy, args.device, args.n_action_steps, args.temporal_ensemble)
    pc = policy.config
    if pc.type != "act" and args.temporal_ensemble is not None:
        raise SystemExit("--temporal-ensemble은 ACT 전용입니다")
    if args.num_inference_steps:
        if not hasattr(policy, "diffusion"):
            raise SystemExit("--num-inference-steps는 디퓨전 전용입니다")
        policy.diffusion.num_inference_steps = args.num_inference_steps
    runner = PolicyRunner(policy, pre, post, device)
    print(f"  {pc.type}, 장치 {device}, 입력 {sorted(runner.inputs)}, 묶음 {getattr(pc, 'chunk_size', getattr(pc, 'horizon', '?'))}, "
          f"실행 {getattr(pc, 'n_action_steps', '?')}, 시간 평균 {getattr(pc, 'temporal_ensemble_coeff', None)}")

    poses = load_poses()
    q_home = np.asarray(poses["home"]["q_deg"], float)
    g_open = float(poses["gripper_pregrasp"]["gripper"])
    g_closed = float(poses["gripper_closed"]["gripper"])
    g_home = float(poses["home"].get("gripper", g_closed))   # HOME에서 끝낼 때 그리퍼 (닫힘)
    calib = PixelToRobot.load(CALIB)
    kin = SO101Kinematics()
    corr = build_correction("none")
    rng = np.random.default_rng(args.seed)
    p = PickParams()
    log_path = cp.OUT_DIR / f"{args.name}.jsonl"

    def step(msg):
        print(msg, flush=True)
        if args.step:
            input("  Enter = 진행, Ctrl+C = 중단 ")

    def home(gripper=g_open):
        arm.move_joints(q_home, gripper, p.joint_speed_dps)

    stop_all = {"v": False}
    queue = cp.PickQueue(cp.parse_queue(args.queue, colors)) if args.queue else None
    demos = cp.demo_positions()

    def run_trial(rec: dict, color: str):
        """접근 → 인계 판단 → 정책. (자동 결과, 정책 실행 결과 dict 또는 None). 인계 실패도 한 시행으로 센다(비교 공정성)."""
        try:
            a = approach_block(arm, top, top.gains, kin, calib, poses, corr, color=color, step=step,
                               jitter_mm=args.jitter_mm, jitter_deg=args.jitter_deg, rng=rng,
                               handoff_clear_mm=args.handoff_clear or None, nearest=queue is not None)
        except (RuntimeError, ValueError) as e:
            print("  접근 실패:", e)
            home()
            return "approach_failed", {"error": str(e)}
        a.pop("plan", None)
        rec["approach"] = a
        x, y = a["block_camera"][:2]
        rec.update(block_camera=a["block_camera"], **cp.place_record(calib, x, y, demos))
        if rec.get("grid"):
            rec["grid_offset_at_run_mm"] = round(cp.grid_offset(x, y, rec["grid"]), 1)
        print(f"  블록 자리 x={x:.0f} y={y:.0f}: {REGION_KO[rec['region']]}, 가장 가까운 시연 {rec['nearest_demo_mm']}mm")
        if a["max_joint_err_deg"] > args.max_arrival_deg:
            print(f"  인계 판단 실패: 관절 오차 {a['max_joint_err_deg']}° > {args.max_arrival_deg}°")
            home()
            return "handoff_failed_arrival", None
        if not wrist_sees_block(wrist, color):
            print("  인계 판단 실패: 손목 카메라에 블록이 안 보임")
            home()
            return "handoff_failed_wrist", None

        runner.reset()
        q, g = arm.read()
        w, _ = wrist.latest_rgb()
        tp, _ = top.latest_rgb()
        first = runner.act(q, g, w, tp)      # 첫 묶음 예측 (이 값이 곧 처음 보낼 명령)
        gap = max(abs(float(first[n]) - float(v)) for n, v in zip(NAMES[:5], q))
        print(f"  정책 첫 명령과 지금 자세 차이 최대 {gap:.1f}°")
        runner.reset()                       # 실제 실행은 같은 관찰에서 다시 시작
        if gap > args.max_first_gap and (queue is not None or input(f"  {args.max_first_gap}°보다 큼. 그래도 실행? (y) ").strip().lower() != "y"):
            home()
            return "first_gap_refused", {"first_gap_deg": round(gap, 1)}

        step("정책 실행: 손을 리더암·전원 가까이. 멈추기 = n / q / Esc")
        mon = Monitor(q_home, g_open, g_closed, max_s=args.max_s, home_tol_deg=args.home_tol,
                      home_hold_s=args.home_hold_s, empty_hold_s=args.empty_hold_s, floor_mm=args.floor_mm)
        listener, events = init_keyboard_listener()
        try:
            res = run_episode(arm, runner, mon, wrist, top, events,
                              clearance=lambda act_: cmd_clearance(kin, calib, act_, p.block_mm))
        finally:
            if listener is not None:
                listener.stop()
            flush_stdin()
        stop_all["v"] = bool(events["stop_recording"])
        res["first_gap_deg"] = round(gap, 1)
        print(f"  멈춤: {END_KO[res['end']]} ({res['duration_s']}초, 평균 {res['mean_hz']}Hz, "
              f"잡음 {'예' if res['grasped_at_s'] is not None else '아니오'}, "
              f"명령 끝 높이 최소 테이블 위 {res['min_cmd_clear_mm']}mm)")
        if res["clipped_frames"]:
            print(f"  주의: {res['clipped_frames']}프레임이 한 번에 {args.max_step_deg}° 제한에 걸림 (정책이 급하게 움직이려 함)")
        if res["slow_frames"]:
            print(f"  주의: {res['slow_frames']}프레임이 늦음 (정책 계산 시간)")
        if res["end"] in ("empty", "dropped"):
            arm.set_gripper(g_open, 0.5)
            home()
        elif res["end"] != "home":
            if queue is None:
                input("  Enter = HOME으로 (그리퍼 그대로, 블록을 쥐고 있으면 HOME에서 놓음) ")
            home(arm.read()[1])
            arm.set_gripper(g_open, 0.5)
        # 정책이 끝낸 그리퍼 값을 기록한 뒤(시연처럼 스스로 닫았는지), 시연·순수 IK처럼 HOME에서 닫고 끝낸다.
        res["gripper_at_end"] = round(float(arm.read()[1]), 1)
        arm.set_gripper(g_home, 0.8)
        return f"policy_{res['end']}", res

    top = wrist = arm = None
    done, results = args.start, []
    try:
        top = LatestFrame(*open_role("top", WIDTH, HEIGHT, FPS))
        wrist = LatestFrame(*open_role("wrist", WIDTH, HEIGHT, FPS))
        arm = Arm(hold_on_exit=True, max_step_deg=args.max_step_deg)
        cp.print_rules()
        color = args.color
        plan = cp.schedule(args.trials)
        while queue is not None or done < args.trials:
            if queue is not None:
                color = queue.current()
                if color is None:
                    break
                print(f"\n[{queue.label()}]")
            grid = plan[done] if (args.grid and queue is None) else None
            where = grid or "작업 영역 아무 곳"
            ans = "" if queue is not None else input(f"\n[시행 {done + 1}/{args.trials}] 블록({color})을 {where}에 놓고 Enter "
                        f"(색 이름 = 바꾸기 / q = 끝) ").strip().lower()
            if ans == "q":
                break
            if ans:
                if ans not in colors:
                    print(f"모르는 색: {ans}. 가능: {', '.join(colors)}")
                    continue
                color = ans
            rec = {"method": "ik_act", "name": args.name, "trial": done + 1, "policy": args.policy, "color": color,
                   "time": time.strftime("%Y-%m-%d %H:%M:%S")}
            if grid is not None:
                rec.update(cp.precheck_place(top, top.gains, color, calib, grid, args.place_tol))
            t0 = time.monotonic()
            auto, res = run_trial(rec, color)
            no_block = auto == "approach_failed" and "검출하지 못했" in (res or {}).get("error", "")
            if queue is not None:
                ok = auto == "policy_home" and res.get("grasped_at_s") is not None
                queue.report("none" if no_block else ("ok" if ok else "fail"))
            if not (queue is not None and no_block):       # 큐에서 블록이 없어 넘어간 것은 기록하지 않음
                rec.update(res or {}, auto=auto, duration_s=round(time.monotonic() - t0, 1),
                           home_ok=bool(res) and res.get("end") == "home")
                frames = {"top": cv2.cvtColor(top.latest_rgb()[0], cv2.COLOR_RGB2BGR),
                          "wrist": cv2.cvtColor(wrist.latest_rgb()[0], cv2.COLOR_RGB2BGR)}
                rec["photos"] = cp.save_photos(cp.OUT_DIR / args.name, f"t{done + 1:02d}" + (f"_{grid}" if grid else ""), frames)
                print(f"  자동 기록: {auto}")
                rec.update(dict(cp.QUEUE_VERDICT) if queue is not None else cp.ask_verdict())
                cp.append(log_path, rec)
                results.append(rec["verdict"])
                done += 1
            if stop_all["v"] and confirm_stop():
                break
    finally:
        for c in (top, wrist):
            if c is not None:
                c.close()
        if arm is not None:
            arm.disconnect()
        if queue is not None:
            print("\n" + queue.summary())
        if results and log_path.exists():
            print("\n" + cp.report([str(log_path)]))


if __name__ == "__main__":
    main()
