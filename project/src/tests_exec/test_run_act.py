"""로봇·학습된 정책 없이 실행 도구(run_act)의 멈춤 판정과 실행 루프를 검사한다."""
from types import SimpleNamespace

import numpy as np
import pytest

from kitbot_exec import run_act
from kitbot_exec.record_act import NAMES

HOME = np.zeros(5)
START = np.full(5, 30.0)          # 인계 자세 (HOME에서 멀다)
G_OPEN, G_CLOSED = 33.8, 1.3


def mon(**kw):
    return run_act.Monitor(HOME, G_OPEN, G_CLOSED, **kw)


def feed(m, seq, dt=1 / 30):
    """seq: (q, 그리퍼 측정, 그리퍼 명령, 끝 높이) 반복. 멈춘 이유와 그때 시간."""
    for i, (q, gm, gc, c) in enumerate(seq):
        r = m.update(i * dt, q, gm, gc, c)
        if r:
            return r, i * dt
    return None, None


def test_empty_when_gripper_shuts_fully():
    m = mon()
    r, t = feed(m, [(START, 33.8, 33.8, 20)] * 10 + [(START, 1.5, 0.0, 10)] * 30)
    assert r == "empty" and m.grasped_at is None
    assert t == pytest.approx(10 / 30 + 0.5, abs=0.05)


def test_partial_close_command_near_block_width_is_not_mistaken_for_held():
    # 정책이 블록 폭 근처(10)까지만 닫으라 해서 측정도 10이면 → 블록 없음
    m = mon()
    r, _ = feed(m, [(START, 10.0, 10.0, 10)] * 30)
    assert r == "empty"


def test_grasp_carry_release_then_home():
    m = mon()
    seq = ([(START, 33.8, 33.8, 15)] * 5            # 열린 채 보정
           + [(START, 12.0, 0.0, 12)] * 20          # 닫음 → 블록 폭 12에서 멈춤 = 잡음
           + [(START * 0.5, 12.0, 0.0, 80)] * 10    # 옮기기
           + [(START * 0.5, 33.8, 33.8, 80)] * 5    # 놓기
           + [(START * 0.5, 1.3, 0.0, 80)] * 30     # 놓은 뒤 다시 닫아도 빈손 아님
           + [(HOME + 2, 1.3, 0.0, 150)] * 40)      # HOME
    r, _ = feed(m, seq)
    assert r == "home"
    assert m.grasped_at is not None and m.released_at is not None and m.q_release == [15.0] * 5


def test_dropped_while_carrying():
    m = mon()
    r, _ = feed(m, [(START, 12.0, 0.0, 12)] * 20 + [(START, 1.4, 0.0, 60)] * 30)
    assert r == "dropped"


def test_floor_and_timeout():
    m = mon(floor_mm=10)
    assert feed(m, [(START, 33.8, 33.8, -9.9)] * 3 + [(START, 33.8, 33.8, -10.1)])[0] == "floor"
    assert m.summary()["min_cmd_clear_mm"] == -10.1
    m = mon(max_s=1.0)
    assert feed(m, [(START, 33.8, 33.8, 20)] * 40)[0] == "timeout"


def test_home_needs_to_stay():
    m = mon(home_hold_s=1.0)
    near, far = (HOME + 1, 33.8, 33.8, 100), (START, 33.8, 33.8, 100)
    assert feed(m, ([near] * 20 + [far]) * 3)[0] is None     # 잠깐씩 지나가기만 하면 끝이 아님


class FakeCam:
    def latest_rgb(self):
        return np.zeros((4, 4, 3), np.uint8), 0.01


class FakeArm:
    def __init__(self):
        self.q, self.g, self.sent = START.copy(), G_OPEN, []

    def read(self):
        return self.q.copy(), self.g

    def send_action(self, a):
        self.sent.append(a)
        return a


class ScriptRunner:
    def __init__(self, actions):
        self.actions = list(actions)

    def act(self, q, g, w, t):
        return self.actions.pop(0)


def act(q, g):
    return {n: float(v) for n, v in zip(NAMES, list(q) + [g])}


def test_floor_command_is_never_sent(monkeypatch):
    arm = FakeArm()
    acts = [act(START, G_OPEN)] * 3 + [act(START - 50, G_OPEN)]
    clear = lambda a: -20.0 if a["shoulder_pan.pos"] < 0 else 20.0
    events = {"exit_early": False}
    res = run_act.run_episode(arm, ScriptRunner(acts), mon(), FakeCam(), FakeCam(), events, clearance=clear, hz=1000)
    assert res["end"] == "floor" and len(arm.sent) == 3
    assert all(a["shoulder_pan.pos"] > 0 for a in arm.sent)


def test_key_stops_before_next_command():
    arm = FakeArm()
    events = {"exit_early": True}
    res = run_act.run_episode(arm, ScriptRunner([]), mon(), FakeCam(), FakeCam(), events, hz=1000)
    assert res["end"] == "key" and arm.sent == []


def _fake_policy(inputs):
    cfg = SimpleNamespace(input_features={k: None for k in inputs},
                          output_features={"action": SimpleNamespace(shape=(6,))})
    return SimpleNamespace(config=cfg)


def test_runner_passes_only_inputs_the_policy_uses():
    r = run_act.PolicyRunner(_fake_policy(["observation.state", "observation.images.wrist"]), None, None, "cpu")
    w = np.ones((480, 640, 3), np.uint8)
    obs = r.obs_frame(np.arange(5.0), 9.0, w, w * 2)
    assert set(obs) == {"observation.state", "observation.images.wrist"}
    np.testing.assert_allclose(obs["observation.state"], [0, 1, 2, 3, 4, 9])


def test_runner_rejects_inputs_we_cannot_make():
    with pytest.raises(ValueError, match="observation.images.front"):
        run_act.PolicyRunner(_fake_policy(["observation.state", "observation.images.front"]), None, None, "cpu")


def test_cmd_clearance_reads_policy_action_keys():
    # 실물 첫 실행에서 'shoulder_pan.pos.pos' KeyError (2026-11-12). 실제 FK·보정 파일로 확인
    from kitbot_exec.calib import PixelToRobot
    from kitbot_exec.kinematics import SO101Kinematics
    from kitbot_exec.pick import CALIB

    kin, calib = SO101Kinematics(), PixelToRobot.load(CALIB)
    q = [-49.06, -24.55, 49.46, 65.09, 35.41]                     # 실물 인계 자세 (블록 윗면 위 15mm 명령)
    a = {n: float(v) for n, v in zip(NAMES, q + [33.8])}
    c = run_act.cmd_clearance(kin, calib, a, 25.0)
    assert 30 < c < 50                                             # 블록 윗면(테이블 + 25) 위 약 15mm → 테이블 위 약 40mm


def test_main_one_full_trial_with_fake_hardware(tmp_path, monkeypatch):
    """run_act.main 한 시행 전체 (접근 → 정책 → HOME → 사진 → 판정 → 기록). 로봇·카메라·정책만 가짜."""
    from unittest.mock import patch

    from kitbot_exec import compare as cp

    monkeypatch.setattr(cp, "OUT_DIR", tmp_path)
    poses = {"home": {"q_deg": [0.0] * 5, "gripper": 1.3}, "gripper_pregrasp": {"gripper": 33.8},
             "gripper_closed": {"gripper": 1.3}}
    q_hand = [-49.06, -24.55, 49.46, 65.09, 35.41]

    class Arm:
        def __init__(self, **k):
            self.q, self.g, self.grip_cmds = np.array(q_hand), 33.8, []

        def read(self):
            return self.q.copy(), self.g

        def send_action(self, a):
            self.q = np.array([a[n] for n in NAMES[:5]]); self.g = a["gripper.pos"]
            return a

        def move_joints(self, q, g=None, speed=40):
            self.q = np.asarray(q, float)
            if g is not None:
                self.g = g
            return True

        def set_gripper(self, v, wait_s=1.0):
            self.grip_cmds.append(v); self.g = v
            return v

        def disconnect(self):
            pass

    arm = Arm()

    class Frame:
        gains = None

        def __init__(self, *a):
            pass

        def latest_rgb(self):
            return np.zeros((480, 640, 3), np.uint8), 0.01

        def close(self):
            pass

    # 정책: 블록 위에서 몇 프레임 → HOME으로 (끝 판정 'home')
    seq = [q_hand] * 5 + [[0.0] * 5] * 60

    class Runner:
        inputs = {"observation.state"}

        def __init__(self, *a):
            self.i = 0

        def reset(self):
            self.i = 0

        def act(self, q, g, w, t):
            qq = seq[min(self.i, len(seq) - 1)]; self.i += 1
            return {n: float(v) for n, v in zip(NAMES, list(qq) + [33.8])}

    rec0 = {"block_camera": [129.0, 115.0, 4.0, 10.0], "max_joint_err_deg": 0.49, "plan": None}
    fake_pol = type("P", (), {"config": type("C", (), {"type": "act"})()})()
    answers = iter(["", "s 테스트"])
    monkeypatch.setattr("builtins.input", lambda *_: next(answers))
    with patch.object(run_act, "load_policy", return_value=(fake_pol, None, None, "cpu")), \
         patch.object(run_act, "PolicyRunner", Runner), \
         patch.object(run_act, "LatestFrame", Frame), \
         patch.object(run_act, "approach_block", return_value=dict(rec0)), \
         patch.object(run_act, "wrist_sees_block", return_value=True), \
         patch("kitbot_exec.arm.Arm", return_value=arm), \
         patch("kitbot_exec.arm.load_poses", return_value=poses), \
         patch("kitbot_exec.camera.open_role", return_value=(None, None)), \
         patch("lerobot.utils.keyboard_input.init_keyboard_listener",
               return_value=(None, {"exit_early": False, "stop_recording": False})):
        run_act.main(["--policy", "x", "--name", "t", "--trials", "1"])
    rows = cp.read(tmp_path / "t.jsonl")
    assert len(rows) == 1 and rows[0]["verdict"] == "s" and rows[0]["auto"] == "policy_home" and rows[0]["home_ok"]
    assert rows[0]["method"] == "ik_act" and rows[0]["nearest_demo_mm"] is not None
    assert arm.grip_cmds[-1] == 1.3                                # 끝에 HOME에서 그리퍼 닫기
    assert any((tmp_path / "t").glob("*_t01_top.jpg"))


def test_run_diffusion_fills_defaults_and_checks_n_action():
    from kitbot_exec import run_diffusion as rd
    a = rd.build_argv(["--color", "blue"])
    assert a[a.index("--policy") + 1] == rd.POLICY and a[a.index("--max-s") + 1] == "60" and "diffusion_20k" in a
    assert rd.build_argv(["--name", "x", "--max-s", "40"]).count("--name") == 1
    with pytest.raises(SystemExit):
        rd.build_argv(["--n-action-steps", "16"])


def test_main_queue_mode_runs_until_color_is_gone(tmp_path, monkeypatch):
    """--queue: 묻지 않고 이어서, 같은 색은 가까운 것부터(nearest), 안 보이면 다음 명령."""
    from unittest.mock import patch

    from kitbot_exec import compare as cp

    monkeypatch.setattr(cp, "OUT_DIR", tmp_path)
    monkeypatch.setattr("builtins.input", lambda *_: pytest.fail("큐 실행은 묻지 않아야 함"))
    poses = {"home": {"q_deg": [0.0] * 5, "gripper": 1.3}, "gripper_pregrasp": {"gripper": 33.8},
             "gripper_closed": {"gripper": 1.3}}
    q_hand = [-49.06, -24.55, 49.46, 65.09, 35.41]

    class Arm:
        def __init__(self):
            self.q, self.g = np.array(q_hand), 33.8

        def read(self):
            return self.q.copy(), self.g

        def send_action(self, a):
            self.q = np.array([a[n] for n in NAMES[:5]])
            # 닫으라는 명령이면 블록 폭(12)에서 멈춘 것으로 (쥠)
            self.g = 12.0 if a["gripper.pos"] < 10 else a["gripper.pos"]
            return a

        def move_joints(self, q, g=None, speed=40):
            self.q = np.asarray(q, float)
            if g is not None:
                self.g = g
            return True

        def set_gripper(self, v, wait_s=1.0):
            self.g = v
            return v

        def disconnect(self):
            pass

    class Frame:
        gains = None

        def __init__(self, *a):
            pass

        def latest_rgb(self):
            return np.zeros((480, 640, 3), np.uint8), 0.01

        def close(self):
            pass

    # 정책: 쥐기(닫음) 25프레임 → 열고 HOME
    seq = [(q_hand, 2.0)] * 25 + [([0.0] * 5, 33.8)] * 60

    class Runner:
        inputs = {"observation.state"}

        def __init__(self, *a):
            self.i = 0

        def reset(self):
            self.i = 0

        def act(self, q, g, w, t):
            qq, gg = seq[min(self.i, len(seq) - 1)]; self.i += 1
            return {n: float(v) for n, v in zip(NAMES, list(qq) + [gg])}

    table = {"orange": 1, "blue": 2}
    seen = []

    def fake_approach(arm, *a, color, nearest, **k):
        seen.append((color, nearest))
        if table[color] == 0:
            raise RuntimeError("블록을 검출하지 못했습니다: 'x' 블록이 안 보임")
        table[color] -= 1
        arm.q, arm.g = np.array(q_hand), 33.8
        return {"block_camera": [129.0, 115.0, 4.0, 10.0], "max_joint_err_deg": 0.4, "plan": None}

    fake_pol = type("P", (), {"config": type("C", (), {"type": "act"})()})()
    with patch.object(run_act, "load_policy", return_value=(fake_pol, None, None, "cpu")), \
         patch.object(run_act, "PolicyRunner", Runner), \
         patch.object(run_act, "LatestFrame", Frame), \
         patch.object(run_act, "approach_block", side_effect=fake_approach), \
         patch.object(run_act, "wrist_sees_block", return_value=True), \
         patch("kitbot_exec.arm.Arm", return_value=Arm()), \
         patch("kitbot_exec.arm.load_poses", return_value=poses), \
         patch("kitbot_exec.camera.open_role", return_value=(None, None)), \
         patch("lerobot.utils.keyboard_input.init_keyboard_listener",
               return_value=(None, {"exit_early": False, "stop_recording": False})):
        run_act.main(["--policy", "x", "--name", "q", "--queue", "orange 2, blue", "--home-hold-s", "0.2"])
    assert seen == [("orange", True), ("orange", True), ("blue", True), ("blue", True), ("blue", True)]
    rows = cp.read(tmp_path / "q.jsonl")
    assert [r["color"] for r in rows] == ["orange", "blue", "blue"]
    assert all(r["auto"] == "policy_home" and r["grasped_at_s"] is not None for r in rows)
