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
