"""ACT 녹화 루프 시험 (팔·카메라·영상 인코딩 없이)."""
import numpy as np
import pytest

pytest.importorskip("lerobot")
from kitbot_exec import record_act as ra


class FakeDataset:
    def __init__(self):
        self.features = ra.dataset_features()
        self.frames = []

    def add_frame(self, f):
        self.frames.append(f)


class Arm:
    def __init__(self):
        self.sent = []

    def read(self):
        return np.arange(5.0), 30.0

    def send_action(self, a):
        self.sent.append(a)


class Leader:
    def action(self):
        return {n: 7.0 for n in ra.NAMES}


class Cam:
    def __init__(self, missing=0):
        self.missing = missing

    def latest_rgb(self):
        if self.missing > 0:
            self.missing -= 1
            return None, np.inf
        return np.zeros((ra.HEIGHT, ra.WIDTH, 3), np.uint8), 0.02


def events():
    return {"exit_early": False, "rerecord_episode": False, "stop_recording": False}


def test_features_have_state_action_and_two_cameras():
    f = ra.dataset_features()
    assert f["action"]["names"] == ra.NAMES and f["observation.state"]["names"] == ra.NAMES
    assert f["observation.images.wrist"]["shape"] == (ra.HEIGHT, ra.WIDTH, 3)
    assert f["observation.images.top"]["dtype"] == "video"


def test_episode_records_follower_state_and_leader_action_at_rate():
    ds, arm = FakeDataset(), Arm()
    n, hz, age = ra.record_episode(arm, Leader(), Cam(), Cam(), ds, events(), 0.3)
    assert n == len(ds.frames) and 6 <= n <= 10 and hz > 20
    fr = ds.frames[0]
    assert fr["observation.state"].tolist() == [0, 1, 2, 3, 4, 30]       # 팔로워 실제
    assert fr["action"].tolist() == [7.0] * 6                              # 리더 = 학습 목표
    assert fr["task"] == ra.TASK and fr["observation.images.top"].shape == (ra.HEIGHT, ra.WIDTH, 3)
    assert len(arm.sent) >= n                                              # 매 주기 팔로워에 보냄


def test_exit_early_stops_and_missing_frames_are_skipped():
    ds = FakeDataset()
    ev = events()
    ev["exit_early"] = True
    n, _, _ = ra.record_episode(Arm(), Leader(), Cam(), Cam(), ds, ev, 5.0)
    assert n == 0 and ev["exit_early"] is False
    n, _, _ = ra.record_episode(Arm(), Leader(), Cam(missing=3), Cam(), ds, events(), 0.2)
    assert n == len(ds.frames) and n > 0


def test_records_the_action_actually_sent_when_clipped():
    class ClipArm(Arm):
        def send_action(self, a):
            super().send_action(a)
            return {n: 3.0 for n in ra.NAMES}          # 안전 제한으로 잘린 값

    ds = FakeDataset()
    ra.record_episode(ClipArm(), Leader(), Cam(), Cam(), ds, events(), 0.1)
    assert ds.frames and ds.frames[0]["action"].tolist() == [3.0] * 6


# ---------- 이어서 녹화: 반반 흔들기·설정 검사 ----------
def _rec(i, jit, mm=10.0, deg=10.0, h=15.0):
    return {"episode_index": i, "jitter_range": [mm, deg] if jit else [0.0, 0.0], "handoff_clear_mm": h}


def test_balanced_jitter_stays_half_across_sessions_and_failures():
    import numpy as np
    rng = np.random.default_rng(3)
    meta = []
    for session in (25, 25):                    # 25개씩 두 번에 나눠 녹화
        saved = 0
        while saved < session:
            jit = ra.choose_jitter(meta, "balanced", 0.5, rng)
            if rng.uniform() < 0.2:             # 접근 실패·버림 → 저장 안 됨
                continue
            meta.append(_rec(len(meta), jit))
            saved += 1
            j, n = ra.jitter_counts(meta)
            assert abs(j - n) <= 1
    assert ra.jitter_counts(meta) == (25, 25)


def test_random_mode_uses_probability():
    import numpy as np
    rng = np.random.default_rng(0)
    assert not any(ra.choose_jitter([], "random", 0.0, rng) for _ in range(20))
    assert all(ra.choose_jitter([], "random", 1.0, rng) for _ in range(20))


def test_settings_conflict_detects_changed_jitter_or_handoff():
    meta = [_rec(0, False), _rec(1, True)]
    assert ra.settings_conflict(meta, 10, 10, 15) is None
    assert "흔들기" in ra.settings_conflict(meta, 15, 10, 15)
    assert "넘기는 높이" in ra.settings_conflict(meta, 10, 10, 0)
    assert ra.settings_conflict([], 15, 15, 0) is None


def test_read_meta_keeps_only_saved_episodes(tmp_path):
    import json
    p = tmp_path / ra.META_NAME
    p.write_text("".join(json.dumps(_rec(i, i % 2)) + "\n" for i in range(4)) + "\n")
    assert len(ra.read_meta(p)) == 4
    assert [r["episode_index"] for r in ra.read_meta(p, 3)] == [0, 1, 2]
    assert ra.read_meta(tmp_path / "없음.jsonl") == []


# ---------- 리더암 손 맞춤 ----------
class _Arm:
    def read(self):
        return np.array([10.0, -20.0, 30.0, 5.0, -3.0]), 33.8


class _Leader:
    """호출할 때마다 리더암이 팔로워 쪽으로 조금씩 다가간다 (사람이 맞추는 중)."""
    def __init__(self, start_off=6.0, step=1.0):
        self.off, self.step = start_off, step

    def action(self):
        q, g = _Arm().read()
        a = {n: float(v) + self.off for n, v in zip(ra.NAMES, list(q) + [g])}
        self.off = max(self.off - self.step, 0.5)
        return a


class _Keys:
    def __init__(self, seq):
        self.seq = list(seq)

    def get(self, timeout_s):
        return self.seq.pop(0) if self.seq else None


class _Clock:
    def __init__(self, dt=0.1):
        self.t, self.dt = 0.0, dt

    def __call__(self):
        self.t += self.dt
        return self.t


def test_leader_need_is_follower_minus_leader_and_ok_threshold():
    need = ra.leader_need([1, 2, 3, 4, 5], 30.0, {n: 0.0 for n in ra.NAMES})
    assert need == {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 30}
    assert ra.gaps_ok({"elbow_flex": 2.9, "gripper": 4.9}, 3, 5) and not ra.gaps_ok({"elbow_flex": 3.1}, 3, 5)
    assert "elbow  +3.1*" in ra.format_need({"elbow_flex": 3.1}, 3, 5)


def test_enter_starts_only_when_within_tolerance():
    out = []
    # 처음 차이 6 → Enter(거부) → … 차이 0.5까지 줄어든 뒤 Enter(시작)
    keys = _Keys(["\n", None, None, None, None, None, "\n"])
    choice, need, _ = ra.fine_tune_leader(_Arm(), _Leader(), 3.0, 3.0, keys, clock=_Clock(),
                                          show=lambda *a, **k: out.append(a[0]))
    assert choice == "start" and max(abs(v) for v in need.values()) <= 3.0
    assert any("f = 그래도 시작" in o for o in out)


@pytest.mark.parametrize("key,choice", [("f", "force"), ("a", "realign"), ("S", "skip")])
def test_other_keys(key, choice):
    got, need, _ = ra.fine_tune_leader(_Arm(), _Leader(step=0), 3.0, 3.0, _Keys([key]), clock=_Clock(),
                                       show=lambda *a, **k: None)
    assert got == choice and abs(need["elbow_flex"]) == pytest.approx(6.0)


def test_auto_start_after_holding_within_tolerance():
    choice, need, t = ra.fine_tune_leader(_Arm(), _Leader(), 3.0, 3.0, _Keys([]), auto_start_s=1.0,
                                          clock=_Clock(0.1), show=lambda *a, **k: None)
    assert choice == "auto" and max(abs(v) for v in need.values()) <= 3.0
