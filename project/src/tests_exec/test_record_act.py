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
