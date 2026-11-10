"""exp_s02 접근 시행 흐름을 가짜 팔·가짜 모델로 확인 (장비·placo 없이)."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from kitbot_exec import exp_s02 as ex
from kitbot_exec import wrist_measure as wm
from kitbot_exec.correction import Correction, load_samples
from kitbot_exec.pick import PickParams, PickPlan

R_DOWN = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])   # 아래를 보는 그리퍼


class FakeKin:
    def fk(self, q):
        q = np.asarray(q, float)
        return np.array([q[0] * 10.0, q[1] * 10.0, 50.0 + q[2]]), R_DOWN

    def _tool_yaw(self, q):
        return float(q[4])


def test_collect_uses_command_basis_even_with_undershoot(patched, monkeypatch, tmp_path):
    # 팔이 덜 가도(읽은 값 ≠ 명령값) 기록은 명령값 기준이어야 다음 실행에서 맞는다
    monkeypatch.setattr(ex.wm, "observe", lambda frames, color: wm.WristObs((320.0, 240.0), 0.0, 150.0, 10))
    ctx = make_ctx([0.0, 1.0, 0.0, 0.0, 0.0])
    rec = ex.approach_trial(ctx, "u", Correction([]), ref(), tmp_path / "c.json")
    s = load_samples(tmp_path / "c.json")[0]
    jaw_t = ex.jaw_center_xy(FakeKin(), patched, PickParams().jaw_center_mm)
    assert np.allclose(s["measured"][:2], jaw_t, atol=0.02)                 # 사진 오차 0 → 명령 위치 그대로
    assert s["block_true_read_xy"] != _round(s["measured"][:2])             # 실제(읽은 값) 기준과는 다름


def _round(v):
    return [round(float(x), 1) for x in v]


class FakeArm:
    """명령한 관절값에서 undershoot만큼 덜 가서 멈추는 팔."""

    def __init__(self, undershoot):
        self.q = np.zeros(5)
        self.under = np.asarray(undershoot, float)
        self.moves = []

    def move_joints(self, q, g=None, speed=40.0):
        self.q = np.asarray(q, float)
        self.moves.append(self.q.copy())

    def read(self):
        return self.q - self.under, 30.0


def make_ctx(undershoot, step=False):
    return SimpleNamespace(p=PickParams(), kin=FakeKin(), arm=FakeArm(undershoot), top=None, top_gains=None,
                           wrist=None, wrist_gains=None, calib=None, q_home=np.zeros(5), g_open=30.0,
                           args=SimpleNamespace(color="purple", step=step),
                           step=lambda msg: None)


@pytest.fixture
def patched(monkeypatch):
    q_pre = np.array([20.0, 1.0, 0.0, 80.0, 10.0])
    monkeypatch.setattr(ex, "detect_block_pose", lambda *a, **k: ((200.0, 10.0, 25.0, 10.0), None, "ok"))
    monkeypatch.setattr(ex, "plan_pick", lambda *a, **k: PickPlan(True, "", {}, q_pre, [], [], {"z_pre": 65.0}))
    monkeypatch.setattr(ex, "grab", lambda *a, **k: [])
    monkeypatch.setattr(ex, "save_frame", lambda *a, **k: None)
    return q_pre


def ref():
    return wm.WristRef((320.0, 240.0), 0.0, 150.0, [[6.0, 0.0], [0.0, 6.0]], -1.0, 90.0)


def test_trial_records_undershoot_timeout_and_wrist_error(patched, monkeypatch):
    e_tool = np.array([3.0, -2.0])
    r = ref()
    obs = wm.WristObs(tuple(np.asarray(r.c_ref) - np.asarray(r.J) @ e_tool), 0.0, 150.0, 10)
    monkeypatch.setattr(ex.wm, "observe", lambda frames, color: obs)
    ctx = make_ctx([0.0, 2.5, 0.0, 0.0, 0.0])
    rec = ex.approach_trial(ctx, "mid_r1", Correction([]), r, None)
    assert rec["result"] == "approached"
    assert rec["diag"]["settle_timeout"] is True and rec["diag"]["max_abs_deg"] == 2.5
    assert rec["diag"]["jaw_target_minus_read_xy_mm_model"] == [0.0, 25.0]   # 어깨 2.5° → 모델상 25mm (가짜 모델)
    assert rec["wrist"]["err_tool_mm"] == [3.0, -2.0]
    assert rec["wrist"]["err_robot_mm"] == [3.0, 2.0]                          # R_DOWN: y축 뒤집힘
    assert rec["wrist"]["err_yaw_deg"] == 0.0 and rec["wrist"]["height_offset_mm"] == 0.0
    assert np.allclose(ctx.arm.moves[-1], ctx.q_home)                          # 끝나면 HOME


def test_trial_eval_never_writes_corrections(patched, monkeypatch, tmp_path):
    monkeypatch.setattr(ex.wm, "observe", lambda frames, color: wm.WristObs((326.0, 240.0), 0.0, 150.0, 10))
    corr_file = tmp_path / "c.json"
    corr_file.write_text("[]")
    ctx = make_ctx(np.zeros(5))
    ex.approach_trial(ctx, "a", Correction(load_samples(corr_file)), ref(), None)
    assert json.loads(corr_file.read_text()) == []
    rec = ex.approach_trial(ctx, "b", Correction([]), ref(), tmp_path / "collect.json")
    samples = load_samples(tmp_path / "collect.json")
    assert len(samples) == 1 and samples[0]["trial"] == "b" and samples[0]["basis"] == "command"
    # 보정 없이 모으면 기록되는 차이 d = (명령한 부리 가운데 + 사진 오차) − 카메라 값 = 사진 오차 + 모델 계산 차이
    jaw_t = ex.jaw_center_xy(FakeKin(), patched, PickParams().jaw_center_mm)
    e_rob = np.array(rec["wrist"]["err_robot_mm"])
    assert np.allclose(samples[0]["measured"][:2], jaw_t + e_rob, atol=0.02)
    assert rec["wrist"]["collected_to"] == "collect.json"


def test_trial_without_block_or_plan(patched, monkeypatch):
    ctx = make_ctx(np.zeros(5))
    monkeypatch.setattr(ex, "detect_block_pose", lambda *a, **k: (None, None, "없음"))
    assert ex.approach_trial(ctx, "n", Correction([]), ref(), None)["result"] == "no_block"
    monkeypatch.setattr(ex, "detect_block_pose", lambda *a, **k: ((200.0, 0.0, 25.0, 0.0), None, "ok"))
    monkeypatch.setattr(ex, "plan_pick", lambda *a, **k: PickPlan(False, "닿지 않음", {}))
    rec = ex.approach_trial(ctx, "p", Correction([]), ref(), None)
    assert rec["result"] == "plan_failed" and len(ctx.arm.moves) == 2   # HOME 이동만, 접근 안 함


def test_trial_wrist_not_visible(patched, monkeypatch):
    monkeypatch.setattr(ex.wm, "observe", lambda frames, color: None)
    rec = ex.approach_trial(make_ctx(np.zeros(5)), "v", Correction([]), ref(), None)
    assert rec["result"] == "approached" and "wrist" not in rec


def test_report_cli(tmp_path, capsys):
    t = {"name": "g1_r1", "result": "approached", "target_xy": [200, 0],
         "diag": {"target_minus_read_deg": [0, 1, 0, 0, 0], "settle_timeout": False},
         "wrist": {"err_robot_mm": [4, 3], "err_yaw_deg": 2.0, "height_offset_mm": 1.0}}
    ex.append_jsonl(tmp_path / "trials.jsonl", t)
    ex.append_jsonl(tmp_path / "trials.jsonl", {**t, "name": "g1_r2"})
    ex.main(["--run", str(tmp_path), "report"])
    out = json.loads((tmp_path / "report.json").read_text())
    assert out["approach"]["pass"] == "2/2" and out["approach"]["repeat"]["g1"]["spread_mm"] == 0.0
