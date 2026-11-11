"""로봇 연결 없이 접근 전용 실행 흐름을 검사한다."""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from kitbot_exec import approach


@pytest.fixture(autouse=True)
def fast_still(monkeypatch):
    monkeypatch.setattr(approach, "STILL_WINDOW_S", 0.01)
    monkeypatch.setattr(approach, "STILL_TIMEOUT_S", 0.2)


@pytest.mark.parametrize("failure", [None, "home", "detection", "plan", "move", "arrival"])
def test_approach_stops_at_target_or_failure(failure, capsys):
    arm, cap = Mock(), Mock()
    kin = Mock()
    kin.fk.return_value = (np.array([200., 0., 65.]), np.eye(3))
    arm.move_joints.side_effect = [failure != "home", failure != "move"]
    arm.read.return_value = (np.full(5, 6 if failure == "arrival" else 1), 50)
    poses = {"home": {"q_deg": [0] * 5}, "gripper_pregrasp": {"gripper": 50},
             "gripper_closed": {"gripper": 0}}
    corr = Mock()
    corr.apply.return_value = (200, 0, 0)
    plan = SimpleNamespace(ok=failure != "plan", reason="unreachable", q_pre=np.ones(5))
    pose = None if failure == "detection" else (200, 0, 25, 0)
    with patch.object(approach, "detect_block_pose", return_value=(pose, None, "test")), \
         patch.object(approach, "plan_pick", return_value=plan):
        if failure:
            with pytest.raises(RuntimeError):
                approach.approach_block(arm, cap, None, kin, Mock(), poses, corr)
        else:
            assert approach.approach_block(arm, cap, None, kin, Mock(), poses, corr)["plan"] is plan
    expected_moves = 1 if failure in ("home", "detection", "plan") else 2
    assert arm.move_joints.call_count == expected_moves
    for call in arm.move_joints.call_args_list:
        assert call.kwargs["settle_tol_deg"] == 5.0
    assert arm.move_joints.call_args_list[0].args[1] == 0
    if expected_moves == 2:
        np.testing.assert_array_equal(arm.move_joints.call_args.args[0], plan.q_pre)
        assert arm.move_joints.call_args.args[1] == 50
    arm.follow.assert_not_called()
    arm.disconnect.assert_not_called()  # 연결을 만든 호출자가 해제를 담당한다.
    assert ("이동이 끝났습니다." in capsys.readouterr().out) == (failure is None)


def test_arrival_reports_signed_joint_and_position_errors(capsys):
    kin = Mock()
    kin.fk.side_effect = [(np.array([200., 10., 65.]), np.eye(3)),
                          (np.array([197., 12., 59.]), np.eye(3))]
    error = approach.report_arrival(kin, [0, 5, 0, 0, 0], [0, 2, 1, 0, 0], (-22., 0.))
    out = capsys.readouterr().out
    assert error == 3
    assert "shoulder_lift: 목표 5.00°, 현재 2.00°, 차이 +3.00° [허용 범위 안]" in out
    assert "elbow_flex: 목표 0.00°, 현재 1.00°, 차이 -1.00°" in out
    assert "dx=+3.00, dy=-2.00, dz=+6.00" in out


def test_arrival_rejects_invalid_reading():
    kin = Mock()
    with pytest.raises(RuntimeError, match="관절값"):
        approach.report_arrival(kin, np.zeros(5), [0, 0, float('nan'), 0, 0], (-22., 0.))
    kin.fk.assert_not_called()


def test_relaxed_arrival_threshold(capsys):
    kin = Mock()
    kin.fk.return_value = (np.zeros(3), np.eye(3))
    approach.report_arrival(kin, [4.9, 5.0, 0, 0, 0], np.zeros(5), (-22., 0.))
    out = capsys.readouterr().out
    assert "차이 +4.90° [허용 범위 안]" in out
    assert "차이 +5.00° [허용 범위 밖]" in out


def test_default_is_no_correction():
    assert approach.build_correction(None).mode == "none"
    assert approach.build_correction("none").mode == "none"


def test_jitter_stays_in_range_and_is_recorded():
    rng = np.random.default_rng(0)
    for _ in range(200):
        x, y, yaw, dx, dy, dyaw = approach.jitter_target(rng, 100.0, 50.0, 10.0, 15.0, 15.0)
        assert np.hypot(dx, dy) <= 15.0 + 1e-9 and abs(dyaw) <= 15.0
        assert (x, y) == pytest.approx((100.0 + dx, 50.0 + dy))
    assert approach.jitter_target(rng, 1.0, 2.0, 3.0, 0, 0) == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)


def _ok_setup():
    arm, cap, kin = Mock(), Mock(), Mock()
    kin.fk.return_value = (np.array([200., 0., 65.]), np.eye(3))
    arm.move_joints.return_value = True
    arm.read.return_value = (np.ones(5), 50)
    poses = {"home": {"q_deg": [0] * 5}, "gripper_pregrasp": {"gripper": 50}, "gripper_closed": {"gripper": 0}}
    corr = approach.Correction([])
    calib = Mock()
    calib.z_at.return_value = np.array([25.0])
    return arm, cap, kin, poses, corr, calib


def test_handoff_descends_vertically_after_approach():
    arm, cap, kin, poses, corr, calib = _ok_setup()
    plan = SimpleNamespace(ok=True, reason="", q_pre=np.ones(5))
    path = [np.ones(5), np.ones(5)]
    with patch.object(approach, "detect_block_pose", return_value=((200, 0, 25, 0), None, "t")), \
         patch.object(approach, "plan_pick", return_value=plan), \
         patch("kitbot_exec.exp_s02.line_path", return_value=path) as lp:
        rec = approach.approach_block(arm, cap, None, kin, calib, poses, corr, handoff_clear_mm=15.0)
    arm.follow.assert_called_once()
    assert lp.call_args.args[5] == pytest.approx(25.0 + 15.0)        # 블록 윗면 + 15mm까지
    assert rec["handoff_clear_mm"] == 15.0 and rec["jitter"] == [0.0, 0.0, 0.0]


def test_handoff_refuses_below_block_top():
    arm, cap, kin, poses, corr, calib = _ok_setup()
    with pytest.raises(ValueError):
        approach.approach_block(arm, cap, None, kin, calib, poses, corr, handoff_clear_mm=0.0)
    arm.move_joints.assert_not_called()


def test_jitter_resamples_when_plan_fails():
    arm, cap, kin, poses, corr, calib = _ok_setup()
    bad = SimpleNamespace(ok=False, reason="far", q_pre=None)
    good = SimpleNamespace(ok=True, reason="", q_pre=np.ones(5))
    with patch.object(approach, "detect_block_pose", return_value=((200, 0, 25, 0), None, "t")), \
         patch.object(approach, "plan_pick", side_effect=[bad, good]):
        rec = approach.approach_block(arm, cap, None, kin, calib, poses, corr, jitter_mm=15, jitter_deg=15,
                                      rng=np.random.default_rng(1))
    assert rec["plan"] is good and np.hypot(*rec["jitter"][:2]) <= 15.0


class FakeBus:
    def __init__(self):
        self.pos = {n: 0.0 for n in list(approach.ARM_JOINTS) + ["gripper"]}
        self.limits = {n: 1000 for n in self.pos}
        self.log = []
        self.torque = False

    def sync_read(self, name, normalize=True):
        return dict(self.pos) if name == "Present_Position" else dict(self.limits)

    def sync_write(self, name, values, normalize=True):
        self.log.append((name, self.torque, dict(values)))
        if name == "Goal_Position" and self.torque:
            self.pos.update(values)
        if name == "Torque_Limit":
            self.limits.update(values)

    def enable_torque(self):
        self.torque = True
        self.log.append(("enable", True, None))

    def disable_torque(self):
        self.torque = False
        self.log.append(("disable", False, None))


def test_align_leader_holds_start_then_moves_then_releases_and_restores_limit():
    bus = FakeBus()
    leader = SimpleNamespace(teleop=SimpleNamespace(bus=bus))
    q = [10.0, -20.0, 30.0, 5.0, -3.0]
    out = approach.align_leader(leader, q, 33.8, confirm=lambda: None, sleep=lambda s: None)
    names = [e[0] for e in bus.log]
    assert names[0] == "Goal_Position" and bus.log[0][2]["elbow_flex"] == 0.0   # 힘 켜기 전 목표 = 지금 자리
    assert names.index("enable") < names.index("disable")
    assert bus.limits["elbow_flex"] == 1000                                      # 힘 한도 원래대로
    assert out["gap_before_deg"]["elbow_flex"] == 30.0
    assert out["max_gap_released_deg"] == pytest.approx(0.0, abs=1e-6)
