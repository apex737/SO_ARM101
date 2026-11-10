"""기구학 시험 (모델 안에서의 계산 검사). 실제 팔과 맞는지는 S02에서 자로 재서 확인한다.
placo가 있는 conda lerobot 환경에서만 돈다:  cd project/src && env -u PYTHONPATH python -m pytest tests_exec -q
"""
import numpy as np
import pytest

pytest.importorskip("placo")
from kitbot_exec.kinematics import SO101Kinematics, _wrap90  # noqa: E402


@pytest.fixture(scope="module")
def kin():
    return SO101Kinematics()


def test_fk_zero_pose_is_in_front(kin):
    p, R = kin.fk([0, 0, 0, 0, 0])
    assert p[0] > 300 and abs(p[1]) < 1 and p[2] > 100   # 모든 관절 0° = 앞으로 뻗은 자세 (모델 값 391, 0, 226mm)


# 작업 영역 후보 (모델 기준, 블록 윗면 25mm + 접근 40mm = 65mm 높이).
# 모델에서 수직 집기가 되는 범위(받침대에서 약 135~255mm) 안쪽으로 고른다.
TARGETS = [(x, y, 65.0) for x in (150, 200, 230) for y in (-80, 0, 80)]


@pytest.mark.parametrize("target", TARGETS)
@pytest.mark.parametrize("yaw", [0.0, 30.0, -40.0])
def test_ik_top_down_roundtrip(kin, target, yaw):
    r = kin.ik_top_down(target, yaw)
    assert r.ok, r.reason
    p, R = kin.fk(r.q_deg)
    assert np.linalg.norm(p - target) < 1.0            # 위치 1mm 이내
    assert np.rad2deg(np.arccos(-R[2, 2])) < 2.0        # 수직 아래에서 2° 이내
    tool_yaw = np.rad2deg(np.arctan2(R[1, 0], R[0, 0]))
    assert abs(_wrap90(tool_yaw - yaw)) < 1.0          # 그리퍼가 블록 각도에 맞음 (90° 주기)


def test_rejects_below_table(kin):
    r = kin.ik_top_down((200, 0, 2.0), 0.0, z_min_mm=5.0)
    assert not r.ok and "최소" in r.reason


def test_rejects_unreachable(kin):
    r = kin.ik_top_down((600, 0, 65.0), 0.0)
    assert not r.ok


def test_relaxed_tilt_when_vertical_is_impossible(kin):
    """높이 90mm에서는 손목 굽힘 한계(±95°) 때문에 수직 자세가 안 된다 → 기울기를 허용하면 풀린다."""
    strict = kin.ik_top_down((200, 0, 90.0), 0.0)
    assert not strict.ok
    relaxed = kin.ik_top_down((200, 0, 90.0), 0.0, max_tilt_deg=15.0)
    assert relaxed.ok, relaxed.reason
    p, _ = kin.fk(relaxed.q_deg)
    assert np.linalg.norm(p - (200, 0, 90.0)) < 1.0 and relaxed.tilt_deg <= 15.0
