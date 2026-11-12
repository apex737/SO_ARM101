"""집기 계획 시험 (모델 안 계산, 팔 불필요). conda lerobot 환경에서만 돈다."""
import numpy as np
import pytest

pytest.importorskip("placo")
from kitbot_exec.kinematics import SO101Kinematics  # noqa: E402
from kitbot_exec.pick import PickParams, jaw_center_xy, plan_pick  # noqa: E402


@pytest.fixture(scope="module")
def kin():
    return SO101Kinematics()


@pytest.mark.parametrize("x,y,yaw", [(200, 0, 0), (180, 60, 25), (220, -60, -40)])
def test_plan_reaches_and_goes_straight_down(kin, x, y, yaw):
    plan = plan_pick(kin, x, y, 25.0, yaw, np.zeros(5))
    assert plan.ok, plan.reason
    zs = []
    for q in plan.down[:: max(len(plan.down) // 10, 1)] + [plan.down[-1]]:
        p, R = kin.fk(q)
        c = jaw_center_xy(kin, q, PickParams().jaw_center_mm)
        assert np.hypot(c[0] - x, c[1] - y) < 1.5            # 두 부리 가운데가 내려가는 동안 수평으로 1.5mm 안
        zs.append(p[2])
    assert zs[-1] == pytest.approx(25.0 - 12.0, abs=1.0)     # 블록 윗면 − 12mm 까지
    assert all(a >= b - 0.5 for a, b in zip(zs, zs[1:]))      # 계속 내려감
    p, _ = kin.fk(plan.up[-1])
    assert p[2] == pytest.approx(25.0 + PickParams().lift_mm, abs=1.0)


def test_rejects_grasp_below_table(kin):
    plan = plan_pick(kin, 200, 0, 25.0, 0, np.zeros(5), PickParams(grasp_depth_mm=24.0))
    assert not plan.ok and "테이블" in plan.reason


def test_rejects_unreachable_block(kin):
    plan = plan_pick(kin, 450, 0, 25.0, 0, np.zeros(5))
    assert not plan.ok


def test_jaw_center_not_tip_is_on_block(kin):
    """벌린 두 부리 가운데가 블록 위에 오고, 모델 그리퍼 끝은 그만큼(22mm) 옆에 있다."""
    p = PickParams()
    for yaw in (-40, 0, 30):
        plan = plan_pick(kin, 200, 30, 25.0, yaw, np.zeros(5), p)
        assert plan.ok, plan.reason
        c = jaw_center_xy(kin, plan.q_pre, p.jaw_center_mm)
        tip, _ = kin.fk(plan.q_pre)
        assert np.hypot(c[0] - 200, c[1] - 30) < 1.0
        assert np.hypot(tip[0] - 200, tip[1] - 30) == pytest.approx(abs(p.jaw_center_mm[0]), abs=1.0)


def test_near_base_block_needs_lower_approach_and_other_wrist_candidate():
    # 실물(2026-11-12): 주황 블록 (100, 44), 각도 37° → "닿지 않음 15.5mm"로 거부됨
    import numpy as np

    from kitbot_exec import pick
    from kitbot_exec.arm import load_poses
    from kitbot_exec.kinematics import SO101Kinematics
    kin = SO101Kinematics(); q_home = np.array(load_poses()["home"]["q_deg"])
    assert not pick.plan_pick(kin, 100.3, 44.5, 8.8, 37.0, q_home).ok
    plan = pick.plan_pick_adaptive(kin, 100.3, 44.5, 8.8, 37.0, q_home)
    assert plan.ok and plan.info["approach_mm"] == 30.0 and "low_height" in plan.info
    # 평소 자리는 그대로 기본 높이·기본 손목
    normal = pick.plan_pick_adaptive(kin, 120.0, 160.0, 4.0, 0.0, q_home)
    assert normal.ok and "low_height" not in normal.info and "roll_candidate" not in normal.info
