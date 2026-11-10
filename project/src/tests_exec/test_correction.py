"""접근 보정 시험 (팔 불필요)."""
import numpy as np
import pytest

from kitbot_exec.correction import AFFINE_MIN, Correction, add_sample, load_samples, wrap90


def test_wrap90():
    assert wrap90(50) == pytest.approx(-40)
    assert wrap90(-46) == pytest.approx(44)
    assert wrap90(44) == pytest.approx(44)


def test_no_samples_is_identity(tmp_path):
    c = Correction(load_samples(tmp_path / "none.json"))
    assert c.mode == "none"
    assert c.apply(100.0, 50.0, 10.0) == pytest.approx((100.0, 50.0, 10.0))


def test_mean_offset_with_few_samples(tmp_path):
    f = tmp_path / "c.json"
    for x, y in [(150, 50), (200, 100), (180, 150)]:
        add_sample((x, y, 10.0), (x + 4.0, y - 3.0, 12.0), path=f)
    c = Correction(load_samples(f))
    assert c.mode == "mean" and c.n == 3
    assert c.apply(170.0, 80.0, 0.0) == pytest.approx((174.0, 77.0, 2.0))


def test_yaw_offset_wraps_across_45(tmp_path):
    f = tmp_path / "c.json"
    add_sample((150, 50, 43.0), (150, 50, -44.0), path=f)  # 실제로는 +3° 차이
    c = Correction(load_samples(f))
    assert c.offset(150, 50)[2] == pytest.approx(3.0)


def test_affine_learns_position_dependent_offset(tmp_path):
    f = tmp_path / "c.json"
    rng = np.random.default_rng(0)
    true = lambda x, y: (2.0 + 0.05 * (x - 150), -1.0 + 0.03 * (y - 100))
    for _ in range(AFFINE_MIN + 2):
        x, y = rng.uniform(120, 260), rng.uniform(40, 240)
        dx, dy = true(x, y)
        add_sample((x, y, 0.0), (x + dx, y + dy, 0.0), path=f)
    c = Correction(load_samples(f))
    assert c.mode == "affine"
    for x, y in [(130, 60), (240, 220)]:
        assert c.offset(x, y)[:2] == pytest.approx(true(x, y), abs=0.05)


def test_screen_axes_match_calibration():
    cv2 = pytest.importorskip("cv2")
    from kitbot_exec.calib import PixelToRobot
    from kitbot_exec.pick import screen_axes
    # 화면 오른쪽(+u) = 로봇 -y, 화면 위(-v) = 로봇 +x 인 가상 보정
    H = np.array([[0.0, -0.5, 300.0], [-0.5, 0.0, 200.0], [0.0, 0.0, 1.0]])
    cal = PixelToRobot(H, np.array([0.0, 0.0, 25.0]), np.zeros((0, 2)), np.zeros((0, 3)))
    ax = screen_axes(cal, 150.0, 100.0)
    assert ax[:, 0] == pytest.approx([0.0, -1.0], abs=1e-6)   # 오른쪽
    assert ax[:, 1] == pytest.approx([1.0, 0.0], abs=1e-6)    # 위
