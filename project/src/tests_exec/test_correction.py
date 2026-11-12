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


def test_pickable_area_is_table_side_of_basket_front_not_calibration_hull():
    # 2026-11-12 사용자 결정: 보정점 범위 밖도 접근. 바구니 앞 테두리 선 너머(바구니 안·키트 박스)만 뺀다.
    from types import SimpleNamespace
    pytest.importorskip("cv2")
    from kitbot_exec.pick import beyond_basket_front, in_work_area
    cal = SimpleNamespace(pairs_mm=np.array([[87, 38, 0], [150, 38, 0], [150, 239, 0], [87, 239, 0]], float))
    assert in_work_area(cal, 120, 140)
    assert in_work_area(cal, 175, 142)            # 실물에서 거부됐던 보라 (보정점 범위 + 33mm, 바구니 앞 테이블)
    assert in_work_area(cal, 182, 93)             # 같은 장면 파랑 (바구니 앞)
    assert in_work_area(cal, 40, 300)             # 보정점 범위에서 멀어도 테이블 쪽이면 센다 (닿는지는 plan_pick)
    assert not in_work_area(cal, 265, 100)        # 바구니 안 노랑
    assert beyond_basket_front(175, 142) == pytest.approx(-30, abs=1)
    assert in_work_area(SimpleNamespace(pairs_mm=None), 0, 0)


def test_why_not_found_explains_basket_side_and_duplicates():
    # 실물(2026-11-12): 보라 1개를 놓았는데 "0개이거나 2개 이상"으로만 나와 원인을 몰랐음
    from types import SimpleNamespace

    from kitbot_exec import pick
    calib = SimpleNamespace(pairs_mm=np.array([[0, 0, 0], [100, 0, 0], [100, 100, 0], [0, 100, 0]], float))
    assert pick.outside_mm(calib, 133, 50) == pytest.approx(33.0)        # 보정점 범위 밖 거리 (기록·경고용)
    msg = pick.why_not_found(calib, "purple", [0] * 30, [(260, 100, 0, 0)] * 20)
    assert "바구니 쪽에서만" in msg and "x=260" in msg
    assert "2개 보임" in pick.why_not_found(calib, "purple", [2] * 30, [])
    assert "안 보임" in pick.why_not_found(calib, "purple", [0] * 30, [])


def test_detect_block_pose_accepts_outside_calibration_with_warning(monkeypatch):
    from types import SimpleNamespace

    from kitbot_exec import pick
    calib = SimpleNamespace(pairs_mm=np.array([[87, 38, 0], [150, 38, 0], [150, 239, 0], [87, 239, 0]], float),
                            block_pose=lambda b: b)
    cap = SimpleNamespace(read=lambda: (True, np.zeros((4, 4, 3), np.uint8)))
    monkeypatch.setattr("kitbot_exec.camera.correct_wb", lambda f, g: f)
    # 바구니 앞 보라 (175,142) + 바구니 안 보라 (265,100): 테이블 쪽 하나만 세어 접근
    monkeypatch.setattr("kitbot_exec.vision.detect_blocks", lambda f, c: [(175.0, 142.0, 4.0, 5.0), (265.0, 100.0, 4.0, 0.0)])
    pose, _, note = pick.detect_block_pose(cap, None, "purple", calib)
    assert pose[:2] == pytest.approx((175, 142)) and "보정점 범위 밖 25mm" in note


def test_nearest_mode_picks_block_closest_to_robot_and_tracks_it(monkeypatch):
    from types import SimpleNamespace

    from kitbot_exec import pick
    calib = SimpleNamespace(pairs_mm=None, block_pose=lambda b: b)
    cap = SimpleNamespace(read=lambda: (True, np.zeros((4, 4, 3), np.uint8)))
    monkeypatch.setattr("kitbot_exec.camera.correct_wb", lambda f, g: f)
    far, near, basket = (150.0, 150.0, 4.0, 0.0), (100.0, 60.0, 4.0, 10.0), (265.0, 100.0, 4.0, 0.0)
    order = iter([[far, near, basket], [near, far]] * 40)        # 장마다 검출 순서가 달라도 같은 블록
    monkeypatch.setattr("kitbot_exec.vision.detect_blocks", lambda f, c: next(order))
    pose, _, note = pick.detect_block_pose(cap, None, "purple", calib, nearest=True)
    assert pose[:2] == pytest.approx((100, 60)) and pose[3] == pytest.approx(10) and "2개 중" in note
    # 기본(ACT 쪽)은 그대로: 여러 개면 거부
    order = iter([[far, near]] * 40)
    pose, _, note = pick.detect_block_pose(cap, None, "purple", calib)
    assert pose is None and "2개 보임" in note
