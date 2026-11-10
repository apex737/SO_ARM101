"""화면 → 로봇 좌표 보정 시험.

비스듬히 테이블을 내려다보는 가상 카메라(핀홀 모델)로 3D 점을 픽셀로 찍어 보고,
보정점 9쌍으로 구한 변환이 보정에 쓰지 않은 점을 얼마나 정확히 되돌리는지 확인한다.
가상 카메라 값(높이 600mm, 30° 기울기, 초점거리 600px)은 설명용 가정이다.
"""
import numpy as np
import pytest

from kitbot_exec.calib import PixelToRobot, fit_pixel_to_robot, yaw_from_corners
from kitbot_exec.vision import Block

BLOCK = 25.0  # 블록 한 변 [mm] (가정)


def make_camera(height=600.0, tilt_deg=30.0, cam_xy=(-150.0, 0.0), f=600.0, cx=320.0, cy=240.0):
    """로봇 뒤쪽 장대 위에서 앞(+x)을 비스듬히 내려다보는 카메라. 로봇 좌표 [mm] → 픽셀."""
    t = np.deg2rad(tilt_deg)
    fwd = np.array([np.sin(t), 0.0, -np.cos(t)])        # 카메라가 보는 방향 (수직 아래에서 tilt만큼 앞으로)
    right = np.array([0.0, -1.0, 0.0])                    # 화면 오른쪽 = 로봇 -y
    down = np.cross(fwd, right)                           # 화면 아래쪽
    R = np.stack([right, down, fwd])                      # 로봇 → 카메라 회전
    C = np.array([cam_xy[0], cam_xy[1], height])

    def project(P):
        P = np.atleast_2d(np.asarray(P, float))
        pc = (P - C) @ R.T
        return np.c_[f * pc[:, 0] / pc[:, 2] + cx, f * pc[:, 1] / pc[:, 2] + cy]

    return project, C


def grid(xs, ys, z):
    return np.array([[x, y, z] for x in xs for y in ys], float)


def test_recovers_untouched_points_on_block_top_plane():
    project, _ = make_camera()
    calib_pts = grid([120, 200, 280], [-120, 0, 120], BLOCK)       # 보정점 9곳 (블록 윗면)
    test_pts = grid([150, 240], [-80, 60], BLOCK)                 # 보정에 쓰지 않은 4곳
    rng = np.random.default_rng(1)
    px = project(calib_pts) + rng.normal(0, 0.3, (9, 2))         # 검출 흔들림 0.3px
    cal = fit_pixel_to_robot(px, calib_pts)
    err = cal.residuals_mm(project(test_pts), test_pts)
    assert err.max() < 1.5, err
    assert np.allclose(cal.z_at(test_pts[:, :2]), BLOCK, atol=0.1)


def test_parallax_if_calibrated_on_table_floor():
    """테이블 바닥(z=0)에서 보정하고 블록 윗면(z=25)을 재면 위치가 밀린다 → 블록 윗면에서 보정해야 하는 이유."""
    project, C = make_camera()
    floor = grid([120, 200, 280], [-120, 0, 120], 0.0)
    cal_floor = fit_pixel_to_robot(project(floor), floor)
    top = np.array([[280.0, 120.0, BLOCK]])                       # 카메라에서 먼 구석의 블록 윗면
    err = float(cal_floor.residuals_mm(project(top), top)[0])
    # 닮은 삼각형으로 계산한 밀림: (카메라 바로 아래 점에서의 수평 거리) × 블록 높이 ÷ (카메라 높이 − 블록 높이)
    d = np.linalg.norm(top[0, :2] - C[:2])
    expected = d * BLOCK / (C[2] - BLOCK)
    assert err == pytest.approx(expected, rel=0.05)
    assert err > 10  # 이 가정에서는 1cm 넘게 밀린다


def test_block_yaw_through_tilted_camera():
    """비스듬한 카메라에서는 화면 각도가 왜곡된다. 꼭짓점을 로봇 평면으로 옮겨 구한 각도는 실제와 맞아야 한다."""
    project, _ = make_camera()
    pts = grid([120, 200, 280], [-120, 0, 120], BLOCK)
    cal = fit_pixel_to_robot(project(pts), pts)
    for yaw in (0.0, 20.0, -35.0):
        a = np.deg2rad(yaw)
        half = BLOCK / 2
        local = np.array([[-half, -half], [half, -half], [half, half], [-half, half]])
        rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        corners = local @ rot.T + [220.0, 60.0]
        corners3 = np.c_[corners, np.full(4, BLOCK)]
        corners_px = project(corners3)
        b = Block(tuple(corners_px.mean(0)), 0.0, (0, 0), 0, 1, corners_px, None)
        x, y, z, got = cal.block_pose(b)
        assert abs(x - 220) < 1.0 and abs(y - 60) < 1.0
        assert abs(got - yaw) < 0.5


def test_yaw_from_corners_period_90():
    sq = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    assert yaw_from_corners(sq) == pytest.approx(0.0, abs=1e-6)
    a = np.deg2rad(60)  # 60° = -30° (90° 주기)
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    assert yaw_from_corners(sq @ rot.T) == pytest.approx(-30.0, abs=1e-6)


def test_save_load_roundtrip(tmp_path):
    pts = grid([100, 200], [-50, 50], BLOCK)
    px = pts[:, :2] * 2 + 10
    cal = fit_pixel_to_robot(px, pts)
    p = tmp_path / "calib.json"
    cal.save(p)
    cal2 = PixelToRobot.load(p)
    assert np.allclose(cal2.to_xy([[300, 110]]), cal.to_xy([[300, 110]]))


def test_needs_four_points():
    with pytest.raises(ValueError):
        fit_pixel_to_robot([[0, 0], [1, 0], [0, 1]], [[0, 0, 0]] * 3)
