"""손목 카메라 측정 계산 (장비 없이). 알려진 값을 넣어 되돌아 나오는지 본다."""
import cv2
import numpy as np
import pytest

from kitbot_exec import wrist_measure as wm


def ref(J=((6.0, 0.5), (-0.4, 5.5)), k=-1.0, d=90.0):
    return wm.WristRef((320.0, 240.0), 5.0, 150.0, [list(r) for r in J], k, d)


def test_fit_J_recovers_known_matrix():
    J = np.array([[6.0, 0.5], [-0.4, 5.5]])
    T = np.array([[10, 0], [-10, 0], [0, 10], [0, -10], [7, 7]], float)
    P = T @ J.T + np.random.default_rng(0).normal(0, 0.2, (5, 2))
    Jf, rms = wm.fit_J(T, P)
    assert np.allclose(Jf, J, atol=0.05) and rms < 0.5


def test_offset_tool_sign():
    # 블록이 부리 가운데에서 tool (+3, −4)mm에 있으면, 그리퍼가 (−3, +4) 움직인 것과 같은 픽셀 이동
    r = ref()
    e = np.array([3.0, -4.0])
    px = np.asarray(r.c_ref) - np.asarray(r.J) @ e
    obs = wm.WristObs(tuple(px), r.angle_ref, r.size_ref, 10)
    assert np.allclose(wm.offset_tool_mm(obs, r), e, atol=1e-9)


@pytest.mark.parametrize("k", [-1.0, 1.0, -0.9])
def test_offset_yaw_sign(k):
    r = ref(k=k)
    phi = 7.0                                   # 블록이 그리퍼 대비 +7°
    obs = wm.WristObs(r.c_ref, float(wm.wrap90(r.angle_ref - k * phi)), r.size_ref, 10)
    assert wm.offset_yaw_deg(obs, r) == pytest.approx(phi, abs=1e-6)


def test_yaw_gain_from_rotation():
    assert wm.yaw_gain(5.0, -5.0, 30.0, 40.0) == pytest.approx(-1.0)
    with pytest.raises(ValueError):
        wm.yaw_gain(5.0, 6.0, 30.0, 30.2)


def test_height_from_block_size_pinhole():
    f = 600.0
    d1, dz = 80.0, 20.0
    s1, s2 = 25 * f / d1, 25 * f / (d1 + dz)
    assert wm.d_from_two_heights(s1, s2, dz) == pytest.approx(d1)
    r = wm.WristRef((0, 0), 0, s1, [[1, 0], [0, 1]], -1, d1)
    obs = wm.WristObs((0, 0), 0, 25 * f / (d1 + 5.0), 10)   # 5mm 더 높이 떠 있음
    assert wm.height_offset_mm(obs, r) == pytest.approx(5.0)
    with pytest.raises(ValueError):
        wm.d_from_two_heights(100, 101, 20)


def test_tool_robot_roundtrip():
    a = np.deg2rad(30)
    R = np.array([[np.cos(a), np.sin(a), 0], [np.sin(a), -np.cos(a), 0], [0, 0, -1]])  # 아래를 보는 그리퍼
    v = np.array([4.0, -2.0])
    assert np.allclose(wm.robot_to_tool(R, wm.tool_to_robot(R, v)), v)


def test_mean_square_angle_wraps():
    assert wm.mean_square_angle([44.0, -44.0]) == pytest.approx(-45.0, abs=1.1) or \
        wm.mean_square_angle([44.0, -44.0]) == pytest.approx(45.0, abs=1.1)
    assert wm.mean_square_angle([10, 12, 11]) == pytest.approx(11.0, abs=0.1)


def _purple_square(img, c, side, ang):
    box = cv2.boxPoints(((float(c[0]), float(c[1])), (side, side), ang)).astype(np.int32)
    cv2.fillPoly(img, [box], (160, 60, 120))   # BGR, 보라 범위(H 122~155) 안


def test_observe_picks_center_block_and_measures():
    frames = []
    rng = np.random.default_rng(1)
    for _ in range(5):
        img = np.full((480, 640, 3), 200, np.uint8)
        _purple_square(img, (330, 250), 140, 12)      # 목표: 가운데 근처
        _purple_square(img, (80, 60), 90, 0)          # 옆 블록: 무시돼야 함
        img = np.clip(img + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
        frames.append(img)
    obs = wm.observe(frames, "purple")
    assert obs is not None and obs.n == 5
    assert abs(obs.center_px[0] - 330) < 2 and abs(obs.center_px[1] - 250) < 2
    assert abs(obs.size_px - 140) < 4
    assert abs(abs(obs.angle_deg) - 12) < 2


def test_observe_none_when_missing():
    blank = [np.full((480, 640, 3), 200, np.uint8)] * 5
    assert wm.observe(blank, "purple") is None


def _trial(name, err, yaw=1.0, dz=0.5, tgt=(200.0, 0.0), dq=(0.5, 1.0, 0.2, 0.1, 0.0)):
    dq = list(dq)
    return {"name": name, "result": "approached", "target_xy": list(tgt),
            "diag": {"target_minus_read_deg": dq, "settle_timeout": max(abs(v) for v in dq) >= 2.0},
            "wrist": {"err_robot_mm": list(err), "err_yaw_deg": yaw, "height_offset_mm": dz}}


def test_summarize_counts_failures_in_denominator_and_repeat_spread():
    trials = [_trial("mid_r1", (2, 1)), _trial("mid_r2", (3, 1)), _trial("mid_r3", (2.5, 2)),
              _trial("far_r1", (12, 0), tgt=(260, 0), dq=(0, 2.5, 0, 0, 0)),
              {"name": "x1", "result": "plan_failed"}]
    s = wm.summarize(trials)
    assert s["n_trials"] == 5 and s["pass"] == "3/5"
    assert s["settle_timeout"] == "1/4"
    assert s["repeat"]["mid"]["n"] == 3 and s["repeat"]["mid"]["spread_mm"] < 1.0
    assert s["by_result"] == {"approached": 4, "plan_failed": 1}


def test_summarize_radial_fit_detects_distance_growth():
    trials = [_trial(f"g{i}", (0.06 * r - 6.0, 0.0), tgt=(r, 0.0)) for i, r in enumerate([150, 180, 210, 240])]
    s = wm.summarize(trials)
    assert s["radial_fit"]["b_mm_per_100mm"] == pytest.approx(6.0, abs=0.01)
    assert s["radial_fit"]["a_mm"] == pytest.approx(-6.0, abs=0.1)


def test_group_of():
    assert wm.group_of("middle_0_r2") == "middle_0"
    assert wm.group_of("grid01_30") == "grid01_30"


def test_table_summary_constant_vs_distance():
    flat = [{"tip_mm": [r, 0, -22.0], "calib_table_z": -22.0} for r in (140, 180, 220, 260)]
    s = wm.table_summary(flat)
    assert s["z_range"] == 0 and s["z_vs_distance_mm_per_100mm"] == pytest.approx(0, abs=1e-6)
    tilted = [{"tip_mm": [r, 0, -22.0 - 0.05 * (r - 140)]} for r in (140, 180, 220, 260)]
    assert wm.table_summary(tilted)["z_vs_distance_mm_per_100mm"] == pytest.approx(-5.0, abs=0.01)


def test_wristref_save_load(tmp_path):
    r = ref()
    p = tmp_path / "w.json"
    r.save(p)
    q = wm.WristRef.load(p)
    assert q.c_ref == r.c_ref and q.J == r.J and q.k_yaw == r.k_yaw
