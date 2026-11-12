"""비교 실험 공통 부분(compare)과 순수 IK 반복(eval_ik) — 로봇·카메라 없이."""
import json
from unittest.mock import patch

import numpy as np
import pytest

from kitbot_exec import compare as cp


def test_schedule_cycles_grid_twice():
    s = cp.schedule(18)
    assert s[:9] == [f"g0{i}" for i in range(1, 10)] and s[9:] == s[:9]


@pytest.mark.parametrize("ans,code,note", [("s", "s", ""), ("S 튕김", "s", "튕김"), ("b 테두리", "b", "테두리"),
                                           ("?", "?", ""), ("x", "x", "")])
def test_parse_verdict(ans, code, note):
    v = cp.parse_verdict(ans)
    assert v["verdict"] == code and v["note"] == note and v["success"] == (code == "s")


def test_ask_verdict_repeats_until_valid_and_shows_rules(capsys):
    answers = iter(["", "f", "성공", "h", "d 바구니 앞"])
    assert cp.ask_verdict(lambda _: next(answers))["verdict"] == "d"
    assert "테두리에 걸쳐 일부만 안" in capsys.readouterr().out


def test_summary_and_report(tmp_path):
    rows = [{"method": "ik", "grid": "g01", "verdict": "s", "home_ok": True, "duration_s": 20},
            {"method": "ik", "grid": "g02", "verdict": "g", "home_ok": True, "duration_s": 18},
            {"method": "ik", "grid": "g01", "verdict": "?", "home_ok": False, "duration_s": 22}]
    s = cp.summarize(rows)
    assert s["success"] == 1 and s["rate"] == 0.5 and s["codes"]["?"] == 1 and s["by_group"] == {"g01": "s?", "g02": "g"}
    p = tmp_path / "ik_v1.jsonl"
    p.write_text("".join(json.dumps(r) + "\n" for r in rows))
    out = cp.report([str(p)])
    assert "ik_v1" in out and "50%" in out and "g01" in out


def test_precheck_asks_to_move_then_accepts():
    from kitbot_exec.view_blocks import S02_GRID
    gx, gy = S02_GRID["g05"]
    poses = iter([((gx + 30, gy, 4, 0), None, ""), ((gx + 5, gy - 3, 4, 0), None, "")])
    asks = []
    with patch("kitbot_exec.pick.detect_block_pose", side_effect=lambda *a, **k: next(poses)):
        out = cp.precheck_place(None, None, "purple", None, "g05", 15, ask=lambda q: asks.append(q) or "")
    assert out["precheck"] == "ok" and out["grid_offset_mm"] == pytest.approx(np.hypot(5, 3), abs=0.1)
    assert "x -30" in asks[0]


def test_precheck_force_and_not_seen():
    with patch("kitbot_exec.pick.detect_block_pose", return_value=((0, 0, 4, 0), None, "")):
        assert cp.precheck_place(None, None, "purple", None, "g05", 15, ask=lambda q: "f")["precheck"] == "forced"
    with patch("kitbot_exec.pick.detect_block_pose", return_value=(None, None, "안 보임")):
        assert cp.precheck_place(None, None, "purple", None, "g05", 15, ask=lambda q: "")["precheck"] == "not_seen"


def test_place_record_distance_to_nearest_demo():
    calib = type("C", (), {"pairs_mm": None})()
    demos = np.array([[100.0, 100.0], [150.0, 150.0]])
    r = cp.place_record(calib, 103.0, 104.0, demos)
    assert r["nearest_demo_mm"] == 5.0 and r["near"] == "시연<20" and r["block_xy"] == [103.0, 104.0]
    assert cp.place_record(calib, 100.0, 130.0, demos)["near"] == "시연<40"
    assert cp.place_record(calib, 200.0, 50.0, demos)["near"] == "시연≥40"


def test_demo_positions_skip_validation_episodes(tmp_path):
    p = tmp_path / "meta.jsonl"
    p.write_text("".join(json.dumps({"episode_index": i, "block_camera": [i, i, 0, 0]}) + "\n" for i in range(8)))
    pos = cp.demo_positions(p, exclude=(6,))
    assert len(pos) == 7 and 6.0 not in pos[:, 0]


class _Cap:
    def read(self):
        return True, np.zeros((4, 4, 3), np.uint8)

    def release(self):
        pass


def _run_eval_ik(tmp_path, monkeypatch, extra, answers):
    from kitbot_exec import eval_ik

    monkeypatch.setattr(cp, "OUT_DIR", tmp_path)
    monkeypatch.setattr(cp, "demo_positions", lambda *a, **k: np.array([[100.0, 100.0]]))
    calls, pre = [], []
    results = iter([{"result": "placed", "held": True, "block_camera": [96, 101, 0]},
                    {"result": "empty", "held": False, "block_camera": [160, 99, 0]}])
    it = iter(answers)
    monkeypatch.setattr("builtins.input", lambda *_: next(it))
    with patch("kitbot_exec.pick.main", side_effect=lambda argv: calls.append(argv) or next(results)), \
         patch("kitbot_exec.camera.open_role", return_value=(_Cap(), None)), \
         patch("kitbot_exec.camera.correct_wb", side_effect=lambda f, g: f), \
         patch("kitbot_exec.calib.PixelToRobot.load", return_value=type("C", (), {"pairs_mm": None})()), \
         patch.object(cp, "precheck_place", side_effect=lambda cap, g, c, cal, grid, tol: pre.append(grid) or {"grid": grid}):
        eval_ik.main(["--name", "ik_t", "--trials", "2"] + extra)
    return calls, pre, cp.read(tmp_path / "ik_t.jsonl")


def test_eval_ik_default_is_free_placement_with_recorded_position(tmp_path, monkeypatch):
    calls, pre, rows = _run_eval_ik(tmp_path, monkeypatch, [], ["", "s", "", "g"])
    assert pre == [] and all("--no-correction" in a for a in calls) and len(calls) == 2
    assert [r["verdict"] for r in rows] == ["s", "g"] and "grid" not in rows[0]
    assert rows[0]["nearest_demo_mm"] == pytest.approx(4.1, abs=0.1) and rows[1]["near"] == "시연≥40"
    assert rows[0]["method"] == "ik" and rows[0]["home_ok"] and (tmp_path / "ik_t" / "t01_top.jpg").exists()


def test_eval_ik_grid_mode_guides_grid_points(tmp_path, monkeypatch):
    _, pre, rows = _run_eval_ik(tmp_path, monkeypatch, ["--grid"], ["", "s", "", "s"])
    assert pre == ["g01", "g02"] and [r["grid"] for r in rows] == ["g01", "g02"]


def test_eval_ik_color_can_be_changed_at_prompt(tmp_path, monkeypatch):
    calls, _, rows = _run_eval_ik(tmp_path, monkeypatch, [], ["blue", "s", "pink", "red", "g"])
    assert [a[a.index("--color") + 1] for a in calls] == ["blue", "red"]
    assert [r["color"] for r in rows] == ["blue", "red"]


# ---------- 큐 ----------
COLORS = ["blue", "green", "orange", "purple", "red", "yellow"]


def test_parse_queue():
    assert cp.parse_queue("orange 2, yellow 1, blue", COLORS) == [("orange", 2), ("yellow", 1), ("blue", None)]
    assert cp.parse_queue("Red all; purple 3", COLORS) == [("red", None), ("purple", 3)]
    for bad in ("pink 1", "orange 0", "orange two", " , "):
        with pytest.raises(SystemExit):
            cp.parse_queue(bad, COLORS)


def test_queue_moves_on_when_done_missing_or_failing():
    q = cp.PickQueue([("orange", 2), ("yellow", 1), ("blue", None), ("red", None)])
    assert q.current() == "orange"; q.report("ok"); assert q.current() == "orange"
    q.report("none"); assert q.current() == "yellow"          # 주황은 1개뿐 → 다음 명령
    q.report("fail"); q.report("ok"); assert q.current() == "blue"   # 실패 1번은 다시 시도, 성공하면 목표 채움
    q.report("ok"); q.report("ok"); q.report("none"); assert q.current() == "red"   # 안 보일 때까지
    q.report("fail"); q.report("fail"); assert q.current() is None   # 연속 2번 실패 → 넘어감
    assert [(c, ok) for c, _, ok, _ in q.log] == [("orange", 1), ("yellow", 1), ("blue", 2), ("red", 0)]


def test_eval_ik_queue_runs_without_prompts(tmp_path, monkeypatch):
    from kitbot_exec import eval_ik

    monkeypatch.setattr(cp, "OUT_DIR", tmp_path)
    monkeypatch.setattr(cp, "demo_positions", lambda *a, **k: np.array([[100.0, 100.0]]))
    monkeypatch.setattr("builtins.input", lambda *_: pytest.fail("큐 실행은 묻지 않아야 함"))
    table = {"orange": 3, "yellow": 0, "blue": 1}                 # 테이블에 있는 블록 수
    calls = []

    def fake_pick(argv):
        c = argv[argv.index("--color") + 1]; calls.append(c)
        if table[c] == 0:
            return {"result": "no_block"}
        table[c] -= 1
        return {"result": "placed", "held": True, "block_camera": [120, 100, 0]}

    with patch("kitbot_exec.pick.main", side_effect=fake_pick), \
         patch("kitbot_exec.camera.open_role", return_value=(_Cap(), None)), \
         patch("kitbot_exec.camera.correct_wb", side_effect=lambda f, g: f), \
         patch("kitbot_exec.calib.PixelToRobot.load", return_value=type("C", (), {"pairs_mm": None})()):
        eval_ik.main(["--name", "q", "--queue", "orange 2, yellow 1, blue"])
    assert calls == ["orange", "orange", "yellow", "blue", "blue"]     # 주황 2개만, 노랑 없음 → 넘어감, 파랑은 없어질 때까지
    rows = cp.read(tmp_path / "q.jsonl")
    assert [r["color"] for r in rows] == ["orange", "orange", "blue"] and table["orange"] == 1
    assert all(r["verdict"] == "?" and r["auto"] == "placed" for r in rows)
