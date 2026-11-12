"""loss_curve의 로그 읽기·체크포인트 찾기 (학습·GPU 없이)."""
from kitbot_exec import loss_curve as lc


def test_parse_train_log_handles_rounded_steps(tmp_path):
    p = tmp_path / "train.log"
    p.write_text("Training: 1%|\rINFO x ot_train.py:606 step:200 smpl:2K ep:3 epch:0.07 loss:6.791 grdn:1\n"
                 "INFO step:400 smpl:3K loss:2.945 grdn:8\n"
                 "INFO step:1K smpl:8K loss:1.920 grdn:6\n"
                 "INFO step:1K smpl:10K loss:1.743 grdn:6\n"
                 "eval loss:9 (not a step line)\n")
    assert lc.parse_train_log(p) == [(200, 6.791), (400, 2.945), (600, 1.92), (800, 1.743)]


def test_checkpoints_sorted_and_only_complete(tmp_path):
    for s in ("002000", "001000", "last"):
        (tmp_path / "checkpoints" / s / "pretrained_model").mkdir(parents=True)
    (tmp_path / "checkpoints" / "003000").mkdir()          # 저장 중(아직 pretrained_model 없음)
    assert [s for s, _ in lc.checkpoints(tmp_path)] == [1000, 2000]


def test_val_episodes_are_not_trained_on():
    import json
    import re
    from pathlib import Path
    cmd = Path(__file__).resolve().parents[2] / "commands.md"
    text = cmd.read_text(encoding="utf-8")
    m = re.search(r'EPS="\[([\d, ]+)\]"', text)
    assert m, "commands.md에 학습 에피소드 목록이 있어야 함"
    train = {int(v) for v in m.group(1).split(",")}
    assert not train & set(lc.VAL_EPISODES) and len(train | set(lc.VAL_EPISODES)) == 46
