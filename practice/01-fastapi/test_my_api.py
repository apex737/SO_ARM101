"""과제 3 자동 시험. 실행: .venv\\Scripts\\python -m pytest practice\\01-fastapi -q"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / "project" / "src"))

from fastapi.testclient import TestClient  # noqa: E402

from kitbot.config import Settings  # noqa: E402
import my_api  # noqa: E402


def client(tmp_path, colors):
    return TestClient(my_api.build_app(Settings(db_path=str(tmp_path / "t.db"), colors=colors)))


def test_colors_default(tmp_path):
    with client(tmp_path, ("red", "green", "blue")) as c:
        r = c.get("/colors")
        assert r.status_code == 200
        assert r.json() == {"colors": ["red", "green", "blue"]}


def test_colors_follow_settings(tmp_path):
    # 설정을 바꾸면 응답도 바뀌어야 한다 (색 이름을 코드에 직접 쓰면 실패)
    with client(tmp_path, ("red", "yellow")) as c:
        assert c.get("/colors").json() == {"colors": ["red", "yellow"]}


def test_colors_in_docs(tmp_path):
    with client(tmp_path, ("red",)) as c:
        assert "/colors" in c.get("/openapi.json").json()["paths"]
