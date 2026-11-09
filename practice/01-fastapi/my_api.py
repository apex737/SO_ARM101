"""실습 01용 서버. 제품 코드(project/src/kitbot)는 고치지 않고, 그 앱에 주소를 하나 덧붙인다.

실행 (저장소 맨 위 폴더에서): .venv\\Scripts\\python practice\\01-fastapi\\my_api.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "project" / "src"))  # kitbot 패키지 위치

from fastapi import Request  # noqa: E402

from kitbot.api import create_app  # noqa: E402
from kitbot.config import Settings  # noqa: E402


def build_app(settings: Settings | None = None):
    settings = settings or Settings.from_env(db_path=str(HERE / "var" / "practice.db"))
    app = create_app(settings)

    # ===== 과제 3: 여기부터 작성 =====
    # 참고: 서버가 쓰는 설정값은 이 함수의 settings 변수에도 있고,
    #       서버가 켜진 뒤에는 request.app.state.settings 에도 들어 있다.
    #       허용 색 목록은 settings.colors (튜플) 이다.
    @app.get("/colors", summary="주문에 쓸 수 있는 색 목록")
    def colors(request: Request):
        # TODO: {"colors": [...]} 형태로 돌려주기. 색 이름을 직접 쓰지 말 것.
        return None
    # ===== 여기까지 =====

    return app


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(build_app(), host="127.0.0.1", port=8000)
