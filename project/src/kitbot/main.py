"""Pi에서 실행: python -m kitbot.main --host 0.0.0.0 --port 8000"""
import argparse
import logging

import uvicorn

from .api import create_app
from .config import Settings


def main():
    p = argparse.ArgumentParser(description="KitBot gateway server")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    a = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    # 작업자 스레드가 하나뿐이어야 하므로 uvicorn worker 프로세스는 1개로 고정한다.
    uvicorn.run(create_app(Settings.from_env()), host=a.host, port=a.port, workers=1)


if __name__ == "__main__":
    main()
