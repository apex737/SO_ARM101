import time

import pytest
from fastapi.testclient import TestClient

from kitbot.api import create_app
from kitbot.config import Settings
from kitbot.robot import MockDriver


def fast_settings(tmp_path, **kw) -> Settings:
    base = dict(db_path=str(tmp_path / "t.db"), attempt_timeout_s=2.0, driver_grace_s=0.5,
                action_timeout_s=2.0, worker_poll_s=0.02, max_attempts=3)
    base.update(kw)
    return Settings(**base)


@pytest.fixture
def make_client(tmp_path):
    """make_client(driver=MockDriver(...), **settings) → 시작된 TestClient. 끝나면 자동 종료."""
    opened = []

    def _make(driver=None, **kw):
        driver = driver or MockDriver()
        c = TestClient(create_app(fast_settings(tmp_path, **kw), driver))
        c.__enter__()
        c.driver = driver
        opened.append(c)
        return c

    yield _make
    for c in opened:
        c.driver.release.set()
        try:
            c.__exit__(None, None, None)
        except Exception:
            pass  # 시험 안에서 이미 종료한 클라이언트


def wait_status(client, order_id, statuses=("DONE", "FAILED", "CANCELLED"), timeout=5.0):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        o = client.get(f"/orders/{order_id}").json()
        if o["status"] in statuses:
            return o
        time.sleep(0.02)
    raise AssertionError(f"order {order_id} stuck in {o['status']}")


def wait_until(fn, timeout=5.0):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        v = fn()
        if v:
            return v
        time.sleep(0.02)
    raise AssertionError("condition not met")


def order(rid="r1", **kw):
    body = {"request_id": rid, "items": [{"color": "red", "qty": 2}, {"color": "blue", "qty": 1}]}
    body.update(kw)
    return body
