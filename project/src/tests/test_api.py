"""S01 기준 1~4: 접수·거절·멱등성·대기열 한도."""
import json
import threading

import pytest

from kitbot.robot import MockDriver
from tests.conftest import order, wait_status


def test_accept_and_audit(make_client):  # 기준 1
    c = make_client()
    r = c.post("/orders", json=order())
    assert r.status_code == 202
    body = r.json()
    assert body["order_id"].startswith("ord_") and body["status"] in ("QUEUED", "RUNNING", "DONE")
    assert [t["color"] for t in body["tasks"]] == ["red", "red", "blue"]
    log = c.get("/requests", params={"request_id": "r1"}).json()
    assert len(log) == 1 and log[0]["status_code"] == 202
    assert json.loads(log[0]["body"])["request_id"] == "r1"
    assert json.loads(log[0]["response"])["order_id"] == body["order_id"]


@pytest.mark.parametrize("bad", [
    {"request_id": "x", "items": [{"color": "purple", "qty": 1}]},
    {"request_id": "x", "items": [{"color": "red", "qty": 0}]},
    {"request_id": "x", "items": [{"color": "red", "qty": 6}]},
    {"request_id": "x", "items": [{"color": "red", "qty": 5}, {"color": "blue", "qty": 5}, {"color": "green", "qty": 1}]},
    {"request_id": "x", "items": []},
    {"request_id": "x", "items": [{"color": "red", "qty": 1}], "speed": "max"},
    {"request_id": "bad id!", "items": [{"color": "red", "qty": 1}]},
    {"items": [{"color": "red", "qty": 1}]},
])
def test_reject_invalid(make_client, bad):  # 기준 2
    c = make_client()
    r = c.post("/orders", json=bad)
    assert r.status_code == 422, r.text
    assert c.get("/orders").json() == []
    log = c.get("/requests").json()
    assert len(log) == 1 and log[0]["status_code"] == 422 and log[0]["order_id"] is None


def test_reject_broken_json(make_client):  # 기준 2
    c = make_client()
    r = c.post("/orders", content=b'{"request_id": "x", "items": [', headers={"Content-Type": "application/json"})
    assert r.status_code == 422
    assert c.get("/orders").json() == []
    assert c.get("/requests").json()[0]["status_code"] == 422


def test_idempotent_replay_and_conflict(make_client):  # 기준 3
    c = make_client()
    a = c.post("/orders", json=order("dup"))
    b = c.post("/orders", json=order("dup"))
    assert a.status_code == 202 and b.status_code == 200
    assert b.headers.get("Idempotent-Replay") == "true"
    assert a.json()["order_id"] == b.json()["order_id"]
    assert len(c.get("/orders").json()) == 1
    x = c.post("/orders", json=order("dup", items=[{"color": "green", "qty": 1}]))
    assert x.status_code == 409 and x.json()["order_id"] == a.json()["order_id"]
    assert [r["status_code"] for r in c.get("/requests").json()] == [409, 200, 202]


def test_idempotent_concurrent(make_client):  # 기준 3: 동시에 같은 요청 두 번
    c = make_client()
    codes = []

    def send():
        codes.append(c.post("/orders", json=order("same")).status_code)

    ts = [threading.Thread(target=send) for _ in range(5)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    assert sorted(codes) == [200, 200, 200, 200, 202]
    assert len(c.get("/orders").json()) == 1


def test_queue_full(make_client):  # 기준 4
    drv = MockDriver(outcomes=["hang"])  # 첫 주문에서 로봇을 붙잡아 두어 나머지가 QUEUED로 남게 함
    c = make_client(drv, max_queued_orders=2, attempt_timeout_s=30)
    assert c.post("/orders", json=order("q0")).status_code == 202
    # q0가 RUNNING이 된 뒤 대기 2개를 채운다
    from tests.conftest import wait_until
    wait_until(lambda: c.get("/health").json()["orders"].get("RUNNING") == 1)
    assert c.post("/orders", json=order("q1")).status_code == 202
    assert c.post("/orders", json=order("q2")).status_code == 202
    r = c.post("/orders", json=order("q3"))
    assert r.status_code == 503 and r.headers["Retry-After"] == "10"


def test_unknown_order_404(make_client):
    c = make_client()
    assert c.get("/orders/ord_nope").status_code == 404
    assert c.post("/orders/ord_nope/cancel").status_code == 404
