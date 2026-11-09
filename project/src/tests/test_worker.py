"""S01 기준 5~11: 실행·재시도·FAULT·재시작 복구·취소."""
import time

from kitbot.robot import MockDriver
from kitbot.store import Store
from tests.conftest import fast_settings, order, wait_status, wait_until


def results(o):
    return [[a["result"] for a in t["attempts"]] for t in o["tasks"]]


def test_all_success(make_client):  # 기준 5
    c = make_client()
    oid = c.post("/orders", json=order()).json()["order_id"]
    o = wait_status(c, oid)
    assert o["status"] == "DONE"
    assert [t["status"] for t in o["tasks"]] == ["SUCCEEDED"] * 3
    assert results(o) == [["SUCCESS"]] * 3


def test_retry_after_grasp_fail(make_client):  # 기준 6
    drv = MockDriver(outcomes=["grasp_fail", "ok"])
    c = make_client(drv)
    oid = c.post("/orders", json=order(items=[{"color": "red", "qty": 1}])).json()["order_id"]
    o = wait_status(c, oid)
    assert o["status"] == "DONE"
    assert results(o) == [["GRASP_FAIL", "SUCCESS"]]
    names = [x[0] for x in drv.calls]
    # 시작 시 recover 1회, 실패 뒤 recover 1회, 그 다음 재시도 pick
    assert names == ["connect", "recover", "pick", "verify", "recover", "pick", "verify", "place"]


def test_retry_kinds(make_client):  # 기준 6: 다른 실패 유형도 재시도 대상
    drv = MockDriver(outcomes=["timeout", "error", "ok"])
    c = make_client(drv)
    oid = c.post("/orders", json=order(items=[{"color": "green", "qty": 1}])).json()["order_id"]
    o = wait_status(c, oid)
    assert o["status"] == "DONE"
    assert results(o) == [["TIMEOUT", "DRIVER_ERROR", "SUCCESS"]]


def test_final_failure_then_next_order(make_client):  # 기준 7
    drv = MockDriver(outcomes=["ok", "grasp_fail", "place_fail", "grasp_fail"])
    c = make_client(drv, max_attempts=3)
    a = c.post("/orders", json=order("a")).json()["order_id"]
    b = c.post("/orders", json=order("b", items=[{"color": "blue", "qty": 1}])).json()["order_id"]
    oa = wait_status(c, a)
    assert oa["status"] == "FAILED" and "task 2" in oa["error"]
    assert [t["status"] for t in oa["tasks"]] == ["SUCCEEDED", "FAILED", "SKIPPED"]
    assert results(oa) == [["SUCCESS"], ["GRASP_FAIL", "PLACE_FAIL", "GRASP_FAIL"], []]
    assert wait_status(c, b)["status"] == "DONE"
    req = c.get("/requests", params={"request_id": "a"}).json()[0]
    assert req["status_code"] == 202 and req["order_status"] == "FAILED"  # 요청→응답→실제 결과 추적


def test_recover_failure_faults_then_reset(make_client):  # 기준 8
    # recover 호출 순서: 시작 시 True, 실패 뒤 False(→FAULT), 리셋 뒤 True
    drv = MockDriver(outcomes=["grasp_fail", "ok"], recover_outcomes=[True, False, True])
    c = make_client(drv)
    oid = c.post("/orders", json=order(items=[{"color": "red", "qty": 1}])).json()["order_id"]
    h = wait_until(lambda: (lambda j: j if j["status"] == "fault" else None)(c.get("/health").json()))
    assert "recover failed" in h["fault"]
    time.sleep(0.2)
    o = c.get(f"/orders/{oid}").json()
    assert o["status"] == "RUNNING" and o["tasks"][0]["status"] == "PENDING"
    assert sum(1 for x in drv.calls if x[0] == "pick") == 1  # FAULT 동안 더 집지 않음
    assert c.post("/robot/reset").json()["previous_fault"]
    o = wait_status(c, oid)
    assert o["status"] == "DONE" and results(o) == [["GRASP_FAIL", "SUCCESS"]]
    assert c.get("/health").json()["status"] == "ok"


def test_driver_hang_faults(make_client):  # 기준 9
    drv = MockDriver(outcomes=["hang", "ok"])
    c = make_client(drv, attempt_timeout_s=0.2, driver_grace_s=0.2)
    oid = c.post("/orders", json=order(items=[{"color": "red", "qty": 1}])).json()["order_id"]
    wait_until(lambda: c.get("/health").json()["status"] == "fault")
    assert "driver hung" in c.get("/health").json()["fault"]
    drv.release.set()
    c.post("/robot/reset")
    o = wait_status(c, oid)
    assert o["status"] == "DONE" and results(o) == [["DRIVER_HUNG", "SUCCESS"]]


def test_fault_persists_across_restart(make_client, tmp_path):  # 기준 8: FAULT는 재시작해도 유지
    c1 = make_client(MockDriver(recover_outcomes=[False]))
    wait_until(lambda: c1.get("/health").json()["status"] == "fault")
    c1.__exit__(None, None, None)
    c1.driver.release.set()
    c2 = make_client(MockDriver())
    assert c2.get("/health").json()["status"] == "fault"


def test_restart_recovery(make_client, tmp_path):  # 기준 10
    # 서버가 집기 도중 꺼진 상태를 DB에 직접 만든다.
    st = Store(fast_settings(tmp_path).db_path)
    oid, _ = st.create_order("crash", [{"color": "red", "qty": 2}], 0, 20)
    assert st.claim_next_order() == oid
    t1, t2 = [t["id"] for t in st.pending_tasks(oid)]
    st.set_task(t1, "RUNNING")
    st.start_attempt(t1)  # 끝나지 않은 시도
    st.close()

    drv = MockDriver()
    c = make_client(drv)
    o = wait_status(c, oid)
    assert o["status"] == "DONE"
    assert results(o) == [["INTERRUPTED", "SUCCESS"], ["SUCCESS"]]
    assert [x[0] for x in drv.calls][:3] == ["connect", "recover", "pick"]  # 복구 먼저
    types = [e["type"] for e in c.get("/events").json()]
    assert "startup.interrupted" in types


def test_cancel_queued(make_client):  # 기준 11
    drv = MockDriver(outcomes=["hang"])
    c = make_client(drv, attempt_timeout_s=30)
    c.post("/orders", json=order("busy"))
    wait_until(lambda: c.get("/health").json()["orders"].get("RUNNING") == 1)
    oid = c.post("/orders", json=order("later")).json()["order_id"]
    r = c.post(f"/orders/{oid}/cancel")
    assert r.status_code == 200 and r.json()["status"] == "CANCELLED"
    o = c.get(f"/orders/{oid}").json()
    assert o["status"] == "CANCELLED" and all(t["status"] == "SKIPPED" for t in o["tasks"])
    assert c.post(f"/orders/{oid}/cancel").status_code == 409  # 끝난 주문


def test_cancel_running(make_client):  # 기준 11
    drv = MockDriver(pick_s=0.3)
    c = make_client(drv)
    oid = c.post("/orders", json=order(items=[{"color": "red", "qty": 3}])).json()["order_id"]
    wait_until(lambda: c.get("/health").json()["current"])
    assert c.post(f"/orders/{oid}/cancel").json()["status"] == "RUNNING"
    o = wait_status(c, oid)
    assert o["status"] == "CANCELLED"
    st = [t["status"] for t in o["tasks"]]
    assert st[0] == "SUCCEEDED" and st[-1] == "SKIPPED"
    assert sum(1 for x in drv.calls if x[0] == "pick") == 1
