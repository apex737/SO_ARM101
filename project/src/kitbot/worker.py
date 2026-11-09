"""단일 작업자: 대기열에서 주문을 꺼내 작업(집기)을 하나씩 실행한다.

시도 하나 = pick → verify_grasp → place. 실패하면 recover 후 재시도 (D03).
물리 상태를 모르게 되면 FAULT로 멈추고 운영자 리셋을 기다린다 (D04).
"""
from __future__ import annotations

import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout

from . import schemas as S
from .config import Settings
from .store import Store

log = logging.getLogger("kitbot.worker")
FAULT_KEY = "robot_fault"


class DriverHung(Exception):
    pass


class Worker:
    def __init__(self, store: Store, driver, settings: Settings):
        self.store, self.driver, self.s = store, driver, settings
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="driver")
        self._need_recover = True     # 시작 시 항상 알려진 자세로 (D04)
        self.current: dict | None = None   # 지금 실행 중인 주문/작업 (상태 조회용)

    # ---------- 수명 ----------
    def start(self):
        n = self.store.interrupt_inflight()
        if n:
            log.warning("startup: %d in-flight attempt(s) interrupted", n)
        self._thread = threading.Thread(target=self._run, name="kitbot-worker", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 5.0):
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout)
        self._executor.shutdown(wait=False, cancel_futures=True)

    def notify(self):
        self._wake.set()

    @property
    def alive(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    # ---------- FAULT ----------
    @property
    def fault(self) -> str | None:
        return self.store.get_state(FAULT_KEY)

    def _set_fault(self, reason: str, order_id=None, task_id=None):
        self.store.set_state(FAULT_KEY, reason)
        self.store.event("error", "robot.fault", reason, order_id, task_id)
        log.error("FAULT: %s", reason)

    def reset(self) -> None:
        """운영자 리셋. 다음 루프에서 recover가 성공해야 작업을 다시 시작한다."""
        self.store.set_state(FAULT_KEY, None)
        self._need_recover = True
        self.store.event("info", "robot.reset", "fault cleared by operator; recover pending")
        self._wake.set()

    # ---------- 드라이버 호출 (D08) ----------
    def _call(self, timeout: float, fn, *args):
        fut = self._executor.submit(fn, *args)
        try:
            return fut.result(timeout=timeout)
        except FutureTimeout:
            # 멈춘 스레드는 강제 종료할 수 없으므로 버리고 새 실행기를 만든다.
            self._executor.shutdown(wait=False, cancel_futures=True)
            self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="driver")
            raise DriverHung(f"{getattr(fn, '__name__', fn)} did not return within {timeout:.1f}s")

    def _recover(self, order_id=None, task_id=None) -> bool:
        try:
            r = self._call(self.s.action_timeout_s, self.driver.recover)
            ok, detail = r.ok, r.detail
        except Exception as e:  # DriverHung 포함
            ok, detail = False, f"{type(e).__name__}: {e}"
        self.store.event("info" if ok else "error", "recover.ok" if ok else "recover.failed",
                         detail or ("recovered" if ok else "recover failed"), order_id, task_id)
        if not ok:
            self._set_fault("recover failed: " + (detail or "unknown"), order_id, task_id)
        return ok

    # ---------- 주 루프 ----------
    def _run(self):
        while not self._stop.is_set():
            try:
                if self.fault:
                    self._idle()
                    continue
                if self._need_recover:
                    if not self._recover():
                        continue
                    self._need_recover = False
                order_id = self.store.claim_next_order()
                if not order_id:
                    self._idle()
                    continue
                self._run_order(order_id)
            except Exception:
                # 작업자 스레드가 죽으면 대기열 전체가 멈추므로, 예외는 FAULT로 바꾸고 계속 산다.
                log.exception("worker loop error")
                self._set_fault("worker internal error (see log)")
            finally:
                self.current = None

    def _idle(self):
        self._wake.wait(self.s.worker_poll_s)
        self._wake.clear()

    def _run_order(self, order_id: str):
        for task in self.store.pending_tasks(order_id):
            if self._stop.is_set():
                return
            if self.store.cancel_requested(order_id):
                self.store.finish_order(order_id, S.CANCELLED, "cancelled by request")
                return
            result = self._run_task(order_id, task)
            if result == "FAULT":
                return  # 주문은 RUNNING으로 남고, 리셋·복구 후 이어 간다
            if result == S.FAILED:
                self.store.finish_order(
                    order_id, S.FAILED,
                    f"task {task['seq']} ({task['color']}) failed after {self.s.max_attempts} attempts")
                return
        if self._stop.is_set():
            return
        if self.store.pending_tasks(order_id):  # 마지막 작업에서 취소로 빠져나온 경우
            self.store.finish_order(order_id, S.CANCELLED, "cancelled by request")
            return
        # 모든 작업이 성공했으면 취소 요청이 늦게 왔더라도 DONE으로 본다.
        self.store.finish_order(order_id, S.DONE)

    def _run_task(self, order_id: str, task: dict) -> str:
        tid, color = task["id"], task["color"]
        self.store.set_task(tid, S.RUNNING)
        while True:
            if self.store.failed_attempts(tid) >= self.s.max_attempts:
                self.store.set_task(tid, S.FAILED, f"{self.s.max_attempts} failed attempts")
                self.store.event("error", "task.failed", f"seq {task['seq']} {color}", order_id, tid)
                return S.FAILED
            if self.store.cancel_requested(order_id):
                self.store.set_task(tid, S.PENDING)
                return "CANCEL"  # _run_order가 다음 루프에서 취소 처리
            attempt_id, no = self.store.start_attempt(tid)
            self.current = {"order_id": order_id, "task_id": tid, "seq": task["seq"], "color": color, "attempt": no}
            result, detail = self._attempt(color)
            self.store.finish_attempt(attempt_id, result, detail)
            self.store.event("info" if result == S.SUCCESS else "warning", "attempt." + result.lower(),
                             f"seq {task['seq']} {color} attempt {no}", order_id, tid, detail)
            if result == S.SUCCESS:
                self.store.set_task(tid, S.SUCCEEDED)
                return S.SUCCEEDED
            if result == S.DRIVER_HUNG:
                self.store.set_task(tid, S.PENDING)
                self._need_recover = True
                self._set_fault("driver hung: " + detail.get("error", ""), order_id, tid)
                return "FAULT"
            # 원상 복구 후 재시도
            if not self._recover(order_id, tid):
                self.store.set_task(tid, S.PENDING)
                self._need_recover = True
                return "FAULT"

    def _attempt(self, color: str) -> tuple[str, dict]:
        detail: dict = {}
        t0 = time.monotonic()
        try:
            deadline = time.monotonic() + self.s.attempt_timeout_s
            pick = self._call(self.s.attempt_timeout_s + self.s.driver_grace_s, self.driver.pick, color, deadline)
            detail["pick_ms"] = round((time.monotonic() - t0) * 1000, 1)
            detail["pick_metrics"] = pick.metrics
            if pick.status == "TIMEOUT":
                return S.TIMEOUT, detail
            chk = self._call(self.s.action_timeout_s, self.driver.verify_grasp, color)
            detail["grasp"] = {"held": chk.held, "reason": chk.reason, "evidence": chk.evidence}
            if not chk.held:
                return S.GRASP_FAIL, detail
            t1 = time.monotonic()
            pl = self._call(self.s.action_timeout_s, self.driver.place)
            detail["place_ms"] = round((time.monotonic() - t1) * 1000, 1)
            if not pl.ok:
                detail["place"] = pl.detail
                return S.PLACE_FAIL, detail
            return S.SUCCESS, detail
        except DriverHung as e:
            detail["error"] = str(e)
            return S.DRIVER_HUNG, detail
        except Exception as e:
            detail["error"] = f"{type(e).__name__}: {e}"
            return S.DRIVER_ERROR, detail
        finally:
            detail["total_ms"] = round((time.monotonic() - t0) * 1000, 1)
