"""시험·데모용 가짜 로봇. outcomes로 시도별 결과를 정해 실패 시나리오를 재현한다."""
from __future__ import annotations

import random
import threading
import time

from .base import ActionResult, GraspCheck, PickResult

# outcomes 토큰: ok | grasp_fail | place_fail | timeout | error | hang
TOKENS = {"ok", "grasp_fail", "place_fail", "timeout", "error", "hang"}


class MockDriver:
    name = "mock"

    def __init__(self, outcomes=None, recover_outcomes=None, success_rate=1.0, pick_s=0.0, seed=None):
        bad = set(outcomes or []) - TOKENS
        if bad:
            raise ValueError(f"unknown outcome tokens: {bad}")
        self.outcomes = list(outcomes or [])
        self.recover_outcomes = list(recover_outcomes or [])
        self.success_rate = success_rate
        self.pick_s = pick_s
        self._rng = random.Random(seed)
        self._current = "ok"
        self._lock = threading.Lock()
        self.release = threading.Event()   # hang 시나리오를 시험 끝에 풀어 주는 용도
        self.calls: list[tuple] = []

    def connect(self) -> None:
        self.calls.append(("connect",))

    def _next(self) -> str:
        with self._lock:
            if self.outcomes:
                return self.outcomes.pop(0)
        return "ok" if self._rng.random() < self.success_rate else "grasp_fail"

    def pick(self, color, deadline):
        token = self._next()
        self._current = token
        self.calls.append(("pick", color, token))
        t0 = time.monotonic()
        if token == "hang":
            self.release.wait(timeout=30)
            return PickResult("DONE", {"mock": True})
        if token == "error":
            raise RuntimeError("mock driver error")
        if token == "timeout":
            time.sleep(max(0.0, min(self.pick_s, deadline - time.monotonic())))
            return PickResult("TIMEOUT", {"mock": True})
        if self.pick_s:
            time.sleep(min(self.pick_s, max(0.0, deadline - time.monotonic())))
        return PickResult("DONE", {"mock": True, "pick_ms": round((time.monotonic() - t0) * 1000, 1)})

    def verify_grasp(self, color):
        held = self._current not in ("grasp_fail",)
        self.calls.append(("verify", color, held))
        return GraspCheck(held, "mock" if held else "mock: gripper closed fully (empty)", {"mock": True})

    def place(self):
        ok = self._current != "place_fail"
        self.calls.append(("place", ok))
        return ActionResult(ok, "" if ok else "mock: dropped during transfer")

    def recover(self):
        with self._lock:
            ok = self.recover_outcomes.pop(0) if self.recover_outcomes else True
        self.calls.append(("recover", ok))
        return ActionResult(ok, "" if ok else "mock: recover failed")

    def status(self):
        return {"driver": self.name, "connected": True}
