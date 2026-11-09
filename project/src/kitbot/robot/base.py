"""로봇 드라이버 인터페이스 (D06). 실제 SO-ARM101 드라이버는 S03 이후 같은 형태로 만든다."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol


@dataclass
class PickResult:
    status: str                      # "DONE" | "TIMEOUT"
    metrics: dict = field(default_factory=dict)   # 예: 추론 횟수, 추론 지연 통계, 소요 시간


@dataclass
class GraspCheck:
    held: bool
    reason: str = ""
    evidence: dict = field(default_factory=dict)  # 예: 그리퍼 위치·부하, 카메라 판정


@dataclass
class ActionResult:
    ok: bool
    detail: str = ""
    metrics: dict = field(default_factory=dict)


class RobotDriver(Protocol):
    name: str

    def connect(self) -> None: ...

    def pick(self, color: str, deadline: float) -> PickResult:
        """정책으로 color 물체를 집는다. time.monotonic()이 deadline을 넘으면 멈추고 TIMEOUT을 반환해야 한다."""

    def verify_grasp(self, color: str) -> GraspCheck: ...

    def place(self) -> ActionResult:
        """쥔 물체를 키트 박스로 옮겨 놓는다."""

    def recover(self) -> ActionResult:
        """그리퍼를 열고 대기 자세로 돌아간다. 실패하면 ok=False (→ FAULT)."""

    def status(self) -> dict: ...
