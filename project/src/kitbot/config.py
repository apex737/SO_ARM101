"""게이트웨이 설정. 환경변수 KITBOT_* 로 덮어쓴다 (값의 근거는 project/params.html)."""
from __future__ import annotations

import os
from dataclasses import dataclass, fields


@dataclass(frozen=True)
class Settings:
    db_path: str = "var/kitbot.db"
    colors: tuple[str, ...] = ("red", "green", "blue")
    max_qty_per_item: int = 5
    max_items_per_order: int = 10      # 한 주문의 qty 합계 한도
    max_queued_orders: int = 20        # 대기(QUEUED) 주문 한도. 넘으면 503
    max_attempts: int = 3              # 작업 하나의 실패 시도 한도
    attempt_timeout_s: float = 30.0    # pick 한 번에 주는 시간 한도(deadline)
    driver_grace_s: float = 5.0        # 한도 뒤 추가로 기다리는 시간. 넘기면 DRIVER_HUNG
    action_timeout_s: float = 15.0     # verify/place/recover 호출 한도
    worker_poll_s: float = 0.5         # 대기열이 비었을 때 다시 확인하는 주기
    driver: str = "mock"               # mock | (S03 이후 실제 드라이버)
    mock_success_rate: float = 0.8     # 서버 데모용 Mock 집기 성공률
    mock_pick_s: float = 1.5           # 서버 데모용 Mock 집기 소요 시간

    @classmethod
    def from_env(cls, **overrides) -> "Settings":
        values = {}
        for f in fields(cls):
            raw = os.environ.get("KITBOT_" + f.name.upper())
            if raw is None:
                continue
            default = getattr(cls, f.name)
            if isinstance(default, tuple):
                values[f.name] = tuple(s.strip() for s in raw.split(",") if s.strip())
            else:
                values[f.name] = type(default)(raw)
        values.update(overrides)
        return cls(**values)
