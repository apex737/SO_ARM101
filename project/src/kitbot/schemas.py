"""요청 형식과 상태 값."""
from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field

from .config import Settings

# 주문
QUEUED, RUNNING, DONE, FAILED, CANCELLED = "QUEUED", "RUNNING", "DONE", "FAILED", "CANCELLED"
ORDER_FINAL = {DONE, FAILED, CANCELLED}
# 작업(물체 하나 집기)
PENDING, SUCCEEDED, SKIPPED = "PENDING", "SUCCEEDED", "SKIPPED"
# 시도 결과
SUCCESS = "SUCCESS"
GRASP_FAIL = "GRASP_FAIL"      # 집기 동작은 끝났지만 물체를 쥐지 못함
PLACE_FAIL = "PLACE_FAIL"      # 옮기다 놓침 또는 놓기 실패
TIMEOUT = "TIMEOUT"            # 드라이버가 시간 한도 안에 집기를 끝내지 못함
DRIVER_ERROR = "DRIVER_ERROR"  # 드라이버 예외
DRIVER_HUNG = "DRIVER_HUNG"    # 한도+여유 안에 드라이버가 돌아오지 않음 → FAULT
INTERRUPTED = "INTERRUPTED"    # 서버 재시작으로 끊김. 실패 횟수에 넣지 않음
NOT_COUNTED = {SUCCESS, INTERRUPTED}


class OrderItem(BaseModel):
    model_config = ConfigDict(extra="forbid")
    color: str = Field(min_length=1, max_length=32)
    qty: int = Field(ge=1)


class OrderCreate(BaseModel):
    """PC가 보내는 키트 주문. request_id는 재전송 시 같은 값을 써야 한다(멱등성 키)."""
    model_config = ConfigDict(extra="forbid")
    request_id: str = Field(min_length=1, max_length=64, pattern=r"^[A-Za-z0-9_.:-]+$")
    items: list[OrderItem] = Field(min_length=1)
    priority: int = Field(default=0, ge=0, le=9)


def check_limits(order: OrderCreate, s: Settings) -> list[dict]:
    """설정에 따라 달라지는 검증. 형식 검증(pydantic)을 통과한 뒤 실행한다."""
    errors = []
    for i, item in enumerate(order.items):
        if item.color not in s.colors:
            errors.append({"loc": ["body", "items", i, "color"],
                           "msg": f"unknown color; allowed: {list(s.colors)}", "type": "color"})
        if item.qty > s.max_qty_per_item:
            errors.append({"loc": ["body", "items", i, "qty"],
                           "msg": f"qty must be <= {s.max_qty_per_item}", "type": "qty_limit"})
    total = sum(item.qty for item in order.items)
    if total > s.max_items_per_order:
        errors.append({"loc": ["body", "items"],
                       "msg": f"total qty {total} > {s.max_items_per_order}", "type": "order_limit"})
    return errors
