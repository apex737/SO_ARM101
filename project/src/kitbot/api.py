"""Pi 게이트웨이 REST API (G1). 주문은 202로 접수하고 실행은 작업자가 한다 (D07)."""
from __future__ import annotations

import json
import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI, Query, Request
from fastapi.encoders import jsonable_encoder
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

from . import __version__
from .config import Settings
from .robot import build_driver
from .schemas import OrderCreate, check_limits
from .store import AlreadyFinished, IdempotencyConflict, NotFound, QueueFull, Store
from .worker import Worker

log = logging.getLogger("kitbot.api")


def _client(request: Request) -> str | None:
    return request.client.host if request.client else None


def create_app(settings: Settings | None = None, driver=None) -> FastAPI:
    settings = settings or Settings.from_env()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        store = Store(settings.db_path)
        drv = driver or build_driver(settings)
        drv.connect()
        worker = Worker(store, drv, settings)
        app.state.settings, app.state.store, app.state.driver, app.state.worker = settings, store, drv, worker
        store.event("info", "server.start", f"driver={getattr(drv, 'name', '?')}")
        worker.start()
        try:
            yield
        finally:
            worker.stop()
            store.event("info", "server.stop", "shutdown")
            store.close()

    app = FastAPI(title="KitBot Gateway", version=__version__, lifespan=lifespan)

    def audit_and_reply(request: Request, body_text: str | None, request_id: str | None,
                        status: int, content: dict, order_id: str | None = None, headers=None):
        payload = json.dumps(content, ensure_ascii=False)
        request.app.state.store.audit(request.method, request.url.path, _client(request), request_id,
                                      body_text, status, payload, order_id)
        return JSONResponse(content, status_code=status, headers=headers)

    @app.exception_handler(RequestValidationError)
    async def on_invalid(request: Request, exc: RequestValidationError):
        content = {"error": "validation_error", "detail": jsonable_encoder(exc.errors())}
        if request.method == "POST" and request.url.path == "/orders":
            body = exc.body
            if isinstance(body, (bytes, bytearray)):
                body_text = body.decode("utf-8", "replace")
            else:
                body_text = json.dumps(jsonable_encoder(body), ensure_ascii=False) if body is not None else None
            rid = body.get("request_id") if isinstance(body, dict) and isinstance(body.get("request_id"), str) else None
            return audit_and_reply(request, body_text, rid, 422, content)
        return JSONResponse(content, status_code=422)

    @app.post("/orders", status_code=202, summary="키트 주문 접수")
    def create_order(order: OrderCreate, request: Request):
        st: Store = request.app.state.store
        body_text = order.model_dump_json()
        errs = check_limits(order, settings)
        if errs:
            return audit_and_reply(request, body_text, order.request_id, 422,
                                   {"error": "validation_error", "detail": errs})
        items = [i.model_dump() for i in order.items]
        try:
            oid, created = st.create_order(order.request_id, items, order.priority, settings.max_queued_orders)
        except IdempotencyConflict as e:
            return audit_and_reply(request, body_text, order.request_id, 409,
                                   {"error": "idempotency_conflict",
                                    "detail": "request_id already used with a different body",
                                    "order_id": str(e)}, order_id=str(e))
        except QueueFull as e:
            return audit_and_reply(request, body_text, order.request_id, 503,
                                   {"error": "queue_full", "detail": f"{e} orders queued"},
                                   headers={"Retry-After": "10"})
        if created:
            request.app.state.worker.notify()
        content = st.get_order(oid)
        return audit_and_reply(request, body_text, order.request_id, 202 if created else 200, content, oid,
                               headers=None if created else {"Idempotent-Replay": "true"})

    @app.get("/orders", summary="주문 목록")
    def list_orders(request: Request, status: str | None = None, limit: int = Query(50, ge=1, le=500)):
        return request.app.state.store.list_orders(status, limit)

    @app.get("/orders/{order_id}", summary="주문 상세(작업·시도 포함)")
    def get_order(order_id: str, request: Request):
        try:
            return request.app.state.store.get_order(order_id)
        except NotFound:
            return JSONResponse({"error": "not_found"}, status_code=404)

    @app.post("/orders/{order_id}/cancel", summary="주문 취소")
    def cancel_order(order_id: str, request: Request):
        try:
            state = request.app.state.store.request_cancel(order_id)
        except NotFound:
            return JSONResponse({"error": "not_found"}, status_code=404)
        except AlreadyFinished as e:
            return JSONResponse({"error": "already_finished", "status": str(e)}, status_code=409)
        request.app.state.worker.notify()
        return {"order_id": order_id, "status": state,
                "detail": "cancelled" if state == "CANCELLED" else "will stop after current pick"}

    @app.get("/health", summary="게이트웨이·로봇 상태")
    def health(request: Request):
        w: Worker = request.app.state.worker
        fault = w.fault
        return {"status": "fault" if fault else ("ok" if w.alive else "worker_down"),
                "fault": fault, "worker_alive": w.alive, "current": w.current,
                "orders": request.app.state.store.counts(),
                "robot": request.app.state.driver.status(), "version": __version__}

    @app.post("/robot/reset", summary="FAULT 해제(운영자). 복구 동작 후 작업 재개")
    def robot_reset(request: Request):
        w: Worker = request.app.state.worker
        was = w.fault
        w.reset()
        return {"reset": True, "previous_fault": was}

    @app.get("/events", summary="이벤트 로그")
    def events(request: Request, after_id: int = 0, limit: int = Query(100, ge=1, le=1000),
               order_id: str | None = None):
        return request.app.state.store.list_events(after_id, limit, order_id)

    @app.get("/requests", summary="감사 기록: 요청·응답·실제 결과")
    def requests_log(request: Request, limit: int = Query(50, ge=1, le=500), request_id: str | None = None):
        return request.app.state.store.list_requests(limit, request_id)

    return app
