"""SQLite 저장소. 주문·작업·시도·이벤트·감사 기록의 원본이자 작업 대기열(D01)."""
from __future__ import annotations

import hashlib
import json
import sqlite3
import threading
import time
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

from . import schemas as S

SCHEMA = """
CREATE TABLE IF NOT EXISTS orders (
    id TEXT PRIMARY KEY,
    request_id TEXT NOT NULL UNIQUE,
    body_hash TEXT NOT NULL,
    items_json TEXT NOT NULL,
    priority INTEGER NOT NULL DEFAULT 0,
    status TEXT NOT NULL,
    error TEXT,
    cancel_requested INTEGER NOT NULL DEFAULT 0,
    created_at REAL NOT NULL,
    started_at REAL,
    finished_at REAL,
    updated_at REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_orders_status ON orders(status, priority, created_at);
CREATE TABLE IF NOT EXISTS tasks (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    order_id TEXT NOT NULL REFERENCES orders(id),
    seq INTEGER NOT NULL,
    color TEXT NOT NULL,
    status TEXT NOT NULL,
    error TEXT,
    started_at REAL,
    finished_at REAL
);
CREATE INDEX IF NOT EXISTS idx_tasks_order ON tasks(order_id, seq);
CREATE TABLE IF NOT EXISTS attempts (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id INTEGER NOT NULL REFERENCES tasks(id),
    attempt_no INTEGER NOT NULL,
    result TEXT,
    detail_json TEXT,
    started_at REAL NOT NULL,
    finished_at REAL
);
CREATE INDEX IF NOT EXISTS idx_attempts_task ON attempts(task_id);
CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL NOT NULL,
    level TEXT NOT NULL,
    type TEXT NOT NULL,
    message TEXT NOT NULL,
    order_id TEXT,
    task_id INTEGER,
    data_json TEXT
);
CREATE TABLE IF NOT EXISTS api_requests (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts REAL NOT NULL,
    method TEXT NOT NULL,
    path TEXT NOT NULL,
    client TEXT,
    request_id TEXT,
    body TEXT,
    status_code INTEGER NOT NULL,
    response TEXT,
    order_id TEXT
);
CREATE TABLE IF NOT EXISTS system_state (
    key TEXT PRIMARY KEY,
    value TEXT
);
"""


class IdempotencyConflict(Exception):
    """같은 request_id로 다른 본문이 들어옴."""


class QueueFull(Exception):
    pass


class NotFound(Exception):
    pass


class AlreadyFinished(Exception):
    pass


def iso(ts: float | None) -> str | None:
    if ts is None:
        return None
    return datetime.fromtimestamp(ts, timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def body_hash(items: list[dict], priority: int) -> str:
    canon = json.dumps({"items": items, "priority": priority}, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canon.encode()).hexdigest()


class Store:
    def __init__(self, path: str):
        if path != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(path, check_same_thread=False, isolation_level=None)
        self._conn.row_factory = sqlite3.Row
        self._lock = threading.RLock()  # API 스레드들과 작업자 스레드가 연결 하나를 나눠 씀
        with self._lock:
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.execute("PRAGMA synchronous=FULL")  # 전원 차단 시에도 커밋된 상태 보존 우선
            self._conn.execute("PRAGMA foreign_keys=ON")
            self._conn.executescript(SCHEMA)

    def close(self):
        with self._lock:
            self._conn.close()

    @contextmanager
    def tx(self):
        with self._lock:
            self._conn.execute("BEGIN IMMEDIATE")
            try:
                yield self._conn
            except BaseException:
                self._conn.execute("ROLLBACK")
                raise
            self._conn.execute("COMMIT")

    def _q(self, sql, args=()):
        with self._lock:
            return self._conn.execute(sql, args).fetchall()

    # ---------- 주문 접수 ----------
    def create_order(self, request_id: str, items: list[dict], priority: int, max_queued: int):
        """반환: (order_id, created). 같은 요청 재전송이면 created=False."""
        h = body_hash(items, priority)
        now = time.time()
        with self.tx() as c:
            row = c.execute("SELECT id, body_hash FROM orders WHERE request_id=?", (request_id,)).fetchone()
            if row:
                if row["body_hash"] != h:
                    raise IdempotencyConflict(row["id"])
                return row["id"], False
            queued = c.execute("SELECT COUNT(*) FROM orders WHERE status=?", (S.QUEUED,)).fetchone()[0]
            if queued >= max_queued:
                raise QueueFull(queued)
            oid = "ord_" + uuid.uuid4().hex[:12]
            c.execute(
                "INSERT INTO orders(id, request_id, body_hash, items_json, priority, status, created_at, updated_at)"
                " VALUES (?,?,?,?,?,?,?,?)",
                (oid, request_id, h, json.dumps(items), priority, S.QUEUED, now, now))
            seq = 0
            for item in items:
                for _ in range(item["qty"]):
                    seq += 1
                    c.execute("INSERT INTO tasks(order_id, seq, color, status) VALUES (?,?,?,?)",
                              (oid, seq, item["color"], S.PENDING))
            self._event(c, "info", "order.accepted", f"{seq} picks queued", oid, None,
                        {"request_id": request_id, "items": items, "priority": priority})
        return oid, True

    # ---------- 조회 ----------
    def get_order(self, order_id: str) -> dict:
        with self._lock:
            o = self._conn.execute("SELECT * FROM orders WHERE id=?", (order_id,)).fetchone()
            if not o:
                raise NotFound(order_id)
            tasks = self._conn.execute("SELECT * FROM tasks WHERE order_id=? ORDER BY seq", (order_id,)).fetchall()
            atts = self._conn.execute(
                "SELECT a.* FROM attempts a JOIN tasks t ON a.task_id=t.id WHERE t.order_id=? ORDER BY a.id",
                (order_id,)).fetchall()
        by_task: dict[int, list] = {}
        for a in atts:
            by_task.setdefault(a["task_id"], []).append({
                "no": a["attempt_no"], "result": a["result"],
                "started_at": iso(a["started_at"]), "finished_at": iso(a["finished_at"]),
                "duration_ms": round((a["finished_at"] - a["started_at"]) * 1000, 1) if a["finished_at"] else None,
                "detail": json.loads(a["detail_json"]) if a["detail_json"] else None,
            })
        return {
            "order_id": o["id"], "request_id": o["request_id"], "status": o["status"],
            "priority": o["priority"], "items": json.loads(o["items_json"]), "error": o["error"],
            "cancel_requested": bool(o["cancel_requested"]),
            "created_at": iso(o["created_at"]), "started_at": iso(o["started_at"]),
            "finished_at": iso(o["finished_at"]),
            "tasks": [{"task_id": t["id"], "seq": t["seq"], "color": t["color"], "status": t["status"],
                       "error": t["error"], "attempts": by_task.get(t["id"], [])} for t in tasks],
        }

    def list_orders(self, status: str | None = None, limit: int = 50) -> list[dict]:
        sql = "SELECT id, request_id, status, priority, error, created_at, finished_at FROM orders"
        args: tuple = ()
        if status:
            sql += " WHERE status=?"
            args = (status,)
        sql += " ORDER BY created_at DESC LIMIT ?"
        rows = self._q(sql, args + (limit,))
        return [{"order_id": r["id"], "request_id": r["request_id"], "status": r["status"],
                 "priority": r["priority"], "error": r["error"],
                 "created_at": iso(r["created_at"]), "finished_at": iso(r["finished_at"])} for r in rows]

    def counts(self) -> dict:
        rows = self._q("SELECT status, COUNT(*) n FROM orders GROUP BY status")
        return {r["status"]: r["n"] for r in rows}

    # ---------- 취소 ----------
    def request_cancel(self, order_id: str) -> str:
        now = time.time()
        with self.tx() as c:
            o = c.execute("SELECT status FROM orders WHERE id=?", (order_id,)).fetchone()
            if not o:
                raise NotFound(order_id)
            if o["status"] in S.ORDER_FINAL:
                raise AlreadyFinished(o["status"])
            if o["status"] == S.QUEUED:
                c.execute("UPDATE orders SET status=?, cancel_requested=1, finished_at=?, updated_at=? WHERE id=?",
                          (S.CANCELLED, now, now, order_id))
                c.execute("UPDATE tasks SET status=? WHERE order_id=? AND status=?", (S.SKIPPED, order_id, S.PENDING))
                self._event(c, "info", "order.cancelled", "cancelled while queued", order_id)
                return S.CANCELLED
            c.execute("UPDATE orders SET cancel_requested=1, updated_at=? WHERE id=?", (now, order_id))
            self._event(c, "info", "order.cancel_requested", "will stop after current pick", order_id)
            return S.RUNNING

    def cancel_requested(self, order_id: str) -> bool:
        return bool(self._q("SELECT cancel_requested FROM orders WHERE id=?", (order_id,))[0][0])

    # ---------- 작업자용 ----------
    def claim_next_order(self) -> str | None:
        """이어 갈 RUNNING 주문이 있으면 먼저, 없으면 우선순위·접수 순으로 QUEUED 하나를 RUNNING으로."""
        now = time.time()
        with self.tx() as c:
            row = c.execute("SELECT id FROM orders WHERE status=? ORDER BY started_at LIMIT 1", (S.RUNNING,)).fetchone()
            if row:
                return row["id"]
            row = c.execute("SELECT id FROM orders WHERE status=? ORDER BY priority DESC, created_at LIMIT 1",
                            (S.QUEUED,)).fetchone()
            if not row:
                return None
            c.execute("UPDATE orders SET status=?, started_at=?, updated_at=? WHERE id=?",
                      (S.RUNNING, now, now, row["id"]))
            self._event(c, "info", "order.started", "claimed by worker", row["id"])
            return row["id"]

    def pending_tasks(self, order_id: str) -> list[dict]:
        rows = self._q("SELECT id, seq, color FROM tasks WHERE order_id=? AND status IN (?,?) ORDER BY seq",
                       (order_id, S.PENDING, S.RUNNING))
        return [dict(r) for r in rows]

    def set_task(self, task_id: int, status: str, error: str | None = None):
        now = time.time()
        with self.tx() as c:
            if status == S.RUNNING:
                c.execute("UPDATE tasks SET status=?, started_at=COALESCE(started_at, ?) WHERE id=?",
                          (status, now, task_id))
            elif status == S.PENDING:
                c.execute("UPDATE tasks SET status=? WHERE id=?", (status, task_id))
            else:
                c.execute("UPDATE tasks SET status=?, error=?, finished_at=? WHERE id=?",
                          (status, error, now, task_id))

    def failed_attempts(self, task_id: int) -> int:
        marks = ",".join("?" * len(S.NOT_COUNTED))
        return self._q(f"SELECT COUNT(*) FROM attempts WHERE task_id=? AND result IS NOT NULL"
                       f" AND result NOT IN ({marks})", (task_id, *S.NOT_COUNTED))[0][0]

    def start_attempt(self, task_id: int) -> tuple[int, int]:
        with self.tx() as c:
            no = c.execute("SELECT COALESCE(MAX(attempt_no),0)+1 FROM attempts WHERE task_id=?",
                           (task_id,)).fetchone()[0]
            cur = c.execute("INSERT INTO attempts(task_id, attempt_no, started_at) VALUES (?,?,?)",
                            (task_id, no, time.time()))
            return cur.lastrowid, no

    def finish_attempt(self, attempt_id: int, result: str, detail: dict | None):
        with self.tx() as c:
            c.execute("UPDATE attempts SET result=?, detail_json=?, finished_at=? WHERE id=?",
                      (result, json.dumps(detail) if detail is not None else None, time.time(), attempt_id))

    def finish_order(self, order_id: str, status: str, error: str | None = None):
        now = time.time()
        with self.tx() as c:
            c.execute("UPDATE tasks SET status=? WHERE order_id=? AND status IN (?,?)",
                      (S.SKIPPED, order_id, S.PENDING, S.RUNNING))
            c.execute("UPDATE orders SET status=?, error=?, finished_at=?, updated_at=? WHERE id=?",
                      (status, error, now, now, order_id))
            self._event(c, "error" if status == S.FAILED else "info", "order." + status.lower(),
                        error or status, order_id)

    def interrupt_inflight(self) -> int:
        """시작 시 호출. 끝나지 않은 시도를 INTERRUPTED로, RUNNING 작업을 PENDING으로 되돌린다."""
        now = time.time()
        with self.tx() as c:
            n = c.execute("UPDATE attempts SET result=?, finished_at=? WHERE result IS NULL",
                          (S.INTERRUPTED, now)).rowcount
            c.execute("UPDATE tasks SET status=? WHERE status=?", (S.PENDING, S.RUNNING))
            if n:
                self._event(c, "warning", "startup.interrupted", f"{n} in-flight attempt(s) marked INTERRUPTED")
            return n

    # ---------- 시스템 상태 (FAULT 등) ----------
    def get_state(self, key: str) -> str | None:
        rows = self._q("SELECT value FROM system_state WHERE key=?", (key,))
        return rows[0][0] if rows else None

    def set_state(self, key: str, value: str | None):
        with self.tx() as c:
            if value is None:
                c.execute("DELETE FROM system_state WHERE key=?", (key,))
            else:
                c.execute("INSERT INTO system_state(key, value) VALUES (?,?)"
                          " ON CONFLICT(key) DO UPDATE SET value=excluded.value", (key, value))

    # ---------- 이벤트 ----------
    def _event(self, c, level, type_, message, order_id=None, task_id=None, data=None):
        c.execute("INSERT INTO events(ts, level, type, message, order_id, task_id, data_json) VALUES (?,?,?,?,?,?,?)",
                  (time.time(), level, type_, message, order_id, task_id,
                   json.dumps(data) if data is not None else None))

    def event(self, level, type_, message, order_id=None, task_id=None, data=None):
        with self.tx() as c:
            self._event(c, level, type_, message, order_id, task_id, data)

    def list_events(self, after_id: int = 0, limit: int = 100, order_id: str | None = None) -> list[dict]:
        sql, args = "SELECT * FROM events WHERE id>?", [after_id]
        if order_id:
            sql += " AND order_id=?"
            args.append(order_id)
        rows = self._q(sql + " ORDER BY id LIMIT ?", (*args, limit))
        return [{"id": r["id"], "ts": iso(r["ts"]), "level": r["level"], "type": r["type"],
                 "message": r["message"], "order_id": r["order_id"], "task_id": r["task_id"],
                 "data": json.loads(r["data_json"]) if r["data_json"] else None} for r in rows]

    # ---------- 감사 기록 (D05) ----------
    def audit(self, method, path, client, request_id, body, status_code, response, order_id):
        with self.tx() as c:
            c.execute("INSERT INTO api_requests(ts, method, path, client, request_id, body, status_code, response,"
                      " order_id) VALUES (?,?,?,?,?,?,?,?,?)",
                      (time.time(), method, path, client, request_id, body, status_code, response, order_id))

    def list_requests(self, limit: int = 50, request_id: str | None = None) -> list[dict]:
        sql = ("SELECT r.*, o.status AS order_status, o.error AS order_error FROM api_requests r"
               " LEFT JOIN orders o ON o.id=r.order_id")
        args: tuple = ()
        if request_id:
            sql += " WHERE r.request_id=?"
            args = (request_id,)
        rows = self._q(sql + " ORDER BY r.id DESC LIMIT ?", args + (limit,))
        return [{"id": r["id"], "ts": iso(r["ts"]), "method": r["method"], "path": r["path"],
                 "client": r["client"], "request_id": r["request_id"], "body": r["body"],
                 "status_code": r["status_code"], "response": r["response"], "order_id": r["order_id"],
                 "order_status": r["order_status"], "order_error": r["order_error"]} for r in rows]
