"""PC 쪽 주문 클라이언트 (표준 라이브러리만 사용).

예: python -m kitbot.client --url http://raspberrypi.local:8000 red:2 blue:1
같은 --request-id로 다시 보내면 서버는 새 주문을 만들지 않고 기존 주문을 돌려준다.
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request
import uuid

FINAL = {"DONE", "FAILED", "CANCELLED"}


def call(method, url, body=None, timeout=5.0):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b"null")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"null")


def send_order(base, items, request_id, priority=0, retries=3):
    """응답을 못 받으면 같은 request_id로 재전송한다 (서버가 중복 생성을 막음)."""
    body = {"request_id": request_id, "items": items, "priority": priority}
    for i in range(retries):
        try:
            return call("POST", base + "/orders", body)
        except (urllib.error.URLError, TimeoutError) as e:
            print(f"send failed ({e}); retry {i + 1}/{retries}", file=sys.stderr)
            time.sleep(1 + i)
    raise SystemExit("server unreachable")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("items", nargs="+", help="color:qty, 예: red:2")
    p.add_argument("--url", default="http://127.0.0.1:8000")
    p.add_argument("--request-id", default=None)
    p.add_argument("--priority", type=int, default=0)
    p.add_argument("--no-wait", action="store_true")
    a = p.parse_args()
    items = []
    for it in a.items:
        color, _, qty = it.partition(":")
        items.append({"color": color, "qty": int(qty or 1)})
    rid = a.request_id or "pc-" + uuid.uuid4().hex[:12]
    code, resp = send_order(a.url.rstrip("/"), items, rid, a.priority)
    print(code, json.dumps(resp, ensure_ascii=False))
    if code not in (200, 202) or a.no_wait:
        return
    oid = resp["order_id"]
    while True:
        time.sleep(1)
        _, o = call("GET", f"{a.url.rstrip('/')}/orders/{oid}")
        done = sum(t["status"] == "SUCCEEDED" for t in o["tasks"])
        print(f"{o['status']:<9} {done}/{len(o['tasks'])} picks")
        if o["status"] in FINAL:
            print(json.dumps(o, ensure_ascii=False, indent=2))
            break


if __name__ == "__main__":
    main()
