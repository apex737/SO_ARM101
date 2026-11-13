"""순수 IK(pick.py)와 IK+ACT(run_act.py) 비교 실험의 공통 부분 (S05-5).

- 자리: 기본은 작업 영역 아무 곳(일반화 성능). 블록 자리는 접근할 때 위 카메라가 찾은 값으로 자동 기록하고,
  가장 가까운 시연(학습에 쓴 41개) 자리까지 거리로 나눠 본다(가까움 < 20mm ≤ 중간 < 40mm ≤ 멂).
  --grid를 주면 평가 격자 g01~g09를 순서대로 안내하고 자리를 확인한다(S02와 같은 자리 비교용).
- 같은 판정: 아래 코드 하나 + 메모. 애매하면 '?'로 두고 끝 사진으로 나중에 정한다.
- 같은 기록: var/compare/<이름>.jsonl 한 줄 = 한 시행, 사진은 var/compare/<이름>/
- 결과 비교:  python -m kitbot_exec.compare var/compare/ik_v1.jsonl var/compare/act_v1.jsonl

판정 코드 (사람이 눈으로)
  s  성공: 블록 전체가 바구니 안에 있고(바닥이든 다른 블록 위든), 시행 중 사람이 손대지 않음
  g  못 잡음: 들어 올릴 때 블록이 그리퍼에 없음
  d  떨어뜨림: 잡아서 옮기다가 바구니에 닿기 전에 떨어짐
  b  바구니 밖: 놓았지만 바구니 밖이거나 테두리에 걸쳐 있음
  x  개입: 사람이 손대거나 비상 정지(충돌·위험)
  ?  애매: 위 어디에도 확실히 안 맞음 → 끝 사진으로 나중에 판정
애매할 때 규칙
  - 테두리에 걸쳐 일부만 안에 있으면 b (바구니 바닥이나 안쪽 블록에 닿아 있지 않으면 성공 아님)
  - 놓친 블록이 튕겨서 우연히 바구니에 들어가도 s (결과 기준). 메모에 '튕김'
  - 다시 집어서 성공하면 s. 메모에 '재시도'
  - 바구니에는 넣었는데 HOME으로 못 돌아옴 → 판정은 s, HOME 여부는 따로 자동 기록(home_ok)
  - 사람이 조금이라도 손대면 결과와 상관없이 x
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parents[1]
OUT_DIR = SRC / "var" / "compare"
CODES = {"s": "성공", "g": "못 잡음", "d": "떨어뜨림", "b": "바구니 밖·테두리", "x": "사람 개입·비상 정지", "?": "애매(사진으로 나중에)"}
VERDICT_PROMPT = "  판정? s 성공 / g 못 잡음 / d 떨어뜨림 / b 바구니 밖·테두리 / x 개입 / ? 애매 / h 규칙 보기  (뒤에 메모 가능, 예: 's 튕김') "
RULES = """── 판정 규칙 (시행이 끝나고 팔이 HOME에 간 뒤, 눈으로) ──
  s  성공      블록 전체가 바구니 안 (바닥이든 다른 블록 위든), 시행 중 사람이 손대지 않음
  g  못 잡음   들어 올릴 때 블록이 그리퍼에 없음
  d  떨어뜨림  잡아서 옮기다 바구니에 닿기 전에 떨어짐
  b  바구니 밖 놓았지만 바구니 밖이거나 테두리에 걸쳐 있음
  x  개입      사람이 손대거나 비상 정지
  ?  애매      아래 규칙으로도 모르겠음 → 끝 사진으로 나중에 (성공률에서 빠짐)
 애매할 때
  - 테두리에 걸쳐 일부만 안            → b
  - 튕겨서 우연히 들어감 / 다시 집어 성공 → s + 메모 ('s 튕김', 's 재시도')
  - 넣었는데 HOME으로 못 돌아옴          → s (HOME 여부는 자동 기록)
  - 사람이 조금이라도 손댐               → x
──────────────────────────────────────────────────"""


DEMO_META = Path.home() / ".cache/huggingface/lerobot/local/kitbot_act_v1_clean/kitbot_episode_meta.jsonl"
DEMO_VAL = (6, 13, 22, 27, 36)   # loss_curve.VAL_EPISODES와 같음: 학습에 안 쓴 시연
NEAR_MM = (20.0, 40.0)


def demo_positions(path: Path = DEMO_META, exclude=DEMO_VAL) -> np.ndarray:
    """학습에 쓴 시연들의 블록 자리 (N×2, 로봇 x·y mm). 기록이 없으면 빈 배열."""
    if not Path(path).exists():
        return np.zeros((0, 2))
    rows = [json.loads(l) for l in Path(path).read_text(encoding="utf-8").splitlines() if l.strip()]
    return np.array([r["block_camera"][:2] for r in rows if r.get("episode_index") not in exclude], float).reshape(-1, 2)


def near_label(d: float | None) -> str:
    if d is None:
        return "시연거리?"
    return f"시연<{NEAR_MM[0]:.0f}" if d < NEAR_MM[0] else (f"시연<{NEAR_MM[1]:.0f}" if d < NEAR_MM[1] else f"시연≥{NEAR_MM[1]:.0f}")


def place_record(calib, x: float, y: float, demos: np.ndarray | None = None) -> dict:
    """블록 자리 기록: 좌표, 구역(평가 격자 안/보정점 범위 안/밖), 가장 가까운 격자점·시연 자리까지 거리."""
    from .exp_s02 import place_info

    demos = demo_positions() if demos is None else demos
    out = {"block_xy": [round(float(x), 1), round(float(y), 1)], **place_info(calib, x, y)}
    out["nearest_demo_mm"] = round(float(np.min(np.linalg.norm(demos - [x, y], axis=1))), 1) if len(demos) else None
    out["near"] = near_label(out["nearest_demo_mm"])
    from .pick import outside_mm
    oc = outside_mm(calib, x, y)
    out["outside_calib_mm"] = None if not np.isfinite(oc) else round(max(float(oc), 0.0), 1)   # 0 = 보정점 범위 안
    return out


def parse_queue(text: str, colors) -> list[tuple[str, int | None]]:
    """'orange 2, yellow 1, blue' 또는 쉼표 없이 'orange 2 yellow 1 blue' → [('orange', 2), ('yellow', 1), ('blue', None)].
    수를 안 쓰거나 all이면 안 보일 때까지."""
    words = text.replace(",", " ").replace(";", " ").lower().split()
    out, i = [], 0
    while i < len(words):
        color = words[i]
        if color not in colors:
            raise SystemExit(f"큐: 모르는 색 '{color}'. 가능: {', '.join(sorted(colors))} (예: \"orange 2, yellow 1\")")
        n = None
        nxt = words[i + 1] if i + 1 < len(words) else None
        if nxt is not None and nxt not in colors:
            if nxt in ("all", "*"):
                n = None
            elif nxt.isdigit() and int(nxt) >= 1:
                n = int(nxt)
            else:
                raise SystemExit(f"큐: '{color} {nxt}'의 개수는 1 이상의 수 또는 all")
            i += 1
        out.append((color, n))
        i += 1
    if not out:
        raise SystemExit("큐가 비어 있음. 예: --queue \"orange 2, yellow 1\"")
    return out


class PickQueue:
    """명령을 차례로: 색마다 최대 n개(없으면 안 보일 때까지) 집는다.
    그 색이 안 보이면, 또는 연속 max_fail번 실패하면 다음 명령으로 넘어간다
    (닿지 않는 블록 하나 때문에 끝없이 되풀이하지 않게)."""

    def __init__(self, items, max_fail: int = 2):
        self.items, self.max_fail = list(items), max_fail
        self.i, self.ok, self.fails = 0, 0, 0
        self.log = []                                  # (색, 목표, 집은 수, 넘어간 이유)

    def current(self) -> str | None:
        return self.items[self.i][0] if self.i < len(self.items) else None

    def label(self) -> str:
        color, n = self.items[self.i]
        return f"큐 {self.i + 1}/{len(self.items)}: {color} {self.ok + 1}번째" + (f" (최대 {n})" if n else " (안 보일 때까지)")

    def _next(self, why: str) -> None:
        color, n = self.items[self.i]
        self.log.append((color, n, self.ok, why))
        print(f"  → {color}: {self.ok}개 집음, {why} → 다음 명령")
        self.i, self.ok, self.fails = self.i + 1, 0, 0

    def report(self, outcome: str) -> None:
        """outcome: 'ok'(집어서 넣음) / 'fail'(시도했으나 실패) / 'none'(그 색이 안 보임)"""
        n = self.items[self.i][1]
        if outcome == "none":
            self._next("더 안 보임")
        elif outcome == "ok":
            self.ok, self.fails = self.ok + 1, 0
            if n and self.ok >= n:
                self._next(f"목표 {n}개 채움")
        else:
            self.fails += 1
            if self.fails >= self.max_fail:
                self._next(f"연속 {self.fails}번 실패")

    def summary(self) -> str:
        return "큐 결과: " + ", ".join(f"{c} {ok}" + (f"/{n}" if n else "") + f" ({why})" for c, n, ok, why in self.log)


QUEUE_VERDICT = {"verdict": "?", "success": False, "note": "큐 실행(사람 판정 없음)"}


def print_rules() -> None:
    print(RULES)


def schedule(n: int) -> list[str]:
    """g01~g09를 순서대로 반복."""
    from .view_blocks import S02_GRID

    names = sorted(S02_GRID)
    return [names[i % len(names)] for i in range(n)]


def parse_verdict(ans: str) -> dict | None:
    ans = ans.strip()
    if not ans:
        return None
    code = ans[0].lower()
    if code not in CODES:
        return None
    return {"verdict": code, "success": code == "s", "note": ans[1:].strip()}


def ask_verdict(ask=None) -> dict:
    ask = ask or input
    while True:
        ans = ask(VERDICT_PROMPT)
        if ans.strip().lower() == "h":
            print_rules()
            continue
        v = parse_verdict(ans)
        if v is not None:
            return v
        print("  코드 하나로 시작해 주세요:", ", ".join(f"{k}={t}" for k, t in CODES.items()))


def grid_offset(x: float, y: float, grid: str) -> float:
    from .view_blocks import S02_GRID

    gx, gy = S02_GRID[grid]
    return float(np.hypot(x - gx, y - gy))


def precheck_place(cap, gains, color, calib, grid: str, tol_mm: float, ask=None) -> dict:
    """놓은 블록이 격자점 근처인지 위 카메라로 확인. 멀면 다시 놓게 한다. 안 보이면 그대로 진행할지 묻는다."""
    from .pick import detect_block_pose
    from .view_blocks import S02_GRID

    ask = ask or input
    while True:
        pose, _, note = detect_block_pose(cap, gains, color, calib)
        if pose is None:
            if ask(f"  블록 확인 못 함({note}). Enter = 그래도 진행, r = 다시 보기 ").strip().lower() == "r":
                continue
            return {"grid": grid, "precheck": "not_seen"}
        x, y = pose[0], pose[1]
        off = grid_offset(x, y, grid)
        gx, gy = S02_GRID[grid]
        if off <= tol_mm:
            print(f"  자리 확인: {grid}에서 {off:.0f}mm (x={x:.0f}, y={y:.0f})")
            return {"grid": grid, "precheck": "ok", "grid_offset_mm": round(off, 1)}
        ans = ask(f"  {grid}(x={gx}, y={gy})에서 {off:.0f}mm 떨어짐 (x={x:.0f}, y={y:.0f}). "
                  f"x {gx - x:+.0f}, y {gy - y:+.0f}mm 옮기고 Enter / f = 이대로 진행 ").strip().lower()
        if ans == "f":
            return {"grid": grid, "precheck": "forced", "grid_offset_mm": round(off, 1)}


def save_photos(folder: Path, tag: str, frames: dict) -> dict:
    """{'top': BGR, 'wrist': BGR} → jpg 저장, 경로 dict."""
    import cv2

    folder.mkdir(parents=True, exist_ok=True)
    out = {}
    for name, img in frames.items():
        if img is None:
            continue
        p = folder / f"{time.strftime('%m%d_%H%M%S')}_{tag}_{name}.jpg"   # 시각을 붙여 같은 이름으로 다시 돌려도 덮어쓰지 않게
        cv2.imwrite(str(p), img)
        out[name] = str(p.relative_to(SRC)) if p.is_relative_to(SRC) else str(p)
    return out


def append(path: Path, rec: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False, default=float) + "\n")


def read(path: Path) -> list[dict]:
    return [json.loads(l) for l in Path(path).read_text(encoding="utf-8").splitlines() if l.strip()]


def summarize(rows: list[dict]) -> dict:
    n = len(rows)
    by = {k: sum(r.get("verdict") == k for r in rows) for k in CODES}
    judged = n - by["?"]
    out = {"n": n, "success": by["s"], "rate": by["s"] / judged if judged else 0.0, "codes": by,
           "home_ok": sum(bool(r.get("home_ok")) for r in rows),
           "mean_s": float(np.mean([r["duration_s"] for r in rows if r.get("duration_s")])) if any(r.get("duration_s") for r in rows) else None}
    groups = {}
    for r in rows:
        g = r.get("grid") or r.get("near") or "자리?"
        groups.setdefault(g, []).append(r.get("verdict", "?"))
    out["by_group"] = {g: "".join(v) for g, v in sorted(groups.items())}
    return out


def report(paths: list[str]) -> str:
    lines = []
    sums = [(Path(p).stem, read(Path(p))) for p in paths]
    head = f"{'기록':<16}{'방식':<8}{'시행':>4}{'성공':>5}{'성공률':>8}  " + " ".join(f"{k}" for k in CODES) + "   HOME  평균초"
    lines.append(head)
    for name, rows in sums:
        s = summarize(rows)
        meth = rows[0].get("method", "?") if rows else "?"
        codes = " ".join(str(s["codes"][k]) for k in CODES)
        mean = f"{s['mean_s']:.1f}" if s["mean_s"] else "-"
        lines.append(f"{name:<16}{meth:<8}{s['n']:>4}{s['success']:>5}{s['rate'] * 100:>7.0f}%  {codes}   {s['home_ok']:>4}  {mean:>5}")
    lines.append("\n자리별 판정 (격자 모드면 격자점, 아니면 가장 가까운 시연 자리까지 거리 [mm]) — 시행 순서대로")
    keys = sorted({g for _, rows in sums for g in summarize(rows)["by_group"]})
    lines.append(f"{'':<10}" + "".join(f"{name:<18}" for name, _ in sums))
    for g in keys:
        cells = []
        for _, rows in sums:
            v = summarize(rows)["by_group"].get(g, "")
            cells.append(f"{v} ({v.count('s')}/{len(v)})" if v else "-")
        lines.append(f"{g:<10}" + "".join(f"{c:<18}" for c in cells))
    lines.append("\n코드: " + ", ".join(f"{k}={t}" for k, t in CODES.items()) + "  (성공률은 '?' 제외)")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="var/compare/<이름>.jsonl 여러 개")
    args = ap.parse_args(argv)
    print(report(args.logs))


if __name__ == "__main__":
    main()
