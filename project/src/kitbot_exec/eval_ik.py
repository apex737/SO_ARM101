"""순수 IK(학습 없음) 비교 실험: pick.py를 같은 18자리에서 반복하고, IK+ACT(run_act)와 같은 형식으로 기록한다 (S05-5).

    cd project/src
    python -m kitbot_exec.eval_ik --name ik_v1 --color purple

시행 하나: ① 블록을 작업 영역 아무 곳에 놓고 Enter (자리는 pick.py가 찾은 값으로 자동 기록)
             --grid면 격자점(g01→g09, 2바퀴)을 안내하고 위 카메라로 확인(멀면 다시 놓기)
          ② pick.py 한 번 (접근 보정 없음 = IK+ACT와 같은 접근) → 집기 → 바구니 → HOME
          ③ 끝 사진(위·손목) 저장 → 사람 판정(코드는 compare.py 설명)
기록: var/compare/<name>.jsonl, 사진 var/compare/<name>/
"""
from __future__ import annotations

import argparse
import time

from . import compare as cp


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True, help="기록 이름 → var/compare/<name>.jsonl")
    ap.add_argument("--trials", type=int, default=18)
    from .vision import HSV_RANGES
    colors = sorted(HSV_RANGES)
    ap.add_argument("--color", default="purple", choices=colors,
                    help="집을 블록 색 (시작 값). 시행마다 입력 칸에 색 이름을 쳐서 바꿀 수 있음")
    ap.add_argument("--start", type=int, default=0, help="몇 번째 시행부터 (끊긴 뒤 이어서: 이미 기록된 수)")
    ap.add_argument("--grid", action="store_true", help="평가 격자 g01~g09를 순서대로 안내·확인 (기본: 아무 자리)")
    ap.add_argument("--place-tol", type=float, default=15.0, help="--grid일 때 격자점 거리 허용 [mm]")
    ap.add_argument("--step", action="store_true", help="pick.py 단계마다 Enter")
    ap.add_argument("--max-outside-mm", type=float, help="보정점 범위 밖 이 거리까지만 집는다 [mm] (기본 pick.py의 25)")
    ap.add_argument("--any-place", action="store_true", help="범위 제한 없이 테이블 어디든")
    ap.add_argument("--queue", help='큐: "orange 2, yellow 1, blue" = 주황 최대 2개 → 노랑 최대 1개 → 파랑은 안 보일 때까지. '
                                    "그 색이 안 보이면 다음 명령으로. 묻지 않고 이어서 실행(사람 판정 없음)")
    args = ap.parse_args(argv)

    from .calib import PixelToRobot
    from .camera import correct_wb, open_role
    from .pick import CALIB
    from .pick import main as pick_once

    calib = PixelToRobot.load(CALIB)
    log_path = cp.OUT_DIR / f"{args.name}.jsonl"
    photo_dir = cp.OUT_DIR / args.name
    plan = cp.schedule(args.trials)
    demos = cp.demo_positions()
    print(f"순수 IK 비교 실험 {args.trials}회 → {log_path}")
    cp.print_rules()
    color = args.color
    t = args.start
    queue = cp.PickQueue(cp.parse_queue(args.queue, colors)) if args.queue else None
    while queue is not None or t < args.trials:
        if queue is not None:
            color = queue.current()
            if color is None:
                break
            print(f"\n[{queue.label()}]")
        grid = plan[t] if (args.grid and queue is None) else None
        where = grid or "작업 영역 아무 곳"
        ans = "" if queue is not None else input(f"\n[시행 {t + 1}/{args.trials}] 블록({color})을 {where}에 놓고 Enter "
                    f"(색 바꾸기: 색 이름 입력 [{', '.join(colors)}] / q = 끝) ").strip().lower()
        if ans == "q":
            break
        if ans:
            if ans not in colors:
                print(f"  모르는 색: {ans}. 가능: {', '.join(colors)}")
                continue
            color = ans
            print(f"  이번부터 {color} 블록을 집습니다")
        place = {}
        if grid:
            cap, gains = open_role("top")
            try:
                place = cp.precheck_place(cap, gains, color, calib, grid, args.place_tol)
            finally:
                cap.release()
        t0 = time.monotonic()
        try:
            extra = (["--step"] if args.step else []) + (["--any-place"] if args.any_place else []) + \
                    (["--max-outside-mm", str(args.max_outside_mm)] if args.max_outside_mm is not None else [])
            pl = pick_once(["--color", color, "--no-correction"] + extra)
        except Exception as e:  # 연결 오류 등. 기록은 남기고 계속할지 묻는다
            pl = {"result": "error", "error": repr(e)}
            print("  오류:", e)
        dur = time.monotonic() - t0
        if queue is not None:
            outcome = {"placed": "ok", "no_block": "none", "out_of_range": "none"}.get(pl.get("result"), "fail")
            if pl.get("result") == "out_of_range":
                print("  ", pl.get("note", ""))
            queue.report(outcome)
            if outcome == "none":
                continue                                   # 블록이 없어 아무것도 안 함 → 기록하지 않음
        # 끝 사진 (팔은 HOME이라 위 카메라가 바구니·작업 영역을 본다)
        frames = {}
        for role in ("top", "wrist"):
            try:
                c, g = open_role(role)
                ok, f = False, None
                for _ in range(5):
                    ok, f = c.read()
                c.release()
                frames[role] = correct_wb(f, g) if ok else None
            except Exception as e:
                print(f"  {role} 사진 실패:", e)
        photos = cp.save_photos(photo_dir, f"t{t + 1:02d}" + (f"_{grid}" if grid else ""), frames)
        print(f"  자동 기록: {pl.get('result')}" + (f", 그리퍼 {pl.get('gripper_after_lift')}" if "gripper_after_lift" in pl else ""))
        v = dict(cp.QUEUE_VERDICT) if queue is not None else cp.ask_verdict()
        rec = {"method": "ik", "name": args.name, "trial": t + 1, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
               "color": color, **place, **v, "auto": pl.get("result"), "held": pl.get("held"),
               "home_ok": pl.get("result") in ("placed", "empty"), "duration_s": round(dur, 1),
               "block_camera": pl.get("block_camera"), "photos": photos, "pick": pl}
        if pl.get("block_camera"):
            x, y = pl["block_camera"][:2]
            rec.update(cp.place_record(calib, x, y, demos))
            if grid:
                rec["grid_offset_at_run_mm"] = round(cp.grid_offset(x, y, grid), 1)
            print(f"  블록 자리 x={x:.0f} y={y:.0f}, 가장 가까운 시연 {rec['nearest_demo_mm']}mm")
        cp.append(log_path, rec)
        print(f"  기록: {cp.CODES[v['verdict']]}  ({t + 1}/{args.trials})")
        t += 1
    if queue is not None:
        print("\n" + queue.summary())
    if log_path.exists():
        print("\n" + cp.report([str(log_path)]))


if __name__ == "__main__":
    main()
