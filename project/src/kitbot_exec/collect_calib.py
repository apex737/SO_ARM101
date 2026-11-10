"""화면 → 로봇 보정점 수집 (S02-5, conda lerobot).

    cd project/src && python -m kitbot_exec.collect_calib --color red

한 점마다 (블록 하나만 화면에 두고):
  1. 블록을 작업 영역 한 곳에 놓고, 팔을 비켜 둔 채 카메라 창에서 d  → 블록 중심 픽셀 기록(최근 10장 평균)
  2. 리더암으로 팔로워 그리퍼를 수직 아래로 향하게 하고, 두 부리 끝 가운데를 블록 윗면 가운데에 살짝 댄 뒤 r
     → 그때 관절값으로 계산한 그리퍼 끝 위치(x, y, z) 기록 → 한 쌍 완성
  3. 블록을 다른 곳으로 옮겨 반복. 작업 영역 전체에 고르게 9곳 이상.
  u = 마지막 쌍 지우기, f = 변환 계산·저장(4쌍 이상), q = 끝내기
(f를 안 눌러도 q나 터미널 Ctrl+C로 끝낼 때 자동으로 계산·저장한다)
쌍은 var/calib_pairs.json에 계속 저장되어 끊겨도 이어서 할 수 있다(--fresh로 새로 시작).
f를 누르면 '한 점씩 빼고 계산한 오차'(보정에 안 쓴 점 오차와 비슷)를 보여 준다.
키는 카메라 창을 한 번 클릭한 뒤, 영문 입력 상태에서 누른다(한글 입력이면 f가 'ㄹ'로 들어가 무시됨).
팔·카메라 없이 저장된 쌍으로 f만 다시 하려면: python -m kitbot_exec.collect_calib --fit-only
"""
from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from .arm import Arm, Leader
from .calib import fit_pixel_to_robot
from .camera import correct_wb, open_role
from .kinematics import SO101Kinematics
from .vision import HSV_RANGES, detect_blocks, draw_blocks

VAR = Path(__file__).resolve().parents[1] / "var"
PAIRS = VAR / "calib_pairs.json"
CALIB = Path(__file__).with_name("calib_top.json")


def leave_one_out_mm(px, mm) -> np.ndarray:
    errs = []
    for i in range(len(px)):
        keep = [k for k in range(len(px)) if k != i]
        cal = fit_pixel_to_robot(px[keep], mm[keep])
        errs.append(float(cal.residuals_mm(px[i:i + 1], mm[i:i + 1])[0]))
    return np.array(errs)


def fit_and_save(pairs) -> str:
    """저장된 쌍으로 변환을 계산해 calib_top.json에 저장하고, 결과 한 줄을 돌려준다."""
    if len(pairs) < 4:
        return "4쌍 이상 필요"
    px = np.array([p["px"] for p in pairs], float)
    mm = np.array([p["mm"] for p in pairs], float)
    cal = fit_pixel_to_robot(px, mm)
    cal.save(CALIB)
    loo = leave_one_out_mm(px, mm) if len(pairs) >= 5 else np.array([np.nan])
    print("점별 오차(mm):", np.round(loo, 1).tolist())
    print("닿은 높이 z(mm):", np.round(mm[:, 2], 0).tolist(), "(블록 윗면이 모두 같은 높이라 비슷해야 함)")
    print("블록 윗면 높이 평면 z = a·x + b·y + c:", np.round(cal.z_plane, 4).tolist())
    return f"저장 {CALIB.name}: 한 점 빼고 오차 평균 {np.nanmean(loo):.1f} 최대 {np.nanmax(loo):.1f} mm"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--color", default="red", choices=sorted(HSV_RANGES))
    ap.add_argument("--fresh", action="store_true")
    ap.add_argument("--fit-only", action="store_true", help="팔·카메라 없이 저장된 쌍으로 f만 실행")
    args = ap.parse_args()

    VAR.mkdir(exist_ok=True)
    pairs = [] if args.fresh or not PAIRS.exists() else json.loads(PAIRS.read_text())
    if args.fit_only:
        print(fit_and_save(pairs))
        return
    kin = SO101Kinematics()
    arm, leader = Arm(), Leader()
    cap, gains = open_role("top")
    recent: deque = deque(maxlen=10)
    pending = None
    changed = False  # 이번 실행에서 쌍을 추가·삭제했으면 끝낼 때 자동으로 f를 한다
    msg = "d: 블록 픽셀 기록"
    shown = None
    try:
        while True:
            arm.send_action(leader.action())
            ok, frame = cap.read()
            if not ok:
                continue
            frame = correct_wb(frame, gains)
            blocks = detect_blocks(frame, args.color)
            if len(blocks) == 1:
                recent.append(blocks[0].center_px)
            else:
                recent.clear()
            view = draw_blocks(frame, blocks)
            for p in pairs:
                cv2.circle(view, tuple(int(v) for v in p["px"]), 5, (0, 255, 0), -1)
            q, _ = arm.read()
            tip, R = kin.fk(q)
            tilt = np.rad2deg(np.arccos(np.clip(-R[2, 2], -1, 1)))
            if msg != shown:  # 카메라 창은 한글을 못 그려서 안내는 터미널에
                print(msg)
                shown = msg
            hint = "r: record tip" if pending else "d: detect block"
            lines = [f"pairs {len(pairs)}  blocks {len(blocks)}  {hint}  (u undo, f fit, q quit)",
                     f"tip x {tip[0]:.0f} y {tip[1]:.0f} z {tip[2]:.0f} mm  tilt {tilt:.0f}deg"]
            for i, t in enumerate(lines):
                cv2.putText(view, t, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.imshow("collect_calib", view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("d"):
                if len(recent) >= 5:
                    pending = np.mean(np.asarray(recent), axis=0).tolist()
                    msg = f"픽셀 {np.round(pending, 1).tolist()} 기록 -> 그리퍼를 블록 윗면에 대고 r"
                else:
                    msg = "블록이 정확히 1개 보여야 함 (팔을 비키고 다시 d)"
            if key == ord("r"):
                if pending is None:
                    msg = "먼저 d"
                else:
                    pairs.append({"px": pending, "mm": [round(float(v), 2) for v in tip],
                                  "tilt_deg": round(float(tilt), 1), "q_deg": np.round(q, 2).tolist()})
                    PAIRS.write_text(json.dumps(pairs, indent=1))
                    pending = None
                    changed = True
                    warn = "  (기울기 10° 넘음: 수직에 가깝게 다시 하는 게 좋음 -> u)" if tilt > 10 else ""
                    if len(pairs) >= 2:
                        # 블록 윗면은 모두 같은 높이라 닿은 높이도 비슷해야 한다
                        z_ref = float(np.median([p["mm"][2] for p in pairs[:-1]]))
                        if abs(tip[2] - z_ref) > 8:
                            warn += f"  (높이가 다른 점과 {tip[2] - z_ref:+.0f}mm 다름: 블록 윗면에 실제로 닿았나? -> u)"
                    msg = f"{len(pairs)}번째 쌍 저장 (높이 z {tip[2]:.0f}mm){warn}. 블록을 옮기고 d"
            if key == ord("u") and pairs:
                pairs.pop()
                PAIRS.write_text(json.dumps(pairs, indent=1))
                changed = True
                msg = f"마지막 쌍 삭제, 남은 {len(pairs)}"
            if key == ord("f"):
                msg = fit_and_save(pairs)
                changed = False
    finally:
        if changed and len(pairs) >= 4:
            # f 키가 창에 안 들어가는 경우가 있어(터미널에 초점), 끝낼 때(q 또는 Ctrl+C) 자동 계산
            print("끝내기 전에 자동 계산:")
            print(fit_and_save(pairs))
        cap.release()
        cv2.destroyAllWindows()
        leader.disconnect()
        arm.disconnect()


if __name__ == "__main__":
    main()
