"""실시간 블록 검출 화면.

실행 (.venv 또는 conda lerobot, project/src에서):
    python -m kitbot_exec.view_blocks --role top --color red      # 고정 설정·색 보정 적용 (추천)
    python -m kitbot_exec.view_blocks --cam /dev/video0 --color red  # 장치 번호로 직접 (설정 적용 안 함)
    python -m kitbot_exec.view_blocks --image 저장한사진.png --color red     # 저장한 사진으로

키: q = 끝내기, s = 지금 사진 저장(var/frames/), m = 마스크 창 켜기/끄기
화면 왼쪽 위의 "jitter"는 가장 큰 블록 중심이 최근 30장 동안 흔들린 정도(표준편차, 픽셀)다.
S02의 "같은 블록 중심의 반복 편차" 확인에 쓴다. --calib을 주면 로봇 좌표(mm)도 표시한다.
"""
from __future__ import annotations

import argparse
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from .vision import BlockFilter, HSV_RANGES, clean_mask, color_mask, blocks_in_mask, draw_blocks, put_label

FRAMES_DIR = Path(__file__).resolve().parents[1] / "var" / "frames"

# S02 5단계 평가 위치 (로봇 좌표 mm). 로봇 좌표: 원점 = 로봇 받침대 회전축, x = 어깨 회전 0°일 때 팔이 뻗는 방향,
# y = 위에서 볼 때 x를 반시계로 90° 돌린 방향. 화면에서 어느 쪽인지는 --grid 화살표로 보인다.
S02_GRID = {f"g{i + 1:02d}": (x, y) for i, (y, x) in enumerate((y, x) for y in (100, 160, 220) for x in (95, 120, 145))}


def robot_to_px(calib, xy):
    """로봇 좌표 (블록 윗면 높이) → 화면 픽셀 (보정 변환의 역)."""
    pts = np.asarray(xy, float).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, np.linalg.inv(calib.H)).reshape(-1, 2)


def draw_grid(view, calib, block_xy=None):
    """9개 평가 위치(초록 원)와 로봇 x·y 방향 화살표. 블록이 있으면 가장 가까운 위치까지 남은 거리를 돌려준다."""
    names = list(S02_GRID)
    px = robot_to_px(calib, [S02_GRID[n] for n in names])
    for n, (u, v) in zip(names, px):
        cv2.circle(view, (int(u), int(v)), 9, (0, 255, 0), 2)
        cv2.putText(view, n, (int(u) + 10, int(v) + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 120, 0), 2, cv2.LINE_AA)
    o, ax, ay = robot_to_px(calib, [(120, 160), (160, 160), (120, 200)])
    cv2.arrowedLine(view, tuple(map(int, o)), tuple(map(int, ax)), (0, 0, 255), 2, tipLength=0.25)
    cv2.arrowedLine(view, tuple(map(int, o)), tuple(map(int, ay)), (255, 0, 0), 2, tipLength=0.25)
    cv2.putText(view, "+x", tuple(map(int, ax + 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(view, "+y", tuple(map(int, ay + 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)
    put_label(view, "close: click this window + q, or X button", (10, view.shape[0] - 10), 0.5, (0, 255, 255))
    if block_xy is None:
        return None
    d = {n: np.subtract(S02_GRID[n], block_xy) for n in names}
    n = min(d, key=lambda k: np.linalg.norm(d[k]))
    return n, d[n]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["top", "wrist"], help="camera_settings.json의 카메라 (고정 설정·색 보정 적용)")
    ap.add_argument("--cam", default="/dev/video0")
    ap.add_argument("--image", help="카메라 대신 저장한 사진 한 장")
    ap.add_argument("--color", default="red", choices=sorted(HSV_RANGES))
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--min-area", type=float, default=BlockFilter.min_area_px)
    ap.add_argument("--max-area", type=float, default=BlockFilter.max_area_px)
    ap.add_argument("--calib", help="calib.PixelToRobot.save()로 저장한 JSON (있으면 mm 표시)")
    ap.add_argument("--grid", action="store_true", help="S02 평가 위치 9곳(g01~g09)과 로봇 x·y 방향을 화면에 표시 (--calib 필요)")
    args = ap.parse_args()

    flt = BlockFilter(min_area_px=args.min_area, max_area_px=args.max_area)
    calib = None
    if args.calib:
        from .calib import PixelToRobot
        calib = PixelToRobot.load(args.calib)
    if args.grid and calib is None:
        raise SystemExit("--grid에는 --calib이 필요합니다 (예: --calib kitbot_exec/calib_top.json)")

    cap = None
    still = None
    gains = None
    if args.image:
        still = cv2.imread(args.image)
        if still is None:
            raise SystemExit(f"사진을 읽을 수 없습니다: {args.image}")
    elif args.role:
        from .camera import open_role
        cap, gains = open_role(args.role, args.width, args.height)
    else:
        from .camera import open_webcam
        cap = open_webcam(args.cam, args.width, args.height)

    hist: deque = deque(maxlen=30)
    show_mask = False
    t_prev = time.time()
    fps = 0.0
    try:
        while True:
            if still is not None:
                frame = still.copy()
            else:
                ok, frame = cap.read()
                if not ok:
                    continue
                if gains:
                    from .camera import correct_wb
                    frame = correct_wb(frame, gains)
            mask = clean_mask(color_mask(frame, args.color))
            blocks = blocks_in_mask(mask, flt)
            view = draw_blocks(frame, blocks)

            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
            t_prev = now
            lines = [f"{args.color}: {len(blocks)} block(s)  {fps:.0f} fps"]
            if blocks:
                b = blocks[0]
                hist.append(b.center_px)
                if len(hist) >= 5:
                    sd = np.std(np.asarray(hist), axis=0)
                    lines.append(f"jitter u {sd[0]:.2f}px v {sd[1]:.2f}px  area {b.area_px:.0f}px")
                if calib is not None:
                    x, y, z, yaw = calib.block_pose(b)
                    lines.append(f"robot x {x:.1f} y {y:.1f} z {z:.1f} mm  yaw {yaw:+.1f}deg")
                    if args.grid:
                        n, d = draw_grid(view, calib, (x, y))
                        ok = "OK" if np.linalg.norm(d) <= 10 else "move"
                        lines.append(f"nearest {n}: need x {d[0]:+.0f} y {d[1]:+.0f} mm  [{ok}]")
            else:
                hist.clear()
                if args.grid:
                    draw_grid(view, calib)
            for i, t in enumerate(lines):
                put_label(view, t, (10, 24 + 26 * i), 0.6)

            cv2.imshow("blocks", view)
            if show_mask:
                cv2.imshow("mask", mask)
            key = cv2.waitKey(30 if still is not None else 1) & 0xFF
            if key == ord("q"):
                break
            try:  # 창의 X 버튼으로 닫아도 끝낸다
                if cv2.getWindowProperty("blocks", cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
            if key == ord("m"):
                show_mask = not show_mask
                if not show_mask:
                    cv2.destroyWindow("mask")
            if key == ord("s"):
                FRAMES_DIR.mkdir(parents=True, exist_ok=True)
                p = FRAMES_DIR / time.strftime("%Y%m%d-%H%M%S.png")
                cv2.imwrite(str(p), frame)
                print("저장:", p)
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
