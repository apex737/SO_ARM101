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

from .vision import BlockFilter, HSV_RANGES, clean_mask, color_mask, blocks_in_mask, draw_blocks

FRAMES_DIR = Path(__file__).resolve().parents[1] / "var" / "frames"


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
    args = ap.parse_args()

    flt = BlockFilter(min_area_px=args.min_area, max_area_px=args.max_area)
    calib = None
    if args.calib:
        from .calib import PixelToRobot
        calib = PixelToRobot.load(args.calib)

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
            else:
                hist.clear()
            for i, t in enumerate(lines):
                cv2.putText(view, t, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            cv2.imshow("blocks", view)
            if show_mask:
                cv2.imshow("mask", mask)
            key = cv2.waitKey(30 if still is not None else 1) & 0xFF
            if key == ord("q"):
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
