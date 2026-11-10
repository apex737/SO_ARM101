"""웹캠 노출·화이트밸런스 고정 (Linux v4l2-ctl 사용).

UVC 웹캠 설정은 USB를 뽑거나 재부팅하면 초기화된다. 그래서 정한 값을 camera_settings.json에 저장하고
작업을 시작할 때마다 다시 적용한다.

    cd project/src
    python -m kitbot_exec.camera_lock apply            # 저장한 값 적용 (작업 시작할 때마다)
    python -m kitbot_exec.camera_lock tune top         # 위 카메라 값을 새로 찾아 저장 (조명이 바뀌었을 때)
    python -m kitbot_exec.camera_lock show             # 지금 카메라에 들어가 있는 값

값 찾는 기준 (tune)
- 노출: 조명 깜빡임 주기(60Hz 전기 → 8.3ms)의 배수 83·167·250·333 중, 흰 테이블이 하얗게 날아가지 않는
  (포화 픽셀 1% 미만) 가장 긴 값. 배수가 아니면 가로 줄무늬가 생긴다(2026-11-10 노출 40에서 확인).
- 화이트밸런스: 밝은 쪽 절반 픽셀(흰 테이블)의 B, G, R 중앙값이 가장 비슷해지는 색온도.
- 그래도 포화가 1%를 넘으면 brightness를 16씩 낮춘다.
- 이 웹캠(Innomaker U20CAM)은 수동 화이트밸런스에서 어떤 색온도로도 초록빛이 남는다(2026-11-10 측정: 6500K에서 흰 테이블
  B 161, G 231, R 169 / 자동은 213, 214, 213). 그래서 색온도는 가장 덜 치우친 값으로 고정하고,
  남는 치우침은 software_wb_gains(B, G, R 배율)로 코드에서 보정한다 → camera.correct_wb().
- 전원 주파수 60Hz(한국), 어두워도 fps를 낮추지 않음.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np

SETTINGS = Path(__file__).with_name("camera_settings.json")
FIXED = {"power_line_frequency": 2, "exposure_dynamic_framerate": 0, "auto_exposure": 1,
         "white_balance_automatic": 0, "gain": 0, "backlight_compensation": 0}


def resolve(by_path: str) -> str:
    """/dev/v4l/by-path/... → /dev/videoN. USB 포트가 같으면 번호가 바뀌어도 같은 카메라를 찾는다."""
    p = Path("/dev/v4l/by-path") / by_path
    if not p.exists():
        raise SystemExit(f"카메라가 없습니다: {p}\n같은 USB 포트에 꽂혀 있는지 확인하세요 (ls -l /dev/v4l/by-path/)")
    return str(p.resolve())


def set_ctrls(dev: str, ctrls: dict) -> None:
    # 자동을 먼저 끄고 나서 수동 값을 넣어야 적용된다 → 순서대로 따로 보낸다
    order = ["auto_exposure", "white_balance_automatic", "exposure_dynamic_framerate"]
    for k in order + [k for k in ctrls if k not in order]:
        if k in ctrls:
            subprocess.run(["v4l2-ctl", "-d", dev, "-c", f"{k}={ctrls[k]}"], check=True)


def get_ctrls(dev: str) -> dict:
    out = subprocess.run(["v4l2-ctl", "-d", dev, "--list-ctrls"], capture_output=True, text=True, check=True).stdout
    vals = {}
    for line in out.splitlines():
        parts = line.split()
        if parts and "value=" in line:
            vals[parts[0]] = int(line.split("value=")[1].split()[0])
    return vals


def grab(cap, n=5, settle_s=0.6):
    t = time.time()
    while time.time() - t < settle_s:
        cap.read()
    frames = [cap.read()[1] for _ in range(n)]
    return np.mean([f.astype(np.float32) for f in frames if f is not None], axis=0)


def bright_median(img: np.ndarray) -> np.ndarray:
    """밝은 쪽 절반 픽셀(대부분 흰 테이블)의 B, G, R 중앙값. 색이 끼어 있어도 고를 수 있게 채도로 거르지 않는다."""
    px = img.reshape(-1, 3)
    v = px.max(axis=1)
    sel = px[(v >= np.percentile(v, 50)) & (v < 250)]
    return np.median(sel, axis=0) if len(sel) > 500 else np.median(px, axis=0)


def tune(dev: str, width=640, height=480) -> dict:
    from .camera import open_webcam

    # 조명은 60Hz 전기로 1/120초(8.3ms)마다 깜빡인다. 노출이 그 배수가 아니면 화면에 가로 줄무늬가 생긴다.
    exposures = (83, 167, 250, 333)
    set_ctrls(dev, dict(FIXED, white_balance_automatic=1, brightness=0))
    cap = open_webcam(dev, width, height, 30)
    try:
        # 1) 노출: 포화 1% 미만인 가장 긴 노출 (화이트밸런스는 잠시 자동으로 두고)
        best_exp, log = exposures[0], []
        for e in exposures:
            set_ctrls(dev, {"exposure_time_absolute": e})
            img = grab(cap)
            sat = float((img.max(axis=2) >= 250).mean())
            log.append((e, round(float(img.mean()), 1), round(sat * 100, 2)))
            if sat < 0.01:
                best_exp = e
        set_ctrls(dev, {"exposure_time_absolute": best_exp})
        # 2) 화이트밸런스: 밝은 픽셀(흰 테이블)의 B, G, R이 같아지는 색온도
        set_ctrls(dev, {"white_balance_automatic": 0})
        best_wb, best_err = 4600, 1e9
        for wb in range(2800, 6501, 100):
            set_ctrls(dev, {"white_balance_temperature": wb})
            b, g, r = bright_median(grab(cap, 3, 0.3))
            err = float(max(b, g, r) - min(b, g, r))
            if err < best_err:
                best_wb, best_err = wb, err
        set_ctrls(dev, {"white_balance_temperature": best_wb})
        # 3) 그래도 포화가 1% 넘으면 밝기(brightness)를 낮춘다
        brightness = 0
        while brightness > -64:
            img = grab(cap, 3, 0.4)
            if float((img.max(axis=2) >= 250).mean()) < 0.01:
                break
            brightness -= 16
            set_ctrls(dev, {"brightness": brightness})
        final = grab(cap, 5, 0.4)
    finally:
        cap.release()
    ctrls = dict(FIXED, exposure_time_absolute=best_exp, white_balance_temperature=best_wb, brightness=brightness)
    set_ctrls(dev, ctrls)
    # 이 웹캠은 수동 화이트밸런스에서 초록이 늘 강하다(색온도는 파랑↔빨강만 조절). 남는 치우침은 코드에서
    # 채널 배율로 보정한다: 흰 테이블의 B, G, R이 같아지도록 B·R에 곱할 값.
    b_, g_, r_ = bright_median(final)
    gains = [round(float(g_ / b_), 3), 1.0, round(float(g_ / r_), 3)]
    return {"controls": ctrls, "software_wb_gains": gains, "exposure_sweep(exposure, mean, saturated%)": log,
            "white_bgr_spread": round(best_err, 1),
            "final_white_bgr": [round(float(x), 1) for x in bright_median(final)],
            "final_saturated%": round(float((final.max(axis=2) >= 250).mean()) * 100, 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["apply", "tune", "show"])
    ap.add_argument("role", nargs="?", help="top 또는 wrist (생략하면 전부)")
    args = ap.parse_args()
    cfg = json.loads(SETTINGS.read_text())
    roles = [args.role] if args.role else list(cfg["cameras"])
    for role in roles:
        cam = cfg["cameras"][role]
        dev = resolve(cam["by_path"])
        if args.cmd == "apply":
            set_ctrls(dev, cam["controls"])
            print(f"{role} ({dev}) 적용:", cam["controls"])
        elif args.cmd == "show":
            print(f"{role} ({dev}):", get_ctrls(dev))
        else:
            res = tune(dev)
            cam["controls"] = res["controls"]
            cam["software_wb_gains"] = res["software_wb_gains"]
            cam["tuned_at"] = time.strftime("%Y-%m-%d %H:%M")
            print(f"{role} ({dev}) 새 값:", json.dumps(res, ensure_ascii=False))
    if args.cmd == "tune":
        SETTINGS.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n")
        print("저장:", SETTINGS)


if __name__ == "__main__":
    main()
