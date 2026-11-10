"""USB 웹캠 열기 (Linux V4L2).

카메라 번호: 웹캠 한 대가 /dev/video 번호를 2개 만든다. 영상은 짝수 번호(예: /dev/video0, /dev/video2)에서 나온다.
노출·화이트밸런스 고정은 camera_lock.py(v4l2-ctl)로 한다. lock_exposure()는 OpenCV만으로 하는 예비 방법이다.
open_role("top")은 camera_settings.json의 USB 포트 경로로 카메라를 찾고, 고정 값을 적용한 뒤 연다.
"""
from __future__ import annotations

import cv2


def open_webcam(device: str | int = "/dev/video2", width: int = 640, height: int = 480, fps: int = 30,
                mjpg: bool = True) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"카메라 {device}를 열 수 없습니다. `ls /dev/video*`로 번호를 확인하세요")
    if mjpg:  # 1280×720에서 30fps를 내려면 압축 형식이 필요하다
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def lock_exposure(cap: cv2.VideoCapture, exposure: float | None = None, wb_temperature: float | None = None) -> dict:
    """자동 노출·자동 화이트밸런스를 끈다. 값을 주지 않으면 지금 값으로 고정한다.
    V4L2에서 CAP_PROP_AUTO_EXPOSURE: 1 = 수동, 3 = 자동. 카메라가 지원하지 않으면 set()이 False를 돌려준다."""
    result = {
        "auto_exposure_off": cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1),
        "auto_wb_off": cap.set(cv2.CAP_PROP_AUTO_WB, 0),
    }
    if exposure is not None:
        result["exposure_set"] = cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    if wb_temperature is not None:
        result["wb_set"] = cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb_temperature)
    result["exposure"] = cap.get(cv2.CAP_PROP_EXPOSURE)
    result["wb_temperature"] = cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
    return result


def correct_wb(bgr, gains):
    """camera_settings.json의 software_wb_gains(B, G, R 배율)로 색 치우침을 보정한다."""
    import numpy as np

    if not gains:
        return bgr
    out = bgr.astype(np.float32) * np.asarray(gains, np.float32)
    return np.clip(out, 0, 255).astype(np.uint8)


def open_role(role: str, width: int = 640, height: int = 480, fps: int = 30):
    """'top' / 'wrist' → (VideoCapture, software_wb_gains). 고정 값을 적용한 뒤 연다."""
    import json

    from .camera_lock import SETTINGS, resolve, set_ctrls

    cam = json.loads(SETTINGS.read_text())["cameras"][role]
    dev = resolve(cam["by_path"])
    cap = open_webcam(dev, width, height, fps)
    if cam.get("controls"):
        set_ctrls(dev, cam["controls"])  # 여는 과정에서 바뀌었을 수 있어 연 뒤에 다시 적용
    return cap, cam.get("software_wb_gains")
