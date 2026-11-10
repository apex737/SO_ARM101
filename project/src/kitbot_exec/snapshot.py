"""잘 맞는 상태를 통째로 저장·복원한다.

    cd project/src
    python -m kitbot_exec.snapshot save 이름      # 지금 상태를 calibration/snapshots/날짜_이름/에 복사
    python -m kitbot_exec.snapshot list           # 저장된 상태 목록
    python -m kitbot_exec.snapshot restore 폴더명  # 그 상태로 되돌림 (지금 파일은 덮어쓰기 전에 var/before_restore/에 백업)

저장하는 것: 화면→로봇 보정(calib_top.json, 보정점), 접근 보정 기록, teach 자세(poses.json),
카메라 고정값(camera_settings.json), LeRobot 팔 캘리브레이션 2개(팔로워·리더).
카메라 장대·바구니·로봇 위치가 그대로일 때만 복원한 값이 맞는다.
"""
from __future__ import annotations

import json
import shutil
import sys
import time
from pathlib import Path

SRC = Path(__file__).resolve().parents[1]
PKG = Path(__file__).resolve().parent
LEROBOT_CAL = Path.home() / ".cache/huggingface/lerobot/calibration"
SNAP_DIR = SRC / "calibration" / "snapshots"

FILES = {  # 스냅샷 안 이름 → 실제 위치
    "calib_top.json": PKG / "calib_top.json",
    "calib_pairs.json": SRC / "var" / "calib_pairs.json",
    "approach_corrections.json": SRC / "var" / "approach_corrections.json",
    "poses.json": PKG / "poses.json",
    "camera_settings.json": PKG / "camera_settings.json",
    "lerobot_follower_my_follower_arm.json": LEROBOT_CAL / "robots/so_follower/my_follower_arm.json",
    "lerobot_leader_my_leader_arm.json": LEROBOT_CAL / "teleoperators/so_leader/my_leader_arm.json",
}


def save(name: str) -> Path:
    dst = SNAP_DIR / f"{time.strftime('%Y-%m-%d_%H%M')}_{name}"
    dst.mkdir(parents=True, exist_ok=False)
    saved = []
    for key, src in FILES.items():
        if src.exists():
            shutil.copy2(src, dst / key)
            saved.append(key)
    (dst / "manifest.json").write_text(json.dumps(
        {"saved_at": time.strftime("%Y-%m-%d %H:%M:%S"), "files": saved,
         "missing": [k for k in FILES if k not in saved]}, ensure_ascii=False, indent=1))
    return dst


def restore(folder: str) -> None:
    src_dir = SNAP_DIR / folder
    if not src_dir.is_dir():
        raise SystemExit(f"없음: {src_dir}")
    backup = SRC / "var" / "before_restore" / time.strftime("%Y-%m-%d_%H%M%S")
    backup.mkdir(parents=True)
    for key, dst in FILES.items():
        s = src_dir / key
        if not s.exists():
            continue
        if dst.exists():
            shutil.copy2(dst, backup / key)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(s, dst)
        print("복원:", key)
    print("복원 전 파일 백업:", backup)


def main() -> None:
    args = sys.argv[1:]
    if args[:1] == ["save"] and len(args) == 2:
        print("저장:", save(args[1]))
    elif args[:1] == ["restore"] and len(args) == 2:
        restore(args[1])
    elif args[:1] == ["list"]:
        for d in sorted(SNAP_DIR.glob("*")) if SNAP_DIR.exists() else []:
            print(d.name)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
