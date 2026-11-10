"""리더암으로 팔로워암을 움직이며 자세를 저장하고, 기구학 모델이 실제 팔과 맞는지 확인한다 (conda lerobot).

    cd project/src && python -m kitbot_exec.teach

터미널에 글자를 치고 Enter:
  h = 지금 자세를 HOME으로 저장 (블록 작업 영역을 가리지 않는 자세)
  b = 지금 자세를 BASKET으로 저장 (바구니 안쪽 위, 테두리보다 3~5cm 높게. 비스듬해도 됨. 여기서 그리퍼를 열어 떨어뜨림)
  g = 지금 그리퍼 값을 '집기 직전 벌림'으로 저장 (부리 끝에서 15mm 안쪽 간격이 약 45mm일 때)
  c = 지금 그리퍼 값을 '닫힘(빈손)'으로 저장 (그리퍼를 끝까지 닫았을 때)
  p = 지금 그리퍼 끝 위치(모델 계산) 출력: x 앞쪽, y 왼쪽, z 위쪽 [mm], 수직에서 기운 각도, 방향
  q = 끝내기 (팔로워암은 마지막 자세에서 토크를 유지한다)

S02-4 확인: p를 눌러 가며 그리퍼 끝을 움직여 보고, 숫자가 실제 움직임과 같은 방향으로 변하는지 본다.
  - 팔을 앞으로 뻗으면 x 증가, 왼쪽(팔 뒤에서 앞을 볼 때)으로 돌리면 y 증가, 위로 올리면 z 증가
  - 그리퍼가 실제로 수직 아래를 향할 때 '기울기'가 0°에 가까운가
"""
from __future__ import annotations

import threading

import numpy as np

from .arm import HZ, Arm, Leader, load_poses, save_pose
from .kinematics import SO101Kinematics


def main() -> None:
    kin = SO101Kinematics()
    arm = Arm()
    leader = Leader()
    stop = threading.Event()
    lock = threading.Lock()

    def teleop():
        import time
        while not stop.is_set():
            with lock:
                arm.send_action(leader.action())
            time.sleep(1.0 / HZ)

    th = threading.Thread(target=teleop, daemon=True)
    th.start()
    print(__doc__.split("터미널에")[1].split("S02-4")[0])
    print("이미 저장된 자세:", list(load_poses()))
    try:
        while True:
            cmd = input("> ").strip().lower()
            with lock:
                q, g = arm.read()
            if cmd == "q":
                break
            if cmd == "p":
                p, R = kin.fk(q)
                tilt = np.rad2deg(np.arccos(np.clip(-R[2, 2], -1, 1)))
                yaw = np.rad2deg(np.arctan2(R[1, 0], R[0, 0]))
                print(f"그리퍼 끝 x {p[0]:7.1f}  y {p[1]:7.1f}  z {p[2]:7.1f} mm | 수직에서 {tilt:5.1f}° 기움 | "
                      f"방향 {yaw:+6.1f}° | 관절 {np.round(q, 1).tolist()} 그리퍼 {g:.1f}")
            elif cmd == "h":
                save_pose("home", q, g); print("HOME 저장:", np.round(q, 1).tolist())
            elif cmd == "b":
                save_pose("basket", q, g); print("BASKET 저장:", np.round(q, 1).tolist())
            elif cmd == "g":
                save_pose("gripper_pregrasp", q, g); print(f"집기 직전 그리퍼 값 저장: {g:.1f}")
            elif cmd == "c":
                save_pose("gripper_closed", q, g); print(f"닫힘(빈손) 그리퍼 값 저장: {g:.1f}")
            elif cmd:
                print("h / b / g / c / p / q 중 하나")
    finally:
        stop.set()
        th.join(timeout=1)
        leader.disconnect()
        arm.disconnect()


if __name__ == "__main__":
    main()
