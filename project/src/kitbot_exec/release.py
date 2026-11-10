"""팔로워암의 힘(토크)을 푼다 (conda lerobot).

    cd project/src && python -m kitbot_exec.release

teach·collect_calib·pick은 끝나도 팔이 테이블로 떨어지지 않게 힘을 유지한다(arm.py hold_on_exit).
그래서 프로그램이 끝난 뒤에는 팔이 굳은 채로 있고 손으로 움직일 수 없다. 손으로 움직이려면 이걸 실행한다.
힘이 풀리는 순간 팔이 아래로 떨어지므로 반드시 먼저 손으로 받친다.
"""
from __future__ import annotations

from .arm import Arm


def main() -> None:
    input("팔로워암을 손으로 받치고 Enter (힘이 풀리면 팔이 떨어짐), 취소는 Ctrl+C ")
    arm = Arm()
    try:
        arm.robot.bus.disable_torque()
        print("힘 풀림. 이제 손으로 움직일 수 있습니다.")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
