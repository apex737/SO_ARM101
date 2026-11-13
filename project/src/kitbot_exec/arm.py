"""SO-101 팔로워·리더 연결과 안전한 이동 (conda lerobot 환경).

- 관절 각도 단위는 LeRobot 기준 '도'(use_degrees=True). 0° = 캘리브레이션 때 기록한 가동 범위의 가운데.
  LeRobot도 같은 설계 파일(so101_new_calib.urdf)을 이 각도에 그대로 연결해 쓴다.
- 그리퍼는 0~100 (LeRobot 값). 어느 쪽이 닫힘인지는 teach.py로 확인한다.
- 이동은 모두 보간한다: 관절 이동은 가장 많이 움직이는 관절이 speed_dps(도/초)를 넘지 않게,
  직선 이동은 경로 전체를 역기구학으로 먼저 계산해 하나라도 실패하면 움직이기 전에 멈춘다.
- 안전망: LeRobot max_relative_target(한 번 명령에 관절이 움직일 수 있는 최대 각도).
- 프로그램이 끝나도 토크를 유지한다(hold_on_exit). 팔이 테이블로 떨어지지 않게 하기 위함이다.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from .kinematics import ARM_JOINTS

FOLLOWER_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14030764-if00"
LEADER_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14114649-if00"
FOLLOWER_ID = "my_follower_arm"
LEADER_ID = "my_leader_arm"
POSES = Path(__file__).with_name("poses.json")
HZ = 30


class Arm:
    def __init__(self, port: str = FOLLOWER_PORT, robot_id: str = FOLLOWER_ID, hold_on_exit: bool = True,
                 max_step_deg: float = 8.0):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        cfg = SO101FollowerConfig(port=port, id=robot_id, use_degrees=True,
                                  disable_torque_on_disconnect=not hold_on_exit,
                                  max_relative_target=max_step_deg)
        self.robot = SO101Follower(cfg)
        try:
            self.robot.connect()
        except RuntimeError as e:
            if "Missing motor IDs" not in str(e):
                raise
            # 그리퍼가 눌린 채 오래 힘을 쓰면 서보 과부하 보호가 켜져 응답을 멈춘다.
            # 그 모터의 힘을 한 번 끄면 풀린다(2026-11-13 실물 확인). 다른 관절은 힘을 그대로 유지한다.
            cleared = clear_overload(port)
            if cleared:
                print(f"  모터 {cleared}번 과부하 보호를 풀었습니다 (힘을 한 번 끔). 다시 연결합니다.")
                self.robot = SO101Follower(cfg)
                self.robot.connect()
            else:
                raise RuntimeError(
                    "팔로워 모터 일부가 응답하지 않음 (보통 6번 = 그리퍼). 과부하 풀기도 실패.\n"
                    "  → 팔로워 전원 어댑터를 뽑고 10초 뒤 다시 꽂은 다음 실행. 계속되면 그 모터 케이블 확인.\n"
                    f"  (원래 오류: {str(e).splitlines()[0]})") from e

    # ---------- 읽기·쓰기 ----------
    def read(self) -> tuple[np.ndarray, float]:
        obs = self.robot.get_observation()
        return np.array([obs[f"{j}.pos"] for j in ARM_JOINTS], float), float(obs["gripper.pos"])

    def send(self, q_deg, gripper: float) -> None:
        act = {f"{j}.pos": float(v) for j, v in zip(ARM_JOINTS, q_deg)}
        act["gripper.pos"] = float(gripper)
        self.robot.send_action(act)

    def send_action(self, action: dict) -> dict:
        """LeRobot이 실제로 보낸 값을 돌려준다(한 번에 max_step_deg 넘게 움직이는 명령은 잘린다)."""
        return self.robot.send_action(action)

    # ---------- 이동 ----------
    def move_joints(self, q_target, gripper: float | None = None, speed_dps: float = 40.0,
                    settle_tol_deg: float = 2.0, settle_timeout_s: float = 2.0) -> bool:
        """관절 공간에서 부드럽게(코사인 가감속) 이동. 멈춤 기준 안에 들어왔으면 True, 시간 초과면 False."""
        q0, g0 = self.read()
        q1 = np.asarray(q_target, float)
        g1 = g0 if gripper is None else float(gripper)
        dur = max(np.max(np.abs(q1 - q0)) / speed_dps, abs(g1 - g0) / 100.0, 0.3)
        self._stream([q0 + (q1 - q0) * s for s in _profile(dur)], [g0 + (g1 - g0) * s for s in _profile(dur)])
        return self._settle(q1, g1, settle_tol_deg, settle_timeout_s)

    def follow(self, q_path, gripper: float) -> bool:
        """미리 계산한 관절 경로(직선 이동 등)를 HZ로 따라간다. 반환값은 move_joints와 같다."""
        self._stream(list(q_path), [gripper] * len(q_path))
        return self._settle(np.asarray(q_path[-1]), gripper, 2.0, 2.0)

    def set_gripper(self, value: float, wait_s: float = 1.0) -> float:
        q, _ = self.read()
        t = time.time()
        while time.time() - t < wait_s:
            self.send(q, value)
            time.sleep(1.0 / HZ)
        return self.read()[1]

    def disconnect(self) -> None:
        self.robot.disconnect()

    # ---------- 내부 ----------
    def _stream(self, qs, gs) -> None:
        period = 1.0 / HZ
        nxt = time.perf_counter()
        for q, g in zip(qs, gs):
            self.send(q, g)
            nxt += period
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _settle(self, q1, g1, tol, timeout) -> bool:
        """목표 관절값과의 차이가 tol 안이 될 때까지 기다린다. 동작은 예전과 같고, 결과만 돌려준다
        (예전에는 시간 초과여도 아무 표시 없이 끝났다)."""
        t = time.time()
        while time.time() - t < timeout:
            self.send(q1, g1)
            q, _ = self.read()
            if np.max(np.abs(q - q1)) < tol:
                return True
            time.sleep(1.0 / HZ)
        return False


def _profile(duration_s: float) -> np.ndarray:
    n = max(int(duration_s * HZ), 2)
    t = np.linspace(0.0, 1.0, n)
    return 0.5 - 0.5 * np.cos(np.pi * t)


def clear_overload(port: str) -> list[int]:
    """응답이 없는 모터(1~6번)에 힘 끄기를 보내 과부하 보호를 푼다. 다시 응답하게 된 모터 번호 목록."""
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    bus = FeetechMotorsBus(port, {f"m{i}": Motor(i, "sts3215", MotorNormMode.DEGREES) for i in range(1, 7)})
    try:
        bus.connect(handshake=False)
        before = set(bus.broadcast_ping() or {})
        missing = [i for i in range(1, 7) if i not in before]
        for i in missing:
            try:
                bus.write("Torque_Enable", f"m{i}", 0, normalize=False, num_retry=2)
            except Exception:      # 과부하 상태에서는 오류 응답이 오지만 명령은 처리된다
                pass
        time.sleep(0.5)
        after = set(bus.broadcast_ping() or {})
        return [i for i in missing if i in after]
    except Exception:
        return []
    finally:
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass


class Leader:
    def __init__(self, port: str = LEADER_PORT, leader_id: str = LEADER_ID):
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

        self.teleop = SO101Leader(SO101LeaderConfig(port=port, id=leader_id, use_degrees=True))
        self.teleop.connect()

    def action(self) -> dict:
        return self.teleop.get_action()

    def disconnect(self) -> None:
        self.teleop.disconnect()


def load_poses() -> dict:
    return json.loads(POSES.read_text()) if POSES.exists() else {}


def save_pose(name: str, q_deg, gripper: float) -> None:
    poses = load_poses()
    poses[name] = {"q_deg": [round(float(v), 2) for v in q_deg], "gripper": round(float(gripper), 1),
                   "saved_at": time.strftime("%Y-%m-%d %H:%M")}
    POSES.write_text(json.dumps(poses, ensure_ascii=False, indent=2) + "\n")
