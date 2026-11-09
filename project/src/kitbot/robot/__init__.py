from .base import ActionResult, GraspCheck, PickResult, RobotDriver
from .mock import MockDriver


def build_driver(settings) -> RobotDriver:
    if settings.driver == "mock":
        return MockDriver(success_rate=settings.mock_success_rate, pick_s=settings.mock_pick_s)
    raise ValueError(f"unknown driver: {settings.driver}")


__all__ = ["ActionResult", "GraspCheck", "PickResult", "RobotDriver", "MockDriver", "build_driver"]
