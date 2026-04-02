"""Configuration for the UR medicine LeRobot robot plugin."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable


try:
    from lerobot.robots import RobotConfig as LeRobotRobotConfig  # type: ignore
except ImportError:
    @dataclass
    class LeRobotRobotConfig:
        """Fallback RobotConfig used when lerobot is not installed."""

        type: str = "ur_medicine"

        @classmethod
        def register_subclass(cls, _name: str) -> Callable[[type], type]:
            def decorator(subclass: type) -> type:
                return subclass

            return decorator


@LeRobotRobotConfig.register_subclass("ur_medicine")
@dataclass
class URMedicineRobotConfig(LeRobotRobotConfig):
    """Configuration for the UR robot + gripper + L515 setup."""

    host: str = "192.168.56.2"
    command_port: int = 30002
    state_port: int = 30003
    robot_backend: str = "auto"
    rtde_frequency: float = 500.0
    gripper_port: str = "/dev/ttyUSB0"
    gripper_baudrate: int = 115200
    gripper_slave_id: int = 1
    gripper_timeout: float = 0.2
    legacy_root: str = "/home/nav/JuShen_new"
    pose_file: str = str(Path(__file__).resolve().parents[3] / "config" / "ur_pos.yaml")
    top_camera_serial: str | None = None
    top_camera_width: int = 640
    top_camera_height: int = 480
    top_camera_fps: int = 30
    use_mock_camera: bool = False
    use_speed_control: bool = True
    control_period_s: float = 0.02
    max_linear_step_m: float = 0.01
    max_angular_step_rad: float = 0.10
    max_linear_speed_m_s: float = 0.10
    max_angular_speed_rad_s: float = 0.50
    speed_acceleration_m_s2: float = 0.25
    speed_rot_acceleration_rad_s2: float = 0.50
    stop_acceleration_m_s2: float = 1.00
    stop_rot_acceleration_rad_s2: float = 1.50
    wait_for_motion: bool = False
    workspace_min: list[float] = field(default_factory=lambda: [-0.80, -0.80, 0.10])
    workspace_max: list[float] = field(default_factory=lambda: [0.80, 0.80, 1.20])
    home_pose_name: str = "home"
