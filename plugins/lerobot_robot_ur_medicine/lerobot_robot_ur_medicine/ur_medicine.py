"""Minimal LeRobot-compatible UR medicine robot implementation."""

from __future__ import annotations

import logging
import socket
import struct
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

import yaml

try:
    from lerobot.robots import Robot as LeRobotRobot  # type: ignore
except ImportError:
    class LeRobotRobot:
        """Fallback Robot base used when lerobot is not installed."""

        config_class = None
        name = "robot"

        def __init__(self, config: Any):
            self.config = config


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.gripper.gripper_adapter import Gripper
from src.robot.ur_adapter import URRobotAdapter

from .config_ur_medicine import URMedicineRobotConfig

LOGGER = logging.getLogger(__name__)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _normalize_pose(values: Sequence[float], field_name: str) -> list[float]:
    pose = [float(value) for value in values]
    if len(pose) != 6:
        raise ValueError(f"{field_name} must contain 6 values, got {len(pose)}")
    return pose


def _read_realtime_joints(host: str, port: int, timeout: float) -> Optional[list[float]]:
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        data = sock.recv(1108)
        if len(data) < 300:
            return None
        return list(struct.unpack("!6d", data[252:300]))
    except Exception:
        return None
    finally:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass


class L515FrameSource:
    """Small RealSense L515 reader with optional mock fallback."""

    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        depth_width: int | None = None,
        depth_height: int | None = None,
        depth_fps: int | None = None,
        serial: str | None = None,
        use_mock_camera: bool = False,
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.depth_width = int(depth_width if depth_width is not None else width)
        self.depth_height = int(depth_height if depth_height is not None else height)
        self.depth_fps = int(depth_fps if depth_fps is not None else fps)
        self.serial = serial
        self.use_mock_camera = bool(use_mock_camera)
        self._pipeline = None
        self._align = None
        self._rs = None
        self._started = False
        self._active_color_profile = (self.width, self.height, self.fps)
        self._active_depth_profile = (self.depth_width, self.depth_height, self.depth_fps)

    def connect(self) -> bool:
        if self.use_mock_camera:
            LOGGER.warning("Using mock top camera frames.")
            self._started = False
            return True

        try:
            import pyrealsense2 as rs  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "pyrealsense2 is required for L515 camera access. "
                "Use use_mock_camera=True for dry runs."
            ) from exc

        profile_candidates = []
        seen_profiles: set[tuple[int, int, int, int, int, int]] = set()

        def add_candidate(
            color_width: int,
            color_height: int,
            color_fps: int,
            depth_width: int,
            depth_height: int,
            depth_fps: int,
        ) -> None:
            candidate = (
                int(color_width),
                int(color_height),
                int(color_fps),
                int(depth_width),
                int(depth_height),
                int(depth_fps),
            )
            if candidate not in seen_profiles:
                profile_candidates.append(candidate)
                seen_profiles.add(candidate)

        add_candidate(
            self.width,
            self.height,
            self.fps,
            self.depth_width,
            self.depth_height,
            self.depth_fps,
        )
        # L515 over USB 2.x often needs a lower color FPS even when rs-enumerate-devices
        # lists the individual stream profiles separately.
        add_candidate(640, 480, 30, 320, 240, 30)
        add_candidate(640, 480, 15, 320, 240, 30)
        add_candidate(640, 480, 6, 320, 240, 30)

        pipeline = rs.pipeline()
        last_error: Exception | None = None
        for color_width, color_height, color_fps, depth_width, depth_height, depth_fps in profile_candidates:
            config = rs.config()
            if self.serial:
                config.enable_device(self.serial)
            config.enable_stream(rs.stream.color, color_width, color_height, rs.format.rgb8, color_fps)
            config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, depth_fps)
            try:
                pipeline.start(config)
                self._active_color_profile = (color_width, color_height, color_fps)
                self._active_depth_profile = (depth_width, depth_height, depth_fps)
                if (color_width, color_height, color_fps, depth_width, depth_height, depth_fps) != profile_candidates[0]:
                    LOGGER.warning(
                        "Falling back to L515-compatible profile color=%sx%s@%s depth=%sx%s@%s",
                        color_width,
                        color_height,
                        color_fps,
                        depth_width,
                        depth_height,
                        depth_fps,
                    )
                break
            except Exception as exc:
                last_error = exc
                try:
                    pipeline.stop()
                except Exception:
                    pass
        else:
            raise RuntimeError(
                "Failed to start L515 camera. Requested stream profile is unsupported on the current USB link. "
                f"Try a USB 3 port/cable or rerun with --mock-camera. Last error: {last_error}"
            ) from last_error

        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color)
        self._rs = rs
        self._started = True
        LOGGER.info(
            "Connected to L515 camera color=%sx%s@%s depth=%sx%s@%s",
            *self._active_color_profile,
            *self._active_depth_profile,
        )
        return True

    def disconnect(self) -> None:
        if self._pipeline is not None and self._started:
            try:
                self._pipeline.stop()
            except Exception as exc:
                LOGGER.warning("Ignoring camera stop failure during cleanup: %s", exc)
        self._pipeline = None
        self._align = None
        self._rs = None
        self._started = False

    def _mock_frame(self, channels: int, encoding: str, bytes_per_pixel: int) -> Dict[str, Any]:
        return {
            "encoding": encoding,
            "width": self.width,
            "height": self.height,
            "data": bytes(self.width * self.height * channels * bytes_per_pixel),
        }

    def read(self) -> Dict[str, Dict[str, Any]]:
        if self.use_mock_camera:
            return {
                "top_rgb": self._mock_frame(channels=3, encoding="rgb8", bytes_per_pixel=1),
                "top_depth": self._mock_frame(channels=1, encoding="z16", bytes_per_pixel=2),
            }

        if self._pipeline is None or self._align is None:
            raise RuntimeError("Camera is not connected")

        frames = self._pipeline.wait_for_frames()
        aligned = self._align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("Failed to acquire camera frames")

        return {
            "top_rgb": {
                "encoding": "rgb8",
                "width": color_frame.get_width(),
                "height": color_frame.get_height(),
                "data": bytes(color_frame.get_data()),
            },
            "top_depth": {
                "encoding": "z16",
                "width": depth_frame.get_width(),
                "height": depth_frame.get_height(),
                "data": bytes(depth_frame.get_data()),
            },
        }


class URMedicineRobot(LeRobotRobot):
    """Minimal LeRobot Robot for the medicine picking setup."""

    config_class = URMedicineRobotConfig
    name = "ur_medicine"

    def __init__(self, config: URMedicineRobotConfig):
        super().__init__(config)
        self.robot = URRobotAdapter(
            host=config.host,
            command_port=config.command_port,
            state_port=config.state_port,
            legacy_root=config.legacy_root,
            backend=config.robot_backend,
            rtde_frequency=config.rtde_frequency,
        )
        self.gripper = Gripper(
            port=config.gripper_port,
            baudrate=config.gripper_baudrate,
            slave_id=config.gripper_slave_id,
            timeout=config.gripper_timeout,
            legacy_root=config.legacy_root,
        )
        self.camera = L515FrameSource(
            width=config.top_camera_width,
            height=config.top_camera_height,
            fps=config.top_camera_fps,
            depth_width=config.top_camera_depth_width,
            depth_height=config.top_camera_depth_height,
            depth_fps=config.top_camera_depth_fps,
            serial=config.top_camera_serial,
            use_mock_camera=config.use_mock_camera,
        )
        self.is_connected = False
        self._pose_book: Optional[dict[str, list[float]]] = None
        self._speed_mode_active = False

    @property
    def observation_features(self) -> Dict[str, Dict[str, Any]]:
        return {
            "observation.images.top_rgb": {
                "dtype": "bytes",
                "shape": [self.config.top_camera_height, self.config.top_camera_width, 3],
                "encoding": "rgb8",
            },
            "observation.images.top_depth": {
                "dtype": "bytes",
                "shape": [self.config.top_camera_height, self.config.top_camera_width],
                "encoding": "z16",
            },
            "observation.state.tcp_pose": {"dtype": "float32", "shape": [6]},
            "observation.state.joint_positions": {"dtype": "float32", "shape": [6]},
            "observation.state.gripper": {"dtype": "float32", "shape": [1]},
        }

    @property
    def action_features(self) -> Dict[str, Dict[str, Any]]:
        return {
            "action.ee_delta": {"dtype": "float32", "shape": [6]},
            "action.gripper": {"dtype": "float32", "shape": [1]},
        }

    def _load_pose_book(self) -> dict[str, list[float]]:
        if self._pose_book is None:
            pose_path = Path(self.config.pose_file).expanduser()
            with pose_path.open("r", encoding="utf-8") as handle:
                raw = yaml.safe_load(handle) or {}
            self._pose_book = {
                name: _normalize_pose(values, f"pose[{name}]")
                for name, values in raw.items()
            }
        return self._pose_book

    def connect(self) -> bool:
        if self.is_connected:
            return True

        try:
            if not self.robot.connect():
                raise RuntimeError("Failed to initialize UR adapter")
            if not self.gripper.connect():
                raise RuntimeError("Failed to initialize gripper adapter")
            self.camera.connect()
            self.is_connected = True
            LOGGER.info("UR medicine robot connected")
            return True
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        if self._speed_mode_active:
            try:
                self.robot.stop_linear(
                    accel=float(self.config.stop_acceleration_m_s2),
                    rot_accel=float(self.config.stop_rot_acceleration_rad_s2),
                )
            except Exception:
                pass
            self._speed_mode_active = False
        self.camera.disconnect()
        self.gripper.disconnect()
        self.robot.disconnect()
        self.is_connected = False
        LOGGER.info("UR medicine robot disconnected")

    def move_home(self) -> bool:
        if self._speed_mode_active:
            self.robot.stop_linear(
                accel=float(self.config.stop_acceleration_m_s2),
                rot_accel=float(self.config.stop_rot_acceleration_rad_s2),
            )
            self._speed_mode_active = False
        home_pose = self._load_pose_book()[self.config.home_pose_name]
        return self.robot.move_tcp(home_pose)

    def _clip_delta(self, ee_delta: Sequence[float]) -> list[float]:
        delta = _normalize_pose(ee_delta, "action.ee_delta")
        for index in range(3):
            delta[index] = _clamp(
                delta[index],
                -float(self.config.max_linear_step_m),
                float(self.config.max_linear_step_m),
            )
        for index in range(3, 6):
            delta[index] = _clamp(
                delta[index],
                -float(self.config.max_angular_step_rad),
                float(self.config.max_angular_step_rad),
            )
        return delta

    def _enforce_workspace(self, current_pose: Sequence[float], delta: Sequence[float]) -> list[float]:
        safe_delta = list(delta)
        for axis in range(3):
            target_axis = float(current_pose[axis]) + float(safe_delta[axis])
            clamped = _clamp(
                target_axis,
                float(self.config.workspace_min[axis]),
                float(self.config.workspace_max[axis]),
            )
            safe_delta[axis] = clamped - float(current_pose[axis])
        return safe_delta

    def _delta_to_speed(self, safe_delta: Sequence[float]) -> list[float]:
        dt = max(float(self.config.control_period_s), 1e-3)
        speed = [float(value) / dt for value in safe_delta]
        for index in range(3):
            speed[index] = _clamp(
                speed[index],
                -float(self.config.max_linear_speed_m_s),
                float(self.config.max_linear_speed_m_s),
            )
        for index in range(3, 6):
            speed[index] = _clamp(
                speed[index],
                -float(self.config.max_angular_speed_rad_s),
                float(self.config.max_angular_speed_rad_s),
            )
        return speed

    def get_observation(self) -> Dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self.name} is not connected.")

        images = self.camera.read()
        tcp_pose = self.robot.get_tcp_pose()
        joint_positions = self.robot.get_joint_positions()
        if tcp_pose is None:
            raise RuntimeError("Failed to read TCP pose")
        if joint_positions is None:
            joint_positions = _read_realtime_joints(
                self.config.host,
                self.config.state_port,
                self.robot.socket_timeout,
            )
        if joint_positions is None:
            raise RuntimeError("Failed to read joint positions")

        return {
            "observation.images.top_rgb": images["top_rgb"],
            "observation.images.top_depth": images["top_depth"],
            "observation.state.tcp_pose": tcp_pose,
            "observation.state.joint_positions": joint_positions,
            "observation.state.gripper": [self.gripper.get_state()],
        }

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self.name} is not connected.")

        ee_delta = action.get("action.ee_delta", [0.0] * 6)
        gripper_action = action.get("action.gripper", [self.gripper.get_state()])
        if isinstance(gripper_action, (int, float)):
            gripper_value = float(gripper_action)
        else:
            gripper_value = float(list(gripper_action)[0])

        clipped_delta = self._clip_delta(ee_delta)
        if any(abs(value) > 1e-9 for value in clipped_delta):
            current_pose = self.robot.get_tcp_pose()
            if current_pose is None:
                raise RuntimeError("Cannot send ee_delta without current TCP pose")
            safe_delta = self._enforce_workspace(current_pose, clipped_delta)
        else:
            safe_delta = [0.0] * 6

        has_motion = any(abs(value) > 1e-9 for value in safe_delta)
        if bool(self.config.use_speed_control):
            if has_motion:
                speed_command = self._delta_to_speed(safe_delta)
                if not self.robot.speed_linear(
                    speed_command,
                    accel=float(self.config.speed_acceleration_m_s2),
                    duration=float(self.config.control_period_s) * 2.0,
                    rot_accel=float(self.config.speed_rot_acceleration_rad_s2),
                ):
                    raise RuntimeError("Failed to send speedl motion to UR")
                self._speed_mode_active = True
            elif self._speed_mode_active:
                if not self.robot.stop_linear(
                    accel=float(self.config.stop_acceleration_m_s2),
                    rot_accel=float(self.config.stop_rot_acceleration_rad_s2),
                ):
                    raise RuntimeError("Failed to send stopl to UR")
                self._speed_mode_active = False
        elif has_motion:
            if not self.robot.delta_move(
                safe_delta,
                wait=bool(self.config.wait_for_motion),
            ):
                raise RuntimeError("Failed to send delta motion to UR")

        gripper_value = _clamp(gripper_value, 0.0, 1.0)
        if abs(gripper_value - self.gripper.get_state()) > (0.5 / 255.0):
            if not self.gripper.set_opening(int(round(gripper_value * 255.0))):
                raise RuntimeError("Failed to send gripper command")

        return {
            "action.ee_delta": safe_delta,
            "action.gripper": [gripper_value],
        }
