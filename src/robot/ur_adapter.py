"""Thin adapter around the legacy UR robot control module."""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path
from typing import List, Optional, Sequence

LOGGER = logging.getLogger(__name__)

DEFAULT_LEGACY_ROOT = Path(
    os.getenv("MEDICINE_VLA_LEGACY_ROOT", "/home/nav/JuShen_new")
).expanduser()


def _normalize_tcp(values: Sequence[float], field_name: str) -> List[float]:
    tcp = [float(value) for value in values]
    if len(tcp) != 6:
        raise ValueError(f"{field_name} must contain 6 values, got {len(tcp)}")
    return tcp


def _normalize_vector3(values: Sequence[float], field_name: str) -> List[float]:
    vector = [float(value) for value in values]
    if len(vector) != 3:
        raise ValueError(f"{field_name} must contain 3 values, got {len(vector)}")
    return vector


def _ensure_legacy_root(legacy_root: Path) -> None:
    if not legacy_root.exists():
        raise FileNotFoundError(f"Legacy code root not found: {legacy_root}")
    legacy_root_str = str(legacy_root)
    if legacy_root_str not in sys.path:
        sys.path.insert(0, legacy_root_str)


def _load_legacy_ur_modules(legacy_root: Path):
    _ensure_legacy_root(legacy_root)

    from UR import UR_Robot as legacy_ur  # type: ignore
    from UR import util as legacy_util  # type: ignore

    return legacy_ur, legacy_util


def rpy_to_rotvec(
    rpy: Sequence[float],
    legacy_root: Path | str = DEFAULT_LEGACY_ROOT,
) -> List[float]:
    """Convert roll-pitch-yaw to UR rotvec using the legacy utility module."""

    _, legacy_util = _load_legacy_ur_modules(Path(legacy_root))
    return legacy_util.rpy2rv(_normalize_vector3(rpy, "rpy")).tolist()


def rotvec_to_rpy(
    rotvec: Sequence[float],
    legacy_root: Path | str = DEFAULT_LEGACY_ROOT,
) -> List[float]:
    """Convert UR rotvec to roll-pitch-yaw using the legacy utility module."""

    _, legacy_util = _load_legacy_ur_modules(Path(legacy_root))
    vector = _normalize_vector3(rotvec, "rotvec")
    return legacy_util.rv2rpy(*vector).tolist()


class URRobotAdapter:
    """Project-level adapter for the legacy UR robot controller."""

    def __init__(
        self,
        host: str,
        command_port: int = 30002,
        state_port: int = 30003,
        legacy_root: Path | str = DEFAULT_LEGACY_ROOT,
        socket_timeout: float = 3.0,
        default_tool_acc: float = 0.68,
        default_tool_vel: float = 0.15,
    ) -> None:
        self.host = host
        self.command_port = int(command_port)
        self.state_port = int(state_port)
        self.legacy_root = Path(legacy_root).expanduser()
        self.socket_timeout = float(socket_timeout)
        self.default_tool_acc = float(default_tool_acc)
        self.default_tool_vel = float(default_tool_vel)
        self._connected = False
        self._legacy_ur = None

    def connect(self, verify_connection: bool = False) -> bool:
        """Validate configuration and optionally probe the realtime state port."""

        if not self.host:
            raise ValueError("host must not be empty")

        self._legacy_ur, _ = _load_legacy_ur_modules(self.legacy_root)
        self._connected = True
        LOGGER.info(
            "UR adapter initialized for host=%s cmd_port=%s state_port=%s",
            self.host,
            self.command_port,
            self.state_port,
        )

        if verify_connection:
            pose = self.get_tcp_pose()
            if pose is None:
                LOGGER.warning("UR realtime pose probe failed for host=%s", self.host)
                return False
        return True

    def _ensure_ready(self) -> None:
        if not self._connected:
            self.connect()

    def get_tcp_pose(self) -> Optional[List[float]]:
        """Read the current TCP pose in `[x, y, z, roll, pitch, yaw]`."""

        self._ensure_ready()
        pose = self._legacy_ur.get_current_tcp(
            HOST=self.host,
            PORT=self.state_port,
            socket_timeout=self.socket_timeout,
        )
        if pose is None:
            LOGGER.warning("Failed to read current TCP pose from host=%s", self.host)
            return None

        tcp = _normalize_tcp(pose, "tcp_pose")
        LOGGER.info("Current TCP pose: %s", tcp)
        return tcp

    def move_tcp(
        self,
        target_tcp: Sequence[float],
        wait: bool = True,
        tool_acc: Optional[float] = None,
        tool_vel: Optional[float] = None,
        tool_pos_tolerance: Optional[Sequence[float]] = None,
        max_wait: Optional[float] = None,
    ) -> bool:
        """Move the robot TCP to a target pose."""

        self._ensure_ready()
        target = _normalize_tcp(target_tcp, "target_tcp")
        kwargs = {
            "HOST": self.host,
            "CMD_PORT": self.command_port,
            "STATE_PORT": self.state_port,
            "target_tcp": target,
            "wait": wait,
            "socket_timeout": self.socket_timeout,
            "tool_acc": self.default_tool_acc if tool_acc is None else float(tool_acc),
            "tool_vel": self.default_tool_vel if tool_vel is None else float(tool_vel),
        }
        if tool_pos_tolerance is not None:
            kwargs["tool_pos_tolerance"] = _normalize_tcp(
                tool_pos_tolerance,
                "tool_pos_tolerance",
            )
        if max_wait is not None:
            kwargs["max_wait"] = float(max_wait)

        LOGGER.info("Sending TCP move to %s (wait=%s)", target, wait)
        success = bool(self._legacy_ur.move_to_tcp(**kwargs))
        if not success:
            LOGGER.error("TCP move failed for target=%s", target)
        return success

    def move_joints(
        self,
        joints: Sequence[float],
        tool_acc: Optional[float] = None,
        tool_vel: Optional[float] = None,
        move_time: float = 2.0,
    ) -> bool:
        """Send a joint-space move command."""

        self._ensure_ready()
        joint_values = _normalize_tcp(joints, "joints")
        LOGGER.info("Sending joint move to %s", joint_values)
        success = bool(
            self._legacy_ur.move_to_joints(
                HOST=self.host,
                CMD_PORT=self.command_port,
                joint_command=joint_values,
                socket_timeout=self.socket_timeout,
                tool_acc=self.default_tool_acc if tool_acc is None else float(tool_acc),
                tool_vel=self.default_tool_vel if tool_vel is None else float(tool_vel),
                t=float(move_time),
            )
        )
        if not success:
            LOGGER.error("Joint move failed for joints=%s", joint_values)
        return success

    def delta_move(
        self,
        delta_tcp: Sequence[float],
        wait: bool = True,
        tool_acc: Optional[float] = None,
        tool_vel: Optional[float] = None,
        tool_pos_tolerance: Optional[Sequence[float]] = None,
        max_wait: Optional[float] = None,
    ) -> bool:
        """Send a small delta motion in TCP space."""

        self._ensure_ready()
        delta = _normalize_tcp(delta_tcp, "delta_tcp")
        kwargs = {
            "HOST": self.host,
            "CMD_PORT": self.command_port,
            "STATE_PORT": self.state_port,
            "delta_tcp": delta,
            "socket_timeout": self.socket_timeout,
            "tool_acc": self.default_tool_acc if tool_acc is None else float(tool_acc),
            "tool_vel": self.default_tool_vel if tool_vel is None else float(tool_vel),
        }
        if tool_pos_tolerance is not None:
            kwargs["tool_pos_tolerance"] = _normalize_tcp(
                tool_pos_tolerance,
                "tool_pos_tolerance",
            )
        if max_wait is not None:
            kwargs["max_wait"] = float(max_wait)

        LOGGER.info("Sending delta TCP move %s (wait=%s)", delta, wait)
        success = bool(self._legacy_ur.increase_move(**kwargs))
        if wait is False:
            LOGGER.warning(
                "Legacy increase_move always waits for completion; wait=%s was ignored",
                wait,
            )
        if not success:
            LOGGER.error("Delta TCP move failed for delta=%s", delta)
        return success
