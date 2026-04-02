"""Thin adapter around the legacy UR robot control module."""

from __future__ import annotations

import logging
import os
import socket
import struct
import sys
import threading
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
        backend: str = "auto",
        rtde_frequency: float = 500.0,
        socket_timeout: float = 3.0,
        default_tool_acc: float = 0.68,
        default_tool_vel: float = 0.15,
    ) -> None:
        self.host = host
        self.command_port = int(command_port)
        self.state_port = int(state_port)
        self.legacy_root = Path(legacy_root).expanduser()
        self.backend = backend
        self.rtde_frequency = float(rtde_frequency)
        self.socket_timeout = float(socket_timeout)
        self.default_tool_acc = float(default_tool_acc)
        self.default_tool_vel = float(default_tool_vel)
        self._connected = False
        self._legacy_ur = None
        self._active_backend = "legacy"
        self._rtde_control = None
        self._rtde_receive = None
        self._command_stream_socket = None
        self._stream_lock = threading.Lock()

    def disconnect(self) -> None:
        """Reset the adapter state."""

        self._close_command_stream()
        if self._rtde_control is not None:
            try:
                if hasattr(self._rtde_control, "speedStop"):
                    self._rtde_control.speedStop()
            except Exception:
                pass
            try:
                if hasattr(self._rtde_control, "stopScript"):
                    self._rtde_control.stopScript()
            except Exception:
                pass
            try:
                self._rtde_control.disconnect()
            except Exception:
                pass
        if self._rtde_receive is not None:
            try:
                self._rtde_receive.disconnect()
            except Exception:
                pass
        self._rtde_control = None
        self._rtde_receive = None
        self._legacy_ur = None
        self._connected = False
        LOGGER.info("UR adapter disconnected from host=%s", self.host)

    def _close_command_stream(self) -> None:
        if self._command_stream_socket is not None:
            try:
                self._command_stream_socket.close()
            except Exception:
                pass
        self._command_stream_socket = None

    def _send_urscript(self, command: str, persistent: bool = False) -> bool:
        self._ensure_ready()

        if persistent:
            with self._stream_lock:
                for _ in range(2):
                    try:
                        if self._command_stream_socket is None:
                            stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            stream.settimeout(self.socket_timeout)
                            stream.connect((self.host, self.command_port))
                            self._command_stream_socket = stream
                        self._command_stream_socket.sendall(command.encode("utf-8"))
                        return True
                    except Exception as exc:
                        LOGGER.warning("Persistent URScript send failed: %s", exc)
                        self._close_command_stream()
                return False

        tcp_socket = None
        try:
            tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_socket.settimeout(self.socket_timeout)
            tcp_socket.connect((self.host, self.command_port))
            tcp_socket.sendall(command.encode("utf-8"))
            return True
        except Exception as exc:
            LOGGER.error("URScript send failed: %s", exc)
            return False
        finally:
            if tcp_socket is not None:
                try:
                    tcp_socket.close()
                except Exception:
                    pass

    def _read_realtime_packet(self) -> Optional[bytes]:
        tcp_socket = None
        try:
            tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_socket.settimeout(self.socket_timeout)
            tcp_socket.connect((self.host, self.state_port))
            data = tcp_socket.recv(1108)
            if len(data) < 300:
                LOGGER.warning("Realtime packet too short from host=%s", self.host)
                return None
            return data
        except Exception as exc:
            LOGGER.warning("Failed to read UR realtime packet from host=%s: %s", self.host, exc)
            return None
        finally:
            if tcp_socket is not None:
                try:
                    tcp_socket.close()
                except Exception:
                    pass

    def connect(self, verify_connection: bool = False) -> bool:
        """Validate configuration and optionally probe the realtime state port."""

        if not self.host:
            raise ValueError("host must not be empty")

        backend_candidates = ["ur_rtde", "legacy"] if self.backend == "auto" else [self.backend]
        connect_error = None
        for candidate in backend_candidates:
            try:
                if candidate == "ur_rtde":
                    from rtde_control import RTDEControlInterface  # type: ignore
                    from rtde_receive import RTDEReceiveInterface  # type: ignore

                    self._rtde_control = RTDEControlInterface(self.host, self.rtde_frequency)
                    self._rtde_receive = RTDEReceiveInterface(self.host, self.rtde_frequency)
                    self._active_backend = "ur_rtde"
                    self._connected = True
                    LOGGER.info(
                        "UR adapter initialized with ur_rtde for host=%s frequency=%sHz",
                        self.host,
                        self.rtde_frequency,
                    )
                    break
                if candidate == "legacy":
                    self._legacy_ur, _ = _load_legacy_ur_modules(self.legacy_root)
                    self._active_backend = "legacy"
                    self._connected = True
                    LOGGER.info(
                        "UR adapter initialized with legacy sockets for host=%s cmd_port=%s state_port=%s",
                        self.host,
                        self.command_port,
                        self.state_port,
                    )
                    break
                raise ValueError(f"Unsupported UR backend: {candidate}")
            except Exception as exc:
                connect_error = exc
                self._rtde_control = None
                self._rtde_receive = None
                self._legacy_ur = None
                self._connected = False
                LOGGER.warning("Failed to initialize UR backend %s: %s", candidate, exc)

        if not self._connected:
            raise RuntimeError(f"Failed to initialize UR adapter: {connect_error}")

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
        if self._active_backend == "ur_rtde":
            try:
                tcp = list(self._rtde_receive.getActualTCPPose())
            except Exception as exc:
                LOGGER.warning("Failed to read current TCP pose from ur_rtde: %s", exc)
                return None
            if len(tcp) != 6:
                return None
            rpy = rotvec_to_rpy(tcp[3:6], legacy_root=self.legacy_root)
            pose = [float(tcp[0]), float(tcp[1]), float(tcp[2]), *rpy]
            LOGGER.debug("Current TCP pose: %s", pose)
            return pose

        pose = self._legacy_ur.get_current_tcp(
            HOST=self.host,
            PORT=self.state_port,
            socket_timeout=self.socket_timeout,
        )
        if pose is None:
            LOGGER.warning("Failed to read current TCP pose from host=%s", self.host)
            return None

        tcp = _normalize_tcp(pose, "tcp_pose")
        LOGGER.debug("Current TCP pose: %s", tcp)
        return tcp

    def get_joint_positions(self) -> Optional[List[float]]:
        """Read the current joint positions in radians from the realtime state packet."""

        self._ensure_ready()
        if self._active_backend == "ur_rtde":
            try:
                joints = list(self._rtde_receive.getActualQ())
            except Exception as exc:
                LOGGER.warning("Failed to read current joints from ur_rtde: %s", exc)
                return None
            LOGGER.debug("Current joint positions: %s", joints)
            return joints

        packet = self._read_realtime_packet()
        if packet is None:
            return None

        joints = list(struct.unpack("!6d", packet[252:300]))
        LOGGER.debug("Current joint positions: %s", joints)
        return joints

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
        if self._active_backend == "ur_rtde":
            target_rv = rpy_to_rotvec(target[3:6], legacy_root=self.legacy_root)
            target_pose = [target[0], target[1], target[2], *target_rv]
            try:
                success = bool(
                    self._rtde_control.moveL(
                        target_pose,
                        self.default_tool_vel if tool_vel is None else float(tool_vel),
                        self.default_tool_acc if tool_acc is None else float(tool_acc),
                        not wait,
                    )
                )
            except Exception as exc:
                LOGGER.error("RTDE moveL failed for target=%s: %s", target, exc)
                return False
            if not success:
                LOGGER.error("TCP move failed for target=%s", target)
            return success

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

        LOGGER.debug("Sending TCP move to %s (wait=%s)", target, wait)
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
        if self._active_backend == "ur_rtde":
            try:
                success = bool(
                    self._rtde_control.moveJ(
                        joint_values,
                        self.default_tool_vel if tool_vel is None else float(tool_vel),
                        self.default_tool_acc if tool_acc is None else float(tool_acc),
                        False,
                    )
                )
            except TypeError:
                try:
                    success = bool(
                        self._rtde_control.moveJ(
                            joint_values,
                            self.default_tool_vel if tool_vel is None else float(tool_vel),
                            self.default_tool_acc if tool_acc is None else float(tool_acc),
                        )
                    )
                except Exception as exc:
                    LOGGER.error("RTDE moveJ failed for joints=%s: %s", joint_values, exc)
                    return False
            except Exception as exc:
                LOGGER.error("RTDE moveJ failed for joints=%s: %s", joint_values, exc)
                return False
            if not success:
                LOGGER.error("Joint move failed for joints=%s", joint_values)
            return success

        LOGGER.debug("Sending joint move to %s", joint_values)
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

        LOGGER.debug("Sending delta TCP move %s (wait=%s)", delta, wait)
        if wait:
            success = bool(self._legacy_ur.increase_move(**kwargs))
        else:
            current_tcp = self.get_tcp_pose()
            if current_tcp is None:
                LOGGER.error("Cannot send async delta move because current TCP pose is unavailable")
                return False
            target_tcp = list(current_tcp)
            for index, value in enumerate(delta):
                target_tcp[index] += value
            success = self.move_tcp(
                target_tcp=target_tcp,
                wait=False,
                tool_acc=tool_acc,
                tool_vel=tool_vel,
                tool_pos_tolerance=tool_pos_tolerance,
                max_wait=max_wait,
            )
        if not success:
            LOGGER.error("Delta TCP move failed for delta=%s", delta)
        return success

    def speed_linear(
        self,
        tool_speed: Sequence[float],
        accel: float = 0.25,
        duration: float = 0.1,
        rot_accel: Optional[float] = None,
    ) -> bool:
        """Send a continuous Cartesian speed command using URScript speedl()."""

        self._ensure_ready()
        speed = _normalize_tcp(tool_speed, "tool_speed")
        if self._active_backend == "ur_rtde":
            try:
                return bool(
                    self._rtde_control.speedL(
                        speed,
                        float(accel),
                        float(duration),
                    )
                )
            except Exception as exc:
                LOGGER.error("RTDE speedL failed for tool_speed=%s: %s", speed, exc)
                return False

        rot_acc = float(accel if rot_accel is None else rot_accel)
        command = (
            "speedl([%f,%f,%f,%f,%f,%f],%f,%f,%f)\n"
            % (
                speed[0],
                speed[1],
                speed[2],
                speed[3],
                speed[4],
                speed[5],
                float(accel),
                float(duration),
                rot_acc,
            )
        )
        if not self._send_urscript(command, persistent=True):
            LOGGER.error("speedl failed for tool_speed=%s", speed)
            return False
        return True

    def stop_linear(
        self,
        accel: float = 1.0,
        rot_accel: Optional[float] = None,
    ) -> bool:
        """Stop Cartesian speed control using URScript stopl()."""

        self._ensure_ready()
        if self._active_backend == "ur_rtde":
            try:
                return bool(self._rtde_control.speedStop(float(accel)))
            except Exception as exc:
                LOGGER.error("RTDE speedStop failed: %s", exc)
                return False

        if rot_accel is None:
            command = "stopl(%f)\n" % float(accel)
        else:
            command = "stopl(%f,%f)\n" % (float(accel), float(rot_accel))
        if not self._send_urscript(command, persistent=True):
            LOGGER.error("stopl failed")
            return False
        return True
