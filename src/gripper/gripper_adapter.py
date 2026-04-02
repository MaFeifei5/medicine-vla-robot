"""Thin adapter around the legacy Modbus gripper controller."""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path
from typing import Optional

LOGGER = logging.getLogger(__name__)

DEFAULT_LEGACY_ROOT = Path(
    os.getenv("MEDICINE_VLA_LEGACY_ROOT", "/home/nav/JuShen_new")
).expanduser()


def _ensure_legacy_root(legacy_root: Path) -> None:
    if not legacy_root.exists():
        raise FileNotFoundError(f"Legacy code root not found: {legacy_root}")
    legacy_root_str = str(legacy_root)
    if legacy_root_str not in sys.path:
        sys.path.insert(0, legacy_root_str)


def _load_legacy_gripper_class(legacy_root: Path):
    _ensure_legacy_root(legacy_root)
    from funcs.gripper_ub_modbus import GripperModbusRTU  # type: ignore

    return GripperModbusRTU


class Gripper:
    """Project-level adapter for the legacy RS485 gripper."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        slave_id: int = 1,
        timeout: float = 0.2,
        legacy_root: Path | str = DEFAULT_LEGACY_ROOT,
    ) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.slave_id = int(slave_id)
        self.timeout = float(timeout)
        self.legacy_root = Path(legacy_root).expanduser()
        self._driver = None

    def connect(self) -> bool:
        """Open the serial connection to the gripper."""

        if self._driver is not None:
            LOGGER.info("Gripper already connected on %s", self.port)
            return True

        gripper_class = _load_legacy_gripper_class(self.legacy_root)
        self._driver = gripper_class(
            port=self.port,
            baudrate=self.baudrate,
            slave_id=self.slave_id,
            timeout=self.timeout,
        )
        try:
            self._driver.open()
        except Exception as exc:
            self._driver = None
            LOGGER.error("Failed to connect gripper on %s: %s", self.port, exc)
            return False

        LOGGER.info(
            "Gripper connected on port=%s baudrate=%s slave_id=%s",
            self.port,
            self.baudrate,
            self.slave_id,
        )
        return True

    def disconnect(self) -> None:
        """Close the serial connection if it is open."""

        if self._driver is None:
            return

        try:
            self._driver.close()
        finally:
            LOGGER.info("Gripper disconnected from %s", self.port)
            self._driver = None

    def _ensure_ready(self) -> None:
        if self._driver is None and not self.connect():
            raise RuntimeError("Gripper is not connected")

    def open(self, speed: int = 180, torque: int = 140) -> bool:
        """Open the gripper using the legacy default command."""

        self._ensure_ready()
        LOGGER.info("Opening gripper with speed=%s torque=%s", speed, torque)
        success = bool(self._driver.open_gripper(speed=speed, torque=torque))
        if not success:
            LOGGER.error("Gripper open command failed")
        return success

    def close(self, tightness: int = 180, speed: int = 140) -> bool:
        """Close the gripper using the legacy default command."""

        self._ensure_ready()
        LOGGER.info("Closing gripper with tightness=%s speed=%s", tightness, speed)
        success = bool(self._driver.close_gripper(tightness=tightness, speed=speed))
        if not success:
            LOGGER.error("Gripper close command failed")
        return success

    def set_opening(
        self,
        position: int,
        speed: int = 160,
        torque: int = 180,
        acc: int = 200,
        dec: int = 200,
    ) -> bool:
        """Set the gripper opening with the legacy register write interface."""

        self._ensure_ready()
        clipped_position = max(0, min(255, int(position)))
        LOGGER.info(
            "Setting gripper opening to position=%s speed=%s torque=%s",
            clipped_position,
            speed,
            torque,
        )
        success, _ = self._driver.write_regs_10_15(
            pos=clipped_position,
            vel=int(speed),
            torque=int(torque),
            acc=int(acc),
            dec=int(dec),
            trigger=1,
        )
        if not success:
            LOGGER.error("Failed to set gripper opening to %s", clipped_position)
        return bool(success)
