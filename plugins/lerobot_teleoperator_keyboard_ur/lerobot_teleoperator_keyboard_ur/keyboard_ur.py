"""Minimal keyboard teleoperator for UR end-effector delta control."""

from __future__ import annotations

import logging
import select
import sys
import termios
import threading
import time
import tty
from typing import Any, Dict

try:
    from lerobot.teleoperators import Teleoperator as LeRobotTeleoperator  # type: ignore
except ImportError:
    class LeRobotTeleoperator:
        """Fallback Teleoperator base used when lerobot is not installed."""

        config_class = None
        name = "teleoperator"

        def __init__(self, config: Any):
            self.config = config


from .config_keyboard_ur import KeyboardURTeleoperatorConfig

LOGGER = logging.getLogger(__name__)


class KeyboardURTeleoperator(LeRobotTeleoperator):
    """Keyboard teleoperator that outputs ee_delta + gripper actions."""

    config_class = KeyboardURTeleoperatorConfig
    name = "keyboard_ur"

    def __init__(self, config: KeyboardURTeleoperatorConfig):
        super().__init__(config)
        self.is_connected = False
        self._terminal_fd: int | None = None
        self._old_termios = None
        self._gripper_open = True
        self._home_requested = False
        self._exit_requested = False
        self._active_keys: set[str] = set()
        self._listener = None
        self._listener_lock = threading.Lock()
        self._stdin_escape_active = False

    @property
    def action_features(self) -> Dict[str, Dict[str, Any]]:
        return {
            "action.ee_delta": {"dtype": "float32", "shape": [6]},
            "action.gripper": {"dtype": "float32", "shape": [1]},
        }

    def connect(self) -> bool:
        if self.is_connected:
            return True
        if self.config.input_backend == "pynput":
            self._connect_pynput()
        elif self.config.input_backend == "stdin":
            self._connect_stdin()
        else:
            raise ValueError(f"Unsupported input backend: {self.config.input_backend}")
        self.is_connected = True
        LOGGER.info(
            "Keyboard teleoperator connected with backend=%s. Press ESC to exit.",
            self.config.input_backend,
        )
        return True

    def disconnect(self) -> None:
        if self._listener is not None:
            self._listener.stop()
        self._listener = None
        if self._terminal_fd is not None and self._old_termios is not None:
            termios.tcsetattr(self._terminal_fd, termios.TCSADRAIN, self._old_termios)
        self._terminal_fd = None
        self._old_termios = None
        self._active_keys.clear()
        self._stdin_escape_active = False
        self.is_connected = False

    def consume_home_request(self) -> bool:
        requested = self._home_requested
        self._home_requested = False
        return requested

    def should_exit(self) -> bool:
        return self._exit_requested

    def get_gripper_target(self) -> float:
        return 1.0 if self._gripper_open else 0.0

    def _zero_action(self) -> Dict[str, Any]:
        return {
            "action.ee_delta": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "action.gripper": [self.get_gripper_target()],
        }

    def _connect_stdin(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleoperator requires a TTY stdin.")
        self._terminal_fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(self._terminal_fd)
        tty.setcbreak(self._terminal_fd)

    def _connect_pynput(self) -> None:
        if sys.stdin.isatty():
            self._terminal_fd = sys.stdin.fileno()
            self._old_termios = termios.tcgetattr(self._terminal_fd)
            new_attr = termios.tcgetattr(self._terminal_fd)
            new_attr[3] = new_attr[3] & ~termios.ECHO
            termios.tcsetattr(self._terminal_fd, termios.TCSADRAIN, new_attr)

        try:
            from pynput import keyboard as pynput_keyboard  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "pynput is required for hold-to-move keyboard control. "
                "Install it or rerun with --input-backend stdin."
            ) from exc

        def on_press(key: Any) -> None:
            key_name = self._normalize_pynput_key(key)
            if key_name is None:
                return
            with self._listener_lock:
                if key_name == "space":
                    self._gripper_open = not self._gripper_open
                elif key_name == "r":
                    self._home_requested = True
                elif key_name == "esc":
                    self._exit_requested = True
                else:
                    self._active_keys.add(key_name)

        def on_release(key: Any) -> bool | None:
            key_name = self._normalize_pynput_key(key)
            if key_name is None:
                return None
            with self._listener_lock:
                self._active_keys.discard(key_name)
                should_stop = key_name == "esc"
            if should_stop:
                return False
            return None

        self._listener = pynput_keyboard.Listener(on_press=on_press, on_release=on_release)
        self._listener.start()

    @staticmethod
    def _normalize_pynput_key(key: Any) -> str | None:
        try:
            char = key.char
        except AttributeError:
            char = None
        if char:
            return str(char).lower()
        key_name = getattr(key, "name", None)
        if key_name in {"space", "esc"}:
            return str(key_name)
        return None

    def _current_delta_from_active_keys(self) -> list[float]:
        linear = float(self.config.linear_step_m)
        angular = float(self.config.angular_step_rad)
        delta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        with self._listener_lock:
            active_keys = set(self._active_keys)

        if "w" in active_keys:
            delta[1] += linear
        if "s" in active_keys:
            delta[1] -= linear
        if "a" in active_keys:
            delta[0] -= linear
        if "d" in active_keys:
            delta[0] += linear
        if "q" in active_keys:
            delta[2] += linear
        if "e" in active_keys:
            delta[2] -= linear
        if "u" in active_keys:
            delta[3] += angular
        if "o" in active_keys:
            delta[3] -= angular
        if "i" in active_keys:
            delta[4] += angular
        if "k" in active_keys:
            delta[4] -= angular
        if "j" in active_keys:
            delta[5] += angular
        if "l" in active_keys:
            delta[5] -= angular
        return delta

    def _map_stdin_key(self, key: str) -> Dict[str, Any]:
        action = self._zero_action()
        if key in {"w", "s", "a", "d", "q", "e", "u", "o", "i", "k", "j", "l"}:
            with self._listener_lock:
                self._active_keys = {key}
            action["action.ee_delta"] = self._current_delta_from_active_keys()
        elif key == " ":
            self._gripper_open = not self._gripper_open
            action["action.gripper"] = [self.get_gripper_target()]
        elif key == "r":
            self._home_requested = True
        elif key == "\x1b":
            self._stdin_escape_active = True
            self._exit_requested = True

        return action

    def get_action(self) -> Dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self.name} is not connected.")

        if self.config.input_backend == "pynput":
            time.sleep(float(self.config.poll_timeout_s))
            action = self._zero_action()
            action["action.ee_delta"] = self._current_delta_from_active_keys()
            return action

        readable, _, _ = select.select([sys.stdin], [], [], float(self.config.poll_timeout_s))
        if not readable:
            if self._stdin_escape_active:
                with self._listener_lock:
                    self._active_keys.clear()
                self._stdin_escape_active = False
            return self._zero_action()

        key = sys.stdin.read(1)
        return self._map_stdin_key(key)

    def send_feedback(self, _feedback: Dict[str, Any] | None = None) -> None:
        """No-op hook kept for LeRobot teleoperator compatibility."""

        return None
