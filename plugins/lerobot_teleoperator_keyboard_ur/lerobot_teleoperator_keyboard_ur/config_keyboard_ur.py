"""Configuration for the keyboard teleoperator."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


try:
    from lerobot.teleoperators import TeleoperatorConfig as LeRobotTeleoperatorConfig  # type: ignore
except ImportError:
    @dataclass
    class LeRobotTeleoperatorConfig:
        """Fallback TeleoperatorConfig used when lerobot is not installed."""

        type: str = "keyboard_ur"

        @classmethod
        def register_subclass(cls, _name: str) -> Callable[[type], type]:
            def decorator(subclass: type) -> type:
                return subclass

            return decorator


@LeRobotTeleoperatorConfig.register_subclass("keyboard_ur")
@dataclass
class KeyboardURTeleoperatorConfig(LeRobotTeleoperatorConfig):
    """Minimal keyboard teleoperation configuration."""

    linear_step_m: float = 0.005
    angular_step_rad: float = 0.05
    poll_timeout_s: float = 0.05
    input_backend: str = "pynput"
