"""Print keyboard teleop actions without sending motion commands."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "plugins" / "lerobot_teleoperator_keyboard_ur"))

from lerobot_teleoperator_keyboard_ur import (  # type: ignore
    KeyboardURTeleoperator,
    KeyboardURTeleoperatorConfig,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--linear-step", type=float, default=0.005, help="Linear step in meters.")
    parser.add_argument("--angular-step", type=float, default=0.05, help="Angular step in radians.")
    parser.add_argument("--poll-timeout", type=float, default=0.05, help="Keyboard poll timeout in seconds.")
    parser.add_argument(
        "--input-backend",
        default="pynput",
        choices=["pynput", "stdin"],
        help="Keyboard input backend. Use pynput for hold-to-move control.",
    )
    return parser


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    args = build_parser().parse_args()

    teleop = KeyboardURTeleoperator(
        KeyboardURTeleoperatorConfig(
            linear_step_m=args.linear_step,
            angular_step_rad=args.angular_step,
            poll_timeout_s=args.poll_timeout,
            input_backend=args.input_backend,
        )
    )

    teleop.connect()
    print("Keyboard teleop active: w/s/a/d/q/e move, i/k/j/l/u/o rotate, space gripper, r home, esc exit.")

    try:
        while not teleop.should_exit():
            action = teleop.get_action()
            if teleop.consume_home_request():
                print("HOME requested")
            print(action)
            time.sleep(args.poll_timeout)
    finally:
        teleop.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
