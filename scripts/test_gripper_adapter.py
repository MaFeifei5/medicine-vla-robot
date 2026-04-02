"""Minimal safety-first test for the gripper adapter."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.gripper.gripper_adapter import Gripper


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Gripper serial port.")
    parser.add_argument("--baudrate", type=int, default=115200, help="Gripper baudrate.")
    parser.add_argument("--slave-id", type=int, default=1, help="Modbus slave ID.")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial timeout in seconds.")
    parser.add_argument(
        "--legacy-root",
        default="/home/nav/JuShen_new",
        help="Root directory of the legacy gripper control code.",
    )
    parser.add_argument(
        "--run-actions",
        action="store_true",
        help="Execute open/close actions after connecting.",
    )
    parser.add_argument(
        "--pause-seconds",
        type=float,
        default=1.0,
        help="Pause duration between open and close commands.",
    )
    return parser


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    args = build_parser().parse_args()

    gripper = Gripper(
        port=args.port,
        baudrate=args.baudrate,
        slave_id=args.slave_id,
        timeout=args.timeout,
        legacy_root=args.legacy_root,
    )

    if not gripper.connect():
        logging.error("Gripper connection failed.")
        return 1

    try:
        if args.run_actions:
            if not gripper.open():
                return 1
            time.sleep(max(0.0, args.pause_seconds))
            if not gripper.close():
                return 1
        else:
            logging.info("Open/close actions skipped. Re-run with --run-actions to actuate the gripper.")
    finally:
        gripper.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
