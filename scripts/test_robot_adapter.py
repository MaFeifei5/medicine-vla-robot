"""Minimal safety-first test for the UR robot adapter."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.robot.ur_adapter import URRobotAdapter


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.56.2", help="UR robot IP address.")
    parser.add_argument("--command-port", type=int, default=30002, help="UR command port.")
    parser.add_argument("--state-port", type=int, default=30003, help="UR realtime state port.")
    parser.add_argument(
        "--legacy-root",
        default="/home/nav/JuShen_new",
        help="Root directory of the legacy UR control code.",
    )
    parser.add_argument(
        "--verify-connect",
        action="store_true",
        help="Probe the realtime state port during connect().",
    )
    parser.add_argument(
        "--run-delta-move",
        action="store_true",
        help="Execute a small delta TCP move after reading the current pose.",
    )
    parser.add_argument(
        "--delta",
        type=float,
        nargs=6,
        default=[0.0, 0.0, 0.005, 0.0, 0.0, 0.0],
        metavar=("DX", "DY", "DZ", "DROLL", "DPITCH", "DYAW"),
        help="Small delta TCP move used only with --run-delta-move.",
    )
    return parser


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    args = build_parser().parse_args()

    robot = URRobotAdapter(
        host=args.host,
        command_port=args.command_port,
        state_port=args.state_port,
        legacy_root=args.legacy_root,
    )

    if not robot.connect(verify_connection=args.verify_connect):
        logging.error("UR adapter connection probe failed.")
        return 1

    pose = robot.get_tcp_pose()
    if pose is None:
        logging.error("Could not read current TCP pose.")
        return 1

    logging.info("Current TCP pose: %s", pose)

    if args.run_delta_move:
        logging.warning("Executing delta move: %s", args.delta)
        if not robot.delta_move(args.delta):
            logging.error("Delta move execution failed.")
            return 1
    else:
        logging.info("Delta move skipped. Re-run with --run-delta-move to execute a small motion.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
