"""Minimal test for the UR medicine LeRobot robot plugin."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "plugins" / "lerobot_robot_ur_medicine"))

from lerobot_robot_ur_medicine import URMedicineRobot, URMedicineRobotConfig  # type: ignore


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.56.2", help="UR robot IP address.")
    parser.add_argument("--command-port", type=int, default=30002, help="UR command port.")
    parser.add_argument("--state-port", type=int, default=30003, help="UR realtime state port.")
    parser.add_argument("--robot-backend", default="auto", choices=["auto", "ur_rtde", "legacy"], help="UR control backend.")
    parser.add_argument("--gripper-port", default="/dev/ttyUSB0", help="Gripper serial port.")
    parser.add_argument("--legacy-root", default="/home/nav/JuShen_new", help="Legacy code root.")
    parser.add_argument("--pose-file", default=str(REPO_ROOT / "config" / "ur_pos.yaml"), help="Pose YAML path.")
    parser.add_argument("--mock-camera", action="store_true", help="Use mock top camera frames.")
    return parser


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    args = build_parser().parse_args()

    robot = URMedicineRobot(
        URMedicineRobotConfig(
            host=args.host,
            command_port=args.command_port,
            state_port=args.state_port,
            robot_backend=args.robot_backend,
            gripper_port=args.gripper_port,
            legacy_root=args.legacy_root,
            pose_file=args.pose_file,
            use_mock_camera=args.mock_camera,
        )
    )

    try:
        robot.connect()
        observation = robot.get_observation()
        print(sorted(observation.keys()))
        print("tcp_pose", observation["observation.state.tcp_pose"])
        print("joint_positions", observation["observation.state.joint_positions"])
        print("gripper", observation["observation.state.gripper"])
        print("top_rgb", observation["observation.images.top_rgb"]["width"], observation["observation.images.top_rgb"]["height"])
        print("top_depth", observation["observation.images.top_depth"]["width"], observation["observation.images.top_depth"]["height"])
    finally:
        robot.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
