"""Record raw teleoperation episodes with keyboard control."""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
import uuid
from pathlib import Path
from typing import Any, Dict

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "plugins" / "lerobot_robot_ur_medicine"))
sys.path.insert(0, str(REPO_ROOT / "plugins" / "lerobot_teleoperator_keyboard_ur"))

from lerobot_robot_ur_medicine import URMedicineRobot, URMedicineRobotConfig  # type: ignore
from lerobot_teleoperator_keyboard_ur import (  # type: ignore
    KeyboardURTeleoperator,
    KeyboardURTeleoperatorConfig,
)
from src.lerobot_processors.ur_keyboard_processors import (
    robot_observation_to_dataset_observation,
    teleop_action_to_dataset_action,
    teleop_action_to_robot_action,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", default=None, help="Task text. If omitted, prompt in terminal.")
    parser.add_argument("--host", default="192.168.56.2", help="UR robot IP address.")
    parser.add_argument("--command-port", type=int, default=30002, help="UR command port.")
    parser.add_argument("--state-port", type=int, default=30003, help="UR realtime state port.")
    parser.add_argument("--robot-backend", default="auto", choices=["auto", "ur_rtde", "legacy"], help="UR control backend.")
    parser.add_argument("--gripper-port", default="/dev/ttyUSB0", help="Gripper serial port.")
    parser.add_argument("--legacy-root", default="/home/nav/JuShen_new", help="Legacy code root.")
    parser.add_argument("--pose-file", default=str(REPO_ROOT / "config" / "ur_pos.yaml"), help="Pose YAML path.")
    parser.add_argument("--mock-camera", action="store_true", help="Use mock camera frames.")
    parser.add_argument("--linear-step", type=float, default=0.005, help="Keyboard linear step in meters.")
    parser.add_argument("--angular-step", type=float, default=0.05, help="Keyboard angular step in radians.")
    parser.add_argument("--poll-timeout", type=float, default=0.02, help="Keyboard poll timeout in seconds.")
    parser.add_argument("--linear-speed", type=float, default=0.10, help="Maximum Cartesian teleop speed in m/s.")
    parser.add_argument("--angular-speed", type=float, default=0.50, help="Maximum Cartesian angular teleop speed in rad/s.")
    parser.add_argument(
        "--input-backend",
        default="pynput",
        choices=["pynput", "stdin"],
        help="Keyboard input backend. Use pynput for hold-to-move control.",
    )
    parser.add_argument(
        "--record-rate-hz",
        type=float,
        default=5.0,
        help="How often to save observation/action steps while teleoperating.",
    )
    parser.add_argument("--wait-for-motion", action="store_true", help="Wait for each robot move to finish before reading the next key.")
    parser.add_argument("--output-dir", default=str(REPO_ROOT / "data" / "raw"), help="Output directory for raw episodes.")
    parser.add_argument("--success", choices=["yes", "no"], default=None, help="Episode success flag.")
    return parser


def _json_ready_observation(
    observation: Dict[str, Any],
    step_dir: Path,
    step_index: int,
) -> Dict[str, Any]:
    step_dir.mkdir(parents=True, exist_ok=True)
    result = dict(observation)
    for key in ("observation.images.top_rgb", "observation.images.top_depth"):
        image = dict(result[key])
        image_data = image.pop("data")
        suffix = "rgb.bin" if key.endswith("top_rgb") else "depth.bin"
        image_path = step_dir / f"{step_index:06d}_{suffix}"
        image_path.write_bytes(image_data)
        image["file"] = image_path.name
        result[key] = image
    return result


def _write_step(
    episode_dir: Path,
    step_index: int,
    payload: Dict[str, Any],
) -> None:
    step_dir = episode_dir / "steps"
    step_dir.mkdir(parents=True, exist_ok=True)
    step_path = step_dir / f"{step_index:06d}.json"
    step_path.write_text(json.dumps(payload, indent=2, ensure_ascii=True), encoding="utf-8")


def _resolve_task(task_arg: str | None) -> str:
    if task_arg:
        return task_arg
    return input("task> ").strip() or "unspecified-task"


def _resolve_success(success_arg: str | None) -> bool:
    if success_arg is not None:
        return success_arg == "yes"
    answer = input("success [y/N]> ").strip().lower()
    return answer in {"y", "yes"}


def _is_nonzero_delta(action: Dict[str, Any]) -> bool:
    return any(abs(float(value)) > 1e-9 for value in action.get("action.ee_delta", []))


def _gripper_changed(action: Dict[str, Any], current_target: float) -> bool:
    values = action.get("action.gripper", [current_target])
    target = float(values[0] if not isinstance(values, (int, float)) else values)
    return abs(target - current_target) > 1e-9


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logging.getLogger("src.robot.ur_adapter").setLevel(logging.WARNING)
    logging.getLogger("src.gripper.gripper_adapter").setLevel(logging.INFO)
    logging.getLogger("lerobot_robot_ur_medicine.ur_medicine").setLevel(logging.INFO)
    args = build_parser().parse_args()

    task_text = _resolve_task(args.task)
    episode_id = time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
    episode_dir = Path(args.output_dir).expanduser() / episode_id
    episode_dir.mkdir(parents=True, exist_ok=False)

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
            control_period_s=args.poll_timeout,
            max_linear_speed_m_s=args.linear_speed,
            max_angular_speed_rad_s=args.angular_speed,
            wait_for_motion=args.wait_for_motion,
        )
    )
    teleop = KeyboardURTeleoperator(
        KeyboardURTeleoperatorConfig(
            linear_step_m=args.linear_step,
            angular_step_rad=args.angular_step,
            poll_timeout_s=args.poll_timeout,
            input_backend=args.input_backend,
        )
    )

    logging.info("Episode directory: %s", episode_dir)
    logging.info("Controls: w/s/a/d/q/e move, i/k/j/l/u/o rotate, space gripper, r home, esc stop.")

    step_index = 0
    started_at = time.time()
    run_error = None
    current_gripper_target = 1.0
    last_record_time = 0.0
    last_motion_active = False

    try:
        robot.connect()
        teleop.connect()

        while not teleop.should_exit():
            teleop_action = teleop.get_action()
            if teleop.consume_home_request():
                logging.info("Home requested from keyboard.")
                robot.move_home()
                current_gripper_target = teleop.get_gripper_target()
                last_motion_active = False
                continue

            motion_active = _is_nonzero_delta(teleop_action)
            should_send = motion_active or last_motion_active or _gripper_changed(
                teleop_action,
                current_gripper_target,
            )
            if not should_send:
                continue

            robot_action = teleop_action_to_robot_action(teleop_action)
            gripper_changed = _gripper_changed(teleop_action, current_gripper_target)
            executed_action = robot.send_action(robot_action)
            current_gripper_target = float(executed_action["action.gripper"][0])
            last_motion_active = motion_active

            now = time.time()
            should_record = (
                step_index == 0
                or (args.record_rate_hz > 0 and (now - last_record_time) >= (1.0 / args.record_rate_hz))
                or gripper_changed
            )
            if not should_record:
                continue

            observation_before = robot.get_observation()
            last_record_time = now

            step_payload = {
                "timestamp": now,
                "episode_id": episode_id,
                "task": task_text,
                "observation": _json_ready_observation(
                    robot_observation_to_dataset_observation(observation_before),
                    episode_dir / "steps",
                    step_index,
                ),
                "action": teleop_action_to_dataset_action(executed_action),
            }
            _write_step(episode_dir, step_index, step_payload)
            step_index += 1
    except KeyboardInterrupt as exc:
        run_error = exc
    except Exception as exc:
        run_error = exc
    finally:
        teleop.disconnect()
        robot.disconnect()

    if run_error is not None:
        metadata = {
            "episode_id": episode_id,
            "task": task_text,
            "success": False,
            "step_count": step_index,
            "started_at": started_at,
            "ended_at": time.time(),
            "error": str(run_error),
        }
        (episode_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=True),
            encoding="utf-8",
        )
        if isinstance(run_error, KeyboardInterrupt):
            logging.warning("Recording interrupted by keyboard.")
        else:
            logging.error("Recording aborted: %s", run_error)
        logging.error("Partial episode metadata saved to %s", episode_dir / "metadata.json")
        return 1

    success = _resolve_success(args.success)
    metadata = {
        "episode_id": episode_id,
        "task": task_text,
        "success": success,
        "step_count": step_index,
        "started_at": started_at,
        "ended_at": time.time(),
    }
    (episode_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )
    logging.info("Saved episode metadata to %s", episode_dir / "metadata.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
