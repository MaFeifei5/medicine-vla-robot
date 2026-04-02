"""Minimal pick-and-place pipeline skeleton for the graduation project."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence

from src.gripper.gripper_adapter import Gripper
from src.robot.ur_adapter import URRobotAdapter

DEFAULT_LEGACY_POSE_FILE = Path("/home/nav/JuShen_new/funcs/ur_pos.yaml")


def _normalize_pose(name: str, pose: Sequence[float]) -> list[float]:
    values = [float(value) for value in pose]
    if len(values) != 6:
        raise ValueError(f"Pose '{name}' must contain 6 values, got {len(values)}")
    return values


class PickPlacePipeline:
    """Small, extensible pipeline that orchestrates robot and gripper adapters."""

    def __init__(
        self,
        robot: URRobotAdapter,
        gripper: Gripper,
        poses: Optional[Mapping[str, Sequence[float]]] = None,
    ) -> None:
        self.robot = robot
        self.gripper = gripper
        self.poses: Dict[str, list[float]] = {}
        if poses:
            self.poses = {
                name: _normalize_pose(name, pose)
                for name, pose in poses.items()
            }

    @classmethod
    def from_yaml(
        cls,
        robot: URRobotAdapter,
        gripper: Gripper,
        pose_file: Path | str = DEFAULT_LEGACY_POSE_FILE,
    ) -> "PickPlacePipeline":
        """Build the pipeline from a legacy pose YAML file."""

        try:
            import yaml
        except ImportError as exc:
            raise ImportError(
                "PyYAML is required to load pose files. Install it or pass poses directly."
            ) from exc

        pose_path = Path(pose_file).expanduser()
        with pose_path.open("r", encoding="utf-8") as handle:
            pose_map = yaml.safe_load(handle) or {}
        return cls(robot=robot, gripper=gripper, poses=pose_map)

    def get_pose(self, name: str) -> list[float]:
        """Return a named pose from the loaded pose book."""

        if name not in self.poses:
            raise KeyError(f"Pose '{name}' is not defined")
        return list(self.poses[name])

    @staticmethod
    def _require_step(step_name: str, success: bool) -> None:
        if not success:
            raise RuntimeError(f"Pipeline step failed: {step_name}")

    def move_home(self, pose_name: str = "home") -> bool:
        """Move to the configured home pose."""

        return self.robot.move_tcp(self.get_pose(pose_name))

    def move_pregrasp(self, pose_name: str = "pregrasp") -> bool:
        """Move above the target before descending."""

        return self.robot.move_tcp(self.get_pose(pose_name))

    def descend(self, delta_tcp: Sequence[float] = (0.0, 0.0, -0.05, 0.0, 0.0, 0.0)) -> bool:
        """Descend a small amount from the current pre-grasp pose."""

        return self.robot.delta_move(delta_tcp)

    def close_gripper(self, tightness: int = 180, speed: int = 140) -> bool:
        """Close the gripper on the target object."""

        return self.gripper.close(tightness=tightness, speed=speed)

    def lift(self, delta_tcp: Sequence[float] = (0.0, 0.0, 0.05, 0.0, 0.0, 0.0)) -> bool:
        """Lift the grasped object after closing the gripper."""

        return self.robot.delta_move(delta_tcp)

    def move_to_tray(self, pose_name: str = "release") -> bool:
        """Move to the tray or release pose."""

        return self.robot.move_tcp(self.get_pose(pose_name))

    def open_gripper(self, speed: int = 180, torque: int = 140) -> bool:
        """Open the gripper to release the object."""

        return self.gripper.open(speed=speed, torque=torque)

    def return_home(self, pose_name: str = "home") -> bool:
        """Return to the configured home pose."""

        return self.move_home(pose_name=pose_name)

    def run_once(
        self,
        pregrasp_pose_name: str = "pregrasp",
        tray_pose_name: str = "release",
    ) -> None:
        """Run one minimal pick-and-place cycle.

        This is intentionally a skeleton:
        1. Move to home.
        2. Move to a pre-grasp pose prepared by manual calibration or future vision.
        3. Descend.
        4. Close gripper.
        5. Lift.
        6. Move to tray.
        7. Open gripper.
        8. Return home.
        """

        self._require_step("move_home", self.move_home())
        self._require_step(
            "move_pregrasp",
            self.move_pregrasp(pose_name=pregrasp_pose_name),
        )
        self._require_step("descend", self.descend())
        self._require_step("close_gripper", self.close_gripper())
        self._require_step("lift", self.lift())
        self._require_step(
            "move_to_tray",
            self.move_to_tray(pose_name=tray_pose_name),
        )
        self._require_step("open_gripper", self.open_gripper())
        self._require_step("return_home", self.return_home())
