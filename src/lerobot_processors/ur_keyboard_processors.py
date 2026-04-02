"""Minimal processors between keyboard teleop, robot actions, and dataset records."""

from __future__ import annotations

from typing import Any, Dict


def teleop_action_to_robot_action(teleop_action: Dict[str, Any]) -> Dict[str, Any]:
    """Convert keyboard teleop output into the robot action schema."""

    return {
        "action.ee_delta": list(teleop_action.get("action.ee_delta", [0.0] * 6)),
        "action.gripper": list(teleop_action.get("action.gripper", [1.0])),
    }


def dataset_action_to_robot_action(dataset_action: Dict[str, Any]) -> Dict[str, Any]:
    """Convert recorded dataset action back into the robot action schema."""

    return teleop_action_to_robot_action(dataset_action)


def robot_observation_to_dataset_observation(observation: Dict[str, Any]) -> Dict[str, Any]:
    """Filter the robot observation down to the recorded dataset observation schema."""

    return {
        "observation.images.top_rgb": observation["observation.images.top_rgb"],
        "observation.images.top_depth": observation["observation.images.top_depth"],
        "observation.state.tcp_pose": list(observation["observation.state.tcp_pose"]),
        "observation.state.joint_positions": list(observation["observation.state.joint_positions"]),
        "observation.state.gripper": list(observation["observation.state.gripper"]),
    }


def teleop_action_to_dataset_action(teleop_action: Dict[str, Any]) -> Dict[str, Any]:
    """Keep only the action fields that should be stored in the raw episode."""

    return teleop_action_to_robot_action(teleop_action)
