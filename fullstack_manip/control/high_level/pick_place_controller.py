from __future__ import annotations

from typing import TYPE_CHECKING, List

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - circular import guard
    from .robot import Robot


class PickPlaceController:
    """High-level pick-and-place routine built on the Robot abstraction."""

    def __init__(
        self,
        robot: "Robot",
        gripper_joint_names: List[str] | None = None,
        object_geom: str = None,
    ) -> None:
        if gripper_joint_names is None:
            raise ValueError("Gripper joint names must be provided")
        self.robot = robot
        self.robot.gripper_joint_names = gripper_joint_names
        self.robot.object_geom = object_geom
        self.robot.close_position = 0.0
        self.robot.open_position = 1.0
        self.robot.grasp_force_threshold = 1.0
        self.robot.GRASP_SUCCESS = False
        self.robot.release_force_threshold = 0.01
        self.reach_threshold = np.array([0.1, 0.1, 0.1])

    def is_within_reach(self, current_pos: np.ndarray = None, target_pos: np.ndarray = None) -> bool:
        """Return True if the desired pose is within reach of the gripper."""
        if not isinstance(target_pos, np.ndarray) or target_pos.shape != (3,):
            raise ValueError("Position must be a 3D numpy array")
        if current_pos is None:
            current_pos, _ = self.robot.get_body_pose(self.robot.end_effector_name)
        if not isinstance(current_pos, np.ndarray) or current_pos.shape != (3,):
            raise ValueError("Current position must be a 3D numpy array")
        
        return np.all(
            np.abs(current_pos - target_pos) <= self.reach_threshold
        )

    def pick_object(self, object_position: np.ndarray) -> None:
        """Execute a pick sequence at the provided Cartesian target."""
        if not (
            isinstance(object_position, np.ndarray)
            and object_position.shape == (3,)
        ):
            raise ValueError("Object position must be a 3D numpy array")

        import time

        init_object_pos = object_position.copy()
        self.robot.GRASP_SUCCESS = False
        top_offset = np.array([0.04, 0.0, 0.05])
        grasp_offset = np.array([0.02, 0.0, -0.005])
        while not self.robot.GRASP_SUCCESS:
            object_position, object_orientation = self.robot.get_body_pose(
                self.robot.object_geom
            )

            if not self.is_within_reach(object_position, init_object_pos):
                raise ValueError("Object position is out of reach")

            top_position = object_position + top_offset
            self.robot.move_to_position(top_position, object_orientation)
            time.sleep(3)

            self.robot._open_gripper()
            time.sleep(3)

            object_position, object_orientation = self.robot.get_body_pose(
                self.robot.object_geom
            )

            grasp_position = object_position + grasp_offset
            self.robot.move_to_position(grasp_position, object_orientation)
            time.sleep(3)

            self.robot._close_gripper()
            time.sleep(3)
            if not self.robot.GRASP_SUCCESS:
                print("Grasp failed, retrying...")

    def place_object(self, target_position: np.ndarray) -> None:
        """Place the grasped object at the desired Cartesian pose."""
        if not (
            isinstance(target_position, np.ndarray)
            and target_position.shape == (3,)
        ):
            raise ValueError("Target position must be a 3D numpy array")

        self.robot.move_to_position(target_position)
        self.robot._open_gripper()


__all__ = ["PickPlaceController"]
