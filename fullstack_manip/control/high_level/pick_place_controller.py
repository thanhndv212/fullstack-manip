from __future__ import annotations

from typing import TYPE_CHECKING, List

import numpy as np

from .base_controller import BaseController

if TYPE_CHECKING:  # pragma: no cover - circular import guard
    from .robot import Robot


class PickPlaceController(BaseController):
    """High-level pick-and-place routine built on the Robot abstraction."""

    def __init__(
        self,
        robot: "Robot",
        gripper_joint_names: List[str] | None = None,
        object_geom: str = None,
    ) -> None:
        if gripper_joint_names is None:
            raise ValueError("Gripper joint names must be provided")

        super().__init__(robot)

        self.robot.gripper_joint_names = gripper_joint_names
        self.robot.object_geom = object_geom
        self.robot.close_position = 0.0
        self.robot.open_position = 1.0
        self.robot.grasp_force_threshold = 1.0
        self.robot.GRASP_SUCCESS = False
        self.robot.release_force_threshold = 0.01

    def execute(
        self,
        pick_position: np.ndarray,
        place_position: np.ndarray,
    ) -> None:
        """Execute the pick-and-place behavior.

        Args:
            pick_position: Target position [x, y, z] for picking the object.
            place_position: Target position [x, y, z] for placing the object.
        """
        self.pick_object(pick_position)
        self.place_object(place_position)

    def pick_object(self, object_position: np.ndarray) -> None:
        """Execute a pick sequence at the provided Cartesian target."""
        if not isinstance(object_position, np.ndarray):
            raise ValueError("Object position must be a 3D numpy array")
        if object_position.shape != (3,):
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
        if not isinstance(target_position, np.ndarray):
            raise ValueError("Target position must be a 3D numpy array")
        if target_position.shape != (3,):
            raise ValueError("Target position must be a 3D numpy array")

        self.robot.move_to_position(target_position)
        self.robot._open_gripper()


__all__ = ["PickPlaceController"]
