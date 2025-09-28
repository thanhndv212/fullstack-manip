import numpy as np
from typing import TYPE_CHECKING, List

if TYPE_CHECKING:
    from fullstack_manip.scripts.robot import Robot
import mujoco


class PickPlaceController:
    def __init__(
        self,
        robot: "Robot",
        gripper_joint_names: List[str] = ["Jaw"],
        object_geom: str = "cube",
    ):
        self.robot = robot
        self.robot.gripper_joint_names = gripper_joint_names
        self.robot.object_geom = object_geom
        self.robot.close_position = 0.0  # Closed position
        self.robot.open_position = 1.0  # Open position
        self.robot.grasp_force_threshold = 5.0  # Minimum force for grasp
        self.robot.GRASP_SUCCESS = False
        self.robot.release_force_threshold = 1.0  # Maximum force for release
        self.reach_threshold = np.array([0.1, 0.1, 0.1])  # Max distance to consider reachable

    def is_within_reach(self, position: np.ndarray) -> bool:
        """Check if a position is within reach"""
        if not isinstance(position, np.ndarray) or position.shape != (3,):
            raise ValueError("Position must be a 3D numpy array")
        current_position, _ = self.robot.get_body_pose(self.robot.end_effector_name)
        # each axis within threshold
        for i in range(3):
            if abs(current_position[i] - position[i]) > self.reach_threshold[i]:
                return False
        return True

    def pick_object(self, object_position: np.ndarray) -> None:
        """Pick up an object at given position"""
        if not isinstance(
            object_position, np.ndarray
        ) or object_position.shape != (3,):
            raise ValueError("Object position must be a 3D numpy array")
        import time

        init_object_pos = object_position.copy()
        self.robot.GRASP_SUCCESS = False
        while not self.robot.GRASP_SUCCESS:
            object_position, _ = self.robot.get_body_pose(self.robot.object_geom)

            if not self.is_within_reach(init_object_pos):
                raise ValueError("Object position is out of reach")
            # Move to top of object
            top_position = object_position + np.array([0.025, 0.005, 0.05])
            self.robot.move_to_position(top_position)  # Above object
            print("Stage 1: Moved above object", top_position)
            time.sleep(3)

            # Open gripper
            self.robot._open_gripper()
            print("Stage 2: Opened gripper")
            time.sleep(3)

            # Get updated object position
            object_position, _ = self.robot.get_body_pose(
                self.robot.object_geom
            )
            # Move to object position
            grasp_position = object_position + np.array([0.025, 0.005, 0.005])
            self.robot.move_to_position(grasp_position)
            print("Stage 3: Moved to object position", grasp_position)
            time.sleep(3)

            # Close gripper (grasp)
            self.robot._close_gripper()
            print("Stage 4: Closed gripper")
            time.sleep(3)
            if not self.robot.GRASP_SUCCESS:
                print("Grasp failed, retrying...")
                continue

    def place_object(self, target_position: np.ndarray) -> None:
        """Place object at target position"""
        if not isinstance(
            target_position, np.ndarray
        ) or target_position.shape != (3,):
            raise ValueError("Target position must be a 3D numpy array")

        # Move to target position
        self.robot.move_to_position(target_position)

        # Open gripper (release)
        self.robot._open_gripper()
