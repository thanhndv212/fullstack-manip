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
        self.gripper_joint_names = gripper_joint_names
        self.object_geom = object_geom
        self.close_position = 0.0  # Closed position
        self.open_position = 0.5  # Open position
        self.grasp_force_threshold = 5.0  # Minimum force for grasp
        self.GRASP_SUCCESS = False
        self.release_force_threshold = 1.0  # Maximum force for release

    def pick_object(self, object_position: np.ndarray) -> None:
        """Pick up an object at given position"""
        if not isinstance(
            object_position, np.ndarray
        ) or object_position.shape != (3,):
            raise ValueError("Object position must be a 3D numpy array")
        import time

        # Move to top of object
        self.robot.move_to_position(object_position + np.array([0, 0, 0.015]))  # Above object
        time.sleep(3)

        while not self.GRASP_SUCCESS:
            # Open gripper
            self._open_gripper()
            time.sleep(3)
            # Move to object position
            self.robot.move_to_position(object_position)
            time.sleep(3)

            # Close gripper (grasp)
            self._close_gripper()
            time.sleep(3)

            # Lift object
            self.robot.move_to_position(
                object_position + np.array([0, 0, 0.015])
            )
            time.sleep(3)

    def place_object(self, target_position: np.ndarray) -> None:
        """Place object at target position"""
        if not isinstance(
            target_position, np.ndarray
        ) or target_position.shape != (3,):
            raise ValueError("Target position must be a 3D numpy array")

        # Move to target position
        self.robot.move_to_position(target_position)

        # Open gripper (release)
        self._open_gripper()

    def compute_contact_force(self, geom1_name: str, geom2_name: str) -> float:
        """
        Compute the total contact force magnitude between two geometries

        Args:
            geom1_name: Name of first geometry
            geom2_name: Name of second geometry

        Returns:
            Total force magnitude
        """
        geom1_id = mujoco.mj_name2id(
            self.robot.model, mujoco.mjtObj.mjOBJ_GEOM, geom1_name
        )
        geom2_id = mujoco.mj_name2id(
            self.robot.model, mujoco.mjtObj.mjOBJ_GEOM, geom2_name
        )
        total_force = 0.0
        for contact in self.robot.data.contact[: self.robot.data.ncon]:
            if (contact.geom1 == geom1_id and contact.geom2 == geom2_id) or (
                contact.geom1 == geom2_id and contact.geom2 == geom1_id
            ):
                total_force += np.linalg.norm(contact.force)
        return total_force

    def _close_gripper(self) -> None:
        """Close robot gripper and check for successful grasp"""
        # Set gripper joints to closed position
        for joint_name in self.gripper_joint_names:
            joint_id = self.robot.model.joint(joint_name).id
            self.robot.data.qpos[joint_id] = self.close_position
        mujoco.mj_forward(self.robot.model, self.robot.data)

        # Check for grasp success: contact forces > threshold
        total_force = 0.0
        for joint_name in self.gripper_joint_names:
            gripper_geom = (
                f"{joint_name}_geom"  # Assume geom name based on joint
            )
            total_force += self.compute_contact_force(
                gripper_geom, self.object_geom
            )
        if total_force > self.grasp_force_threshold:
            self.GRASP_SUCCESS = True
            print("Grasp successful")
        else:
            print("Grasp failed")

    def _open_gripper(self) -> None:
        """Open robot gripper and check for successful release"""
        # Set gripper joints to open position
        for joint_name in self.gripper_joint_names:
            joint_id = self.robot.model.joint(joint_name).id
            self.robot.data.qpos[joint_id] = self.open_position
        mujoco.mj_forward(self.robot.model, self.robot.data)

        # Check for release success: contact forces < threshold
        total_force = 0.0
        for joint_name in self.gripper_joint_names:
            gripper_geom = f"{joint_name}_geom"
            total_force += self.compute_contact_force(
                gripper_geom, self.object_geom
            )
        if total_force < self.release_force_threshold:
            print("Release successful")
        else:
            print("Release failed")
