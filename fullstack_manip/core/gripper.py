"""Gripper class for manipulation tasks."""

from __future__ import annotations

import time
from typing import List, Optional, TYPE_CHECKING

import mujoco
import numpy as np

if TYPE_CHECKING:
    from .collision import CollisionChecker
    from ..simulation.viewer import MuJoCoViewer


class Gripper:
    """
    Gripper control and state management.
    
    Handles opening, closing, grasp verification, and force monitoring
    for robotic grippers.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        collision_checker: "CollisionChecker",
        viewer: Optional["MuJoCoViewer"] = None,
        end_effector_name: Optional[str] = None,
        gripper_bodies: Optional[List[str]] = None,
        gripper_joint_names: Optional[List[str]] = None,
        dt: float = 0.01,
    ):
        """
        Initialize gripper controller.

        Args:
            model: MuJoCo model
            data: MuJoCo data
            collision_checker: CollisionChecker instance for force computation
            viewer: Optional MuJoCoViewer for visualization
            end_effector_name: Name of the end effector body/site
            gripper_bodies: List of gripper body names for collision checking
            gripper_joint_names: List of gripper joint names to control
            dt: Time step for control loop
        """
        self.model = model
        self.data = data
        self.collision_checker = collision_checker
        self.viewer = viewer
        self.end_effector_name = end_effector_name
        self.gripper_bodies = (
            gripper_bodies if gripper_bodies is not None else []
        )
        self.gripper_joint_names = (
            gripper_joint_names if gripper_joint_names is not None else []
        )
        self.dt = dt

        # Gripper state and configuration
        self.close_position = 0.0
        self.open_position = 1.0
        self.grasp_force_threshold = 1.0
        self.release_force_threshold = 0.01
        self.object_geom: Optional[str] = None
        self.grasp_success = False

    def set_joint_names(self, joint_names: List[str]) -> None:
        """Set gripper joint names."""
        self.gripper_joint_names = joint_names

    def set_bodies(self, body_names: List[str]) -> None:
        """Set gripper body names for collision checking."""
        self.gripper_bodies = body_names

    def set_object_geom(self, object_geom: str) -> None:
        """Set the target object geometry name."""
        self.object_geom = object_geom

    def set_positions(self, close_pos: float, open_pos: float) -> None:
        """
        Set gripper open and close positions.

        Args:
            close_pos: Joint position for closed gripper
            open_pos: Joint position for open gripper
        """
        self.close_position = close_pos
        self.open_position = open_pos

    def set_force_thresholds(
        self,
        grasp_threshold: float,
        release_threshold: float
    ) -> None:
        """
        Set force thresholds for grasp verification.

        Args:
            grasp_threshold: Minimum force to consider grasp successful (N)
            release_threshold: Maximum force to consider release successful (N)
        """
        self.grasp_force_threshold = grasp_threshold
        self.release_force_threshold = release_threshold

    def get_position(self) -> Optional[float]:
        """Get current gripper position (average of all gripper joints)."""
        if not self.gripper_joint_names:
            return None
        
        positions = []
        for joint_name in self.gripper_joint_names:
            joint_id = self.model.joint(joint_name).id
            positions.append(self.data.qpos[joint_id])
        
        return float(np.mean(positions))

    def check_grasp_success(self) -> bool:
        """Check if grasp is successful."""
        return self.check_grasp_contact() and self.check_in_hand()

    def check_grasp_contact(self) -> bool:
        """Check if the grasp is successful based on contact forces."""
        if not self.gripper_bodies or self.object_geom is None:
            return False

        total_forces = {}
        for gripper_body in self.gripper_bodies:
            gripper_geoms = self.collision_checker.get_body_geom_ids(
                self.model.body(gripper_body).id,
            )
            total_forces[gripper_body] = (
                self.collision_checker.compute_contact_force(
                    gripper_geoms,
                    self.object_geom,
                )
            )
            print(
                "Grasp contact forces of body "
                f"{gripper_body}: {total_forces[gripper_body]}"
            )
        return all(
            force > self.grasp_force_threshold
            for force in total_forces.values()
        )

    def check_in_hand(self) -> bool:
        """Check if the object is still in hand based on relative position."""
        if self.object_geom is None or self.end_effector_name is None:
            return False

        # Get object position
        mujoco.mj_forward(self.model, self.data)
        object_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            self.object_geom,
        )
        object_pos = self.data.xpos[object_body_id].copy()

        # Get gripper position
        gripper_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            self.end_effector_name,
        )
        gripper_pos = self.data.xpos[gripper_body_id].copy()

        distance = np.linalg.norm(object_pos - gripper_pos)
        print(
            f"Debug info: gripper_pos = {gripper_pos}, "
            f"object_pos = {object_pos}"
        )
        print(f"Distance between object and gripper: {distance}")
        return distance < 0.015

    def close(self, check_grasp: bool = True) -> bool:
        """
        Close robot gripper and optionally check for successful grasp.

        Args:
            check_grasp: Whether to verify grasp success during closing

        Returns:
            True if grasp was successful, False otherwise
        """
        T = 0.5
        grasp_count = 0
        self.grasp_success = False

        for joint_name in self.gripper_joint_names:
            joint_id = self.model.joint(joint_name).id
            while abs(self.data.qpos[joint_id] - self.close_position) > 1e-3:
                current_gripper_position = self.data.qpos[joint_id]
                vel = (self.close_position - current_gripper_position) / T
                self.data.qpos[joint_id] = (
                    vel * self.dt + current_gripper_position
                )
                
                if self.viewer is not None:
                    # Get current robot joint positions for viewer
                    current_joint_positions = self.data.qpos[:self.model.nu]
                    self.viewer.step(current_joint_positions)
                    time.sleep(self.dt)

                if check_grasp and self.check_grasp_success():
                    grasp_count += 1
                else:
                    grasp_count = 0

                if check_grasp:
                    print(f"Grasp success count: {grasp_count}")
                    if grasp_count == 100:
                        self.grasp_success = True
                        print("Grasp successful")
                        break

        if check_grasp and not self.grasp_success:
            print("Grasp failed")

        return self.grasp_success

    def open(self, check_release: bool = True) -> bool:
        """
        Open robot gripper and optionally check for successful release.

        Args:
            check_release: Whether to verify release success

        Returns:
            True if release was successful, False otherwise
        """
        T = 0.5
        release_success = False

        for joint_name in self.gripper_joint_names:
            joint_id = self.model.joint(joint_name).id
            while abs(self.data.qpos[joint_id] - self.open_position) > 1e-3:
                current_gripper_position = self.data.qpos[joint_id]
                vel = (self.open_position - current_gripper_position) / T
                self.data.qpos[joint_id] = (
                    vel * self.dt + current_gripper_position
                )
                
                if self.viewer is not None:
                    # Get current robot joint positions for viewer
                    current_joint_positions = self.data.qpos[:self.model.nu]
                    self.viewer.step(current_joint_positions)
                    time.sleep(self.dt)

        if check_release:
            total_force = 0.0
            for gripper_body in self.gripper_bodies:
                gripper_geoms = self.collision_checker.get_body_geom_ids(
                    self.model.body(gripper_body).id,
                )
                total_force += self.collision_checker.compute_contact_force(
                    gripper_geoms,
                    self.object_geom,
                )
            
            if total_force < self.release_force_threshold:
                print("Release successful")
                release_success = True
            else:
                print("Release failed")
                release_success = False

        return release_success if check_release else True


__all__ = ["Gripper"]
