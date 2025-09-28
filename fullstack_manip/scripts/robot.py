import mujoco
import numpy as np
import mink
from typing import Optional, Tuple
from motion_planner import MotionPlanner
from pid_controller import PIDController
from viewer import MuJoCoViewer
import time


class Robot:

    def __init__(
        self,
        model: mujoco.MjModel,
        data: Optional[mujoco.MjData] = None,
        end_effector_name: str = "Fixed_Jaw",
    ):
        try:
            self.model = model
            self.data = mujoco.MjData(self.model)
            self.end_effector_name = end_effector_name

            # Initialize motion planner
            self.motion_planner = MotionPlanner(
                self.model,
                self.data,
                self.end_effector_name,
            )
            # Initialize PID controller
            self.pid_controller = PIDController(
                kp=10.0,
                ki=0.1,
                kd=1.0,
                dt=0.01,
                integral_limit=1.0,
                output_limit=10.0,
            )
            # Initialize viewer
            self.viewer = MuJoCoViewer(self.model, self.data)
            self.viewer.launch_passive()
            self.viewer.sync()
            # Robot and scene dimensions
            self.scene_nq = self.model.nq
            self.scene_nu = self.model.nu
            self.robot_nq = self.model.nu
            self.dt = 0.01
        except Exception as e:
            raise RuntimeError(f"Failed to initialize robot: {e}")

    def get_robot_joint_positions(self):
        """Get robot joint positions (assumes first robot_nq joints)."""
        if self.data:
            return self.data.qpos[: self.robot_nq]
        return None

    def set_robot_joint_positions(self, joint_positions) -> None:
        """Set robot joint positions (assumes first robot_nq joints)."""
        if self.data:
            self.data.qpos[: self.robot_nq] = joint_positions
            mujoco.mj_forward(self.model, self.data)

    def get_body_pose(self, body_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """Get body position and orientation"""
        mujoco.mj_forward(self.model, self.data)
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        pos = self.data.xpos[body_id].copy()
        rot = self.data.xmat[body_id].reshape(3, 3).copy()
        return pos, rot

    def move_to_position(
        self, target_pos: np.ndarray, duration: float = 4.0
    ) -> None:
        """Move robot end-effector to target position using Mink IK"""

        if not isinstance(target_pos, np.ndarray) or target_pos.shape != (3,):
            raise ValueError("Target position must be a 3D numpy array")

        current_ee_pos, _ = self.get_body_pose(self.end_effector_name)

        # Plan trajectory
        trajectory, t = self.motion_planner.plan_trajectory(
            current_ee_pos, target_pos, duration=duration
        )

        if trajectory is None:
            raise RuntimeError("Failed to plan trajectory")

        current_joint_positions = self.get_robot_joint_positions()
        # Execute trajectory with PID control
        count = 0
        for target in trajectory:
            target_joint_positions = target
            control_signal = self.pid_controller.compute(
                target_joint_positions, current_joint_positions
            )
            # Apply control signal (assuming direct position control for simplicity)
            current_joint_positions += control_signal * self.dt
            self.viewer.step(current_joint_positions)
            time.sleep(self.dt)
            count += 1
            if self.detect_contact(self.end_effector_name, self.object_geom):
                print("Contact detected with cube, stopping movement.")
                break
        print(f"Executed {count} control steps to reach target position.")

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
            self.model, mujoco.mjtObj.mjOBJ_GEOM, geom1_name
        )
        geom2_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, geom2_name
        )
        total_force = 0.0
        for contact in self.data.contact[: self.data.ncon]:
            if (contact.geom1 == geom1_id and contact.geom2 == geom2_id) or (
                contact.geom1 == geom2_id and contact.geom2 == geom1_id
            ):
                total_force += np.linalg.norm(contact.force)
        return total_force

    def _close_gripper(self) -> None:
        """Close robot gripper and check for successful grasp"""
        # Set gripper joints to closed position
        for joint_name in self.gripper_joint_names:
            joint_id = self.model.joint(joint_name).id
            self.data.qpos[joint_id] = self.close_position
        current_joint_positions = self.get_robot_joint_positions()
        self.viewer.step(current_joint_positions)
        time.sleep(self.dt)

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
            joint_id = self.model.joint(joint_name).id
            self.data.qpos[joint_id] = self.open_position
        current_joint_positions = self.get_robot_joint_positions()
        self.viewer.step(current_joint_positions)
        time.sleep(self.dt)

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

    def detect_contact(self, geom1_name: str, geom2_name: str) -> bool:
        """
        Detect if two geometries are in contact

        Args:
            geom1_name: Name of first geometry
            geom2_name: Name of second geometry

        Returns:
            True if the geometries are in contact, False otherwise
        """
        geom1_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, geom1_name
        )
        geom2_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, geom2_name
        )
        for contact in self.data.contact[: self.data.ncon]:
            if (contact.geom1 == geom1_id and contact.geom2 == geom2_id) or (
                contact.geom1 == geom2_id and contact.geom2 == geom1_id
            ):
                return True
        return False
