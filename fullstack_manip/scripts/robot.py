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

    def get_end_effector_position(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get end effector position and orientation"""
        mujoco.mj_forward(self.model, self.data)
        ee_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, self.end_effector_name
        )
        pos = self.data.xpos[ee_id].copy()
        rot = self.data.xmat[ee_id].reshape(3, 3).copy()
        return pos, rot

    def move_to_position(
        self, target_pos: np.ndarray, duration: float = 4.0
    ) -> None:
        """Move robot end-effector to target position using Mink IK"""

        if not isinstance(target_pos, np.ndarray) or target_pos.shape != (3,):
            raise ValueError("Target position must be a 3D numpy array")

        current_ee_pos, _ = self.get_end_effector_position()

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
        print(f"Executed {count} control steps to reach target position.")
