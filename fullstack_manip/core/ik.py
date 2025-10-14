"""Inverse kinematics solver using Mink."""

from typing import Optional, List

import mink
import mujoco
import numpy as np

from fullstack_manip.utils.loop_rate_limiters import RateLimiter


# IK settings
SOLVER = "daqp"
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 100
RATE = RateLimiter(frequency=100.0, warn=False)


class IKSolver:
    """Inverse kinematics solver using Mink."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        limits: Optional[List] = None,
    ):
        """
        Initialize IK solver.

        Args:
            model: MuJoCo model
            data: MuJoCo data
            limits: List of limit objects for IK solver
        """
        self.model = model
        self.data = data
        self.limits = limits or []

        # Initialize Mink configuration
        self.configuration = mink.Configuration(
            self.model,
            self.model.key("home").qpos,
        )
        mujoco.mj_resetDataKeyframe(
            self.model,
            self.data,
            self.model.key("home").id,
        )

    def solve_ik_for_qpos(
        self,
        frame_name: str,
        frame_type: str,
        target_pos: np.ndarray,
        target_orient: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Solve inverse kinematics for a target pose using Mink.

        Args:
            frame_name: Name of the frame (e.g., end-effector)
            frame_type: Type of the frame
            target_pos: Target position (3D array)
            target_orient: Target orientation (quaternion), optional

        Returns:
            Joint configuration (first 6 joints)
        """
        tasks = []

        self.configuration.update(self.data.qpos)

        target_rotation = (
            mink.SO3(target_orient)
            if target_orient is not None
            else mink.SO3.identity()
        )

        target_pose = mink.SE3.from_rotation_and_translation(
            target_rotation,
            target_pos,
        )
        pose_task = mink.FrameTask(
            frame_name=frame_name,
            frame_type=frame_type,
            position_cost=1.0,
            orientation_cost=0.0,
            lm_damping=1e-6,
        )
        pose_task.set_target(target_pose)

        tasks.append(pose_task)

        for i in range(MAX_ITERS):
            vel = mink.solve_ik(
                self.configuration,
                tasks,
                RATE.dt,
                solver=SOLVER,
                limits=self.limits,
            )
            self.configuration.integrate_inplace(vel, RATE.dt)
            err = pose_task.compute_error(self.configuration)
            pos_achieved = np.linalg.norm(err[:3]) <= POS_THRESHOLD
            ori_achieved = np.linalg.norm(err[3:]) <= ORI_THRESHOLD
            if pos_achieved and ori_achieved:
                break

        return self.configuration.q.copy()[:6]


__all__ = ["IKSolver"]
