"""Motion execution utilities for high-level controllers.

Provides a decoupled helper that coordinates the robot, motion planner,
PID controller, and optional collision feedback to reach target poses.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

import numpy as np

from .low_level.pid_controller import PIDController
from ..core.robot import Robot
from ..planning.motion_planner import MotionPlanner
from ..utils.rate_config import RateConfig

LOGGER = logging.getLogger(__name__)


class MotionExecutor:
    """Execute Cartesian motions by coordinating planning and control."""

    def __init__(
        self,
        robot: Robot,
        motion_planner: Optional[MotionPlanner] = None,
        pid_controller: Optional[PIDController] = None,
        dt: Optional[float] = None,
    ) -> None:
        self.robot = robot
        if dt is not None:
            if dt <= 0.0:
                raise ValueError("MotionExecutor dt override must be positive")
        rates_source = getattr(robot, "rates", None)
        if rates_source is not None and not isinstance(
            rates_source, RateConfig
        ):
            raise TypeError(
                "robot.rates must be a RateConfig when present;"
                f" received {type(rates_source)}"
            )
        self.rates = rates_source or RateConfig()
        dt_value = float(dt or self.rates.control)
        self.dt = dt_value
        self._motion_planner = (
            motion_planner or self._create_default_motion_planner()
        )
        self._pid_controller = (
            pid_controller or self._create_default_pid_controller()
        )

    @property
    def motion_planner(self) -> MotionPlanner:
        """Return the configured motion planner."""
        return self._motion_planner

    @property
    def pid_controller(self) -> PIDController:
        """Return the configured PID controller."""
        return self._pid_controller

    def move_to_pose(
        self,
        target_pos: np.ndarray,
        target_orient: Optional[np.ndarray] = None,
        duration: float = 4.0,
    ) -> None:
        """Move end-effector to target pose via planning and PID control."""
        if not isinstance(target_pos, np.ndarray) or target_pos.shape != (3,):
            raise ValueError("Target position must be a 3D numpy array")
        if duration <= 0:
            raise ValueError("Duration must be positive")
        if target_orient is not None:
            if not isinstance(target_orient, np.ndarray):
                raise ValueError(
                    "Target orientation must be a quaternion [w, x, y, z]"
                )
            if target_orient.shape != (4,):
                raise ValueError(
                    "Target orientation must be a quaternion [w, x, y, z]"
                )

        end_effector = getattr(self.robot, "end_effector_name", None)
        if end_effector is None:
            raise RuntimeError("Robot end effector name is not configured")

        current_ee_pos, current_ee_quat = self.robot.get_body_pose(
            end_effector
        )

        trajectory, _ = self.motion_planner.plan_trajectory(
            current_ee_pos,
            target_pos,
            start_orient=current_ee_quat,
            end_orient=target_orient,
            duration=duration,
        )

        if trajectory is None:
            raise RuntimeError("Failed to plan trajectory")

        current_joint_positions = self.robot.get_robot_joint_positions()
        if current_joint_positions is None:
            raise RuntimeError("Robot joint positions are unavailable")

        # Ensure we operate on a mutable copy to avoid side-effects on slices
        joint_positions = np.array(current_joint_positions, dtype=float)

        executed_steps = 0
        object_geom = getattr(
            getattr(self.robot, "gripper", None), "object_geom", None
        )
        collision_checker = getattr(self.robot, "collision_checker", None)
        viewer = getattr(self.robot, "viewer", None)

        for target in trajectory:
            control_signal = self.pid_controller.compute(
                target,
                joint_positions,
            )
            joint_positions[: control_signal.shape[0]] += (
                control_signal * self.dt
            )

            # Sync back to robot state
            if hasattr(self.robot, "set_robot_joint_positions"):
                self.robot.set_robot_joint_positions(joint_positions)

            if viewer is not None:
                viewer.step(joint_positions)

            time.sleep(self.dt)
            executed_steps += 1

            if object_geom and collision_checker is not None:
                try:
                    contact_detected = collision_checker.detect_contact(
                        end_effector, object_geom
                    )
                except AttributeError:  # detect_contact may not exist
                    contact_detected = False
                if contact_detected:
                    LOGGER.info(
                        "Contact detected with %s, stopping movement.",
                        object_geom,
                    )
                    break

        LOGGER.debug(
            "Executed %d control steps to reach target position.",
            executed_steps,
        )

    def _create_default_motion_planner(self) -> MotionPlanner:
        """Build a default motion planner from the robot configuration."""
        if not hasattr(self.robot, "model") or not hasattr(self.robot, "data"):
            raise RuntimeError(
                "Cannot create default motion planner without robot model"
                " and data"
            )

        return MotionPlanner(
            model=self.robot.model,
            data=self.robot.data,
            end_effector_name=getattr(self.robot, "end_effector_name", None),
            end_effector_type=getattr(self.robot, "end_effector_type", None),
            gripper_bodies=getattr(self.robot, "gripper_bodies", None),
            obstacles=getattr(self.robot, "obstacles", None),
            dt=float(self.rates.planner),
        )

    def _create_default_pid_controller(self) -> PIDController:
        """Create a basic PID controller tuned for position control."""
        joint_positions = self.robot.get_robot_joint_positions()
        if joint_positions is None:
            signal_dim = 6
        else:
            signal_dim = len(joint_positions)

        gains = np.ones(signal_dim)
        return PIDController(
            kp=gains * 2.0,
            ki=gains * 0.1,
            kd=gains * 0.5,
            dt=self.dt,
            signal_dim=signal_dim,
        )


__all__ = ["MotionExecutor"]
