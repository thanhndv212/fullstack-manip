"""Gravity compensation controller for teaching and manual guidance."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import numpy as np

from .base_controller import BaseController

if TYPE_CHECKING:  # pragma: no cover - circular import guard
    from .robot import Robot


class GravityCompensationController(BaseController):
    """Compensate for gravity to enable easy manual manipulation.

    This controller applies joint torques to cancel gravitational
    effects, making the robot feel weightless and easy to move manually.
    Useful for kinesthetic teaching and manual positioning.

    Attributes:
        compensation_gain: Gain for gravity compensation (0.0 to 1.5).
        enable_friction_compensation: Whether to compensate friction.
    """

    def __init__(
        self,
        robot: "Robot",
        compensation_gain: float = 1.0,
        enable_friction_compensation: bool = False,
    ) -> None:
        """Initialize the gravity compensation controller.

        Args:
            robot: Robot instance to control.
            compensation_gain: Gravity compensation gain. 1.0 = full
                compensation, < 1.0 = partial (feels heavier),
                > 1.0 = over-compensation (feels lighter).
            enable_friction_compensation: If True, also compensate for
                friction in joints.
        """
        super().__init__(robot)

        if compensation_gain < 0:
            raise ValueError("Compensation gain must be non-negative")
        if compensation_gain > 1.5:
            raise ValueError("Compensation gain should not exceed 1.5")

        self.compensation_gain = compensation_gain
        self.enable_friction_compensation = enable_friction_compensation

        # Friction parameters (would be identified from robot)
        self._friction_coefficients = np.ones(robot.robot_nq) * 0.1

    def execute(
        self,
        duration: float = 10.0,
        record_trajectory: bool = False,
    ) -> Optional[list[np.ndarray]]:
        """Enable gravity compensation for specified duration.

        Args:
            duration: Duration to maintain compensation (seconds).
            record_trajectory: If True, records joint positions during
                compensation.

        Returns:
            List of recorded joint configurations if record_trajectory
            is True, None otherwise.
        """
        if duration <= 0:
            raise ValueError("Duration must be positive")

        trajectory = [] if record_trajectory else None

        steps = int(duration / self.robot.dt)

        for _ in range(steps):
            # Compute gravity compensation torques
            gravity_torques = self._compute_gravity_torques()

            # Apply compensation
            self._apply_compensation_torques(gravity_torques)

            if record_trajectory:
                joint_pos = self.robot.get_robot_joint_positions()
                trajectory.append(joint_pos.copy())

        return trajectory

    def compensate_and_teach(
        self,
        duration: float = 10.0,
    ) -> list[tuple[np.ndarray, np.ndarray]]:
        """Enable gravity compensation and record taught trajectory.

        Args:
            duration: Duration to record demonstration (seconds).

        Returns:
            List of (joint_positions, end_effector_pose) tuples.
        """
        trajectory = []
        steps = int(duration / self.robot.dt)

        print("Gravity compensation enabled. Manually move the robot...")

        for _ in range(steps):
            gravity_torques = self._compute_gravity_torques()
            self._apply_compensation_torques(gravity_torques)

            # Record state
            joint_pos = self.robot.get_robot_joint_positions()
            ee_pos, ee_orient = self.robot.get_body_pose(
                self.robot.end_effector_name
            )

            trajectory.append(
                (joint_pos.copy(), np.concatenate([ee_pos, ee_orient]))
            )

        print(f"Recorded {len(trajectory)} waypoints")
        return trajectory

    def hold_position(
        self,
        target_joints: Optional[np.ndarray] = None,
        duration: float = 5.0,
        stiffness: float = 0.1,
    ) -> None:
        """Hold current or specified position with gravity compensation.

        Args:
            target_joints: Joint configuration to hold. If None, holds
                current configuration.
            duration: Duration to hold position (seconds).
            stiffness: Position holding stiffness (0.0 to 1.0).
        """
        if target_joints is None:
            target_joints = self.robot.get_robot_joint_positions()
        else:
            if not isinstance(target_joints, np.ndarray):
                raise ValueError("Target joints must be numpy array")
            if target_joints.shape[0] != self.robot.robot_nq:
                raise ValueError(
                    f"Target joints dimension mismatch: "
                    f"expected {self.robot.robot_nq}, "
                    f"got {target_joints.shape[0]}"
                )

        if not 0.0 <= stiffness <= 1.0:
            raise ValueError("Stiffness must be in [0, 1]")

        steps = int(duration / self.robot.dt)

        for _ in range(steps):
            current_joints = self.robot.get_robot_joint_positions()

            # Gravity compensation
            gravity_torques = self._compute_gravity_torques()

            # Position holding torques
            position_error = target_joints - current_joints
            holding_torques = stiffness * position_error * 100.0

            total_torques = gravity_torques + holding_torques
            self._apply_compensation_torques(total_torques)

    def set_compensation_gain(self, gain: float) -> None:
        """Adjust gravity compensation gain.

        Args:
            gain: New compensation gain (0.0 to 1.5).
        """
        if gain < 0 or gain > 1.5:
            raise ValueError("Gain must be in [0, 1.5]")
        self.compensation_gain = gain

    def enable_friction_compensation(self, enable: bool = True) -> None:
        """Enable or disable friction compensation.

        Args:
            enable: Whether to enable friction compensation.
        """
        self.enable_friction_compensation = enable

    def _compute_gravity_torques(self) -> np.ndarray:
        """Compute joint torques to compensate for gravity.

        Returns:
            Joint torques for gravity compensation.
        """
        import mujoco

        # MuJoCo provides gravity compensation through qfrc_bias
        # which includes gravity and Coriolis/centrifugal forces
        mujoco.mj_forward(self.robot.model, self.robot.data)

        # Get gravity component (passive forces)
        gravity_torques = self.robot.data.qfrc_bias[: self.robot.robot_nq]

        # Apply compensation gain
        compensated_torques = gravity_torques * self.compensation_gain

        # Add friction compensation if enabled
        if self.enable_friction_compensation:
            friction_torques = self._compute_friction_torques()
            compensated_torques += friction_torques

        return compensated_torques

    def _compute_friction_torques(self) -> np.ndarray:
        """Compute joint torques to compensate for friction.

        Returns:
            Joint torques for friction compensation.
        """
        # Get joint velocities
        joint_velocities = self.robot.data.qvel[: self.robot.robot_nq]

        # Simple viscous + Coulomb friction model
        # tau_friction = B * dq + sign(dq) * tau_coulomb
        viscous_friction = self._friction_coefficients * joint_velocities

        coulomb_friction = (
            np.sign(joint_velocities) * self._friction_coefficients * 0.5
        )

        return -(viscous_friction + coulomb_friction)

    def _apply_compensation_torques(self, torques: np.ndarray) -> None:
        """Apply compensation torques to robot.

        Args:
            torques: Joint torques to apply.
        """
        # In simulation, apply torques through control input
        if hasattr(self.robot.data, "ctrl"):
            self.robot.data.ctrl[: self.robot.robot_nq] = torques

        # Step simulation
        import mujoco

        mujoco.mj_step(self.robot.model, self.robot.data)

        # Update viewer if available
        if hasattr(self.robot, "viewer") and self.robot.viewer is not None:
            self.robot.viewer.sync()

    def calibrate_friction(
        self,
        duration: float = 5.0,
        slow_motion_velocity: float = 0.1,
    ) -> np.ndarray:
        """Calibrate friction parameters through slow motion.

        Args:
            duration: Duration for calibration (seconds).
            slow_motion_velocity: Target velocity for calibration (rad/s).

        Returns:
            Identified friction coefficients for each joint.
        """
        print("Starting friction calibration...")
        print("Slowly move each joint back and forth")

        steps = int(duration / self.robot.dt)
        torque_samples = []
        velocity_samples = []

        for _ in range(steps):
            gravity_torques = self._compute_gravity_torques()
            self._apply_compensation_torques(gravity_torques)

            joint_velocities = self.robot.data.qvel[: self.robot.robot_nq]
            measured_torques = self.robot.data.qfrc_applied[
                : self.robot.robot_nq
            ]

            torque_samples.append(measured_torques.copy())
            velocity_samples.append(joint_velocities.copy())

        # Simple linear regression to identify friction
        # (in practice, use more sophisticated identification)
        if torque_samples:
            torque_array = np.array(torque_samples)
            velocity_array = np.array(velocity_samples)

            # Avoid division by zero
            velocity_array = np.where(
                np.abs(velocity_array) < 1e-6, 1e-6, velocity_array
            )

            self._friction_coefficients = np.mean(
                np.abs(torque_array / velocity_array), axis=0
            )

        print(
            f"Calibrated friction coefficients: "
            f"{self._friction_coefficients}"
        )
        return self._friction_coefficients

    def get_compensation_status(self) -> dict:
        """Get current compensation status.

        Returns:
            Dictionary with compensation parameters and state.
        """
        return {
            "compensation_gain": self.compensation_gain,
            "friction_compensation_enabled": (
                self.enable_friction_compensation
            ),
            "friction_coefficients": self._friction_coefficients.tolist(),
        }

    def reset(self) -> None:
        """Reset gravity compensation controller state."""
        pass


__all__ = ["GravityCompensationController"]
