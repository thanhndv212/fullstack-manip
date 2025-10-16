"""Admittance controller for force-guided motion and teaching."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import numpy as np

from .base_controller import BaseController

if TYPE_CHECKING:  # pragma: no cover - circular import guard
    from ...core.robot import Robot


class AdmittanceController(BaseController):
    """Motion control based on external force feedback.

    Admittance control is the dual of impedance control: it maps
    measured forces/torques to desired motion. Useful for human
    guidance, teleoperation, and kinesthetic teaching.

    Attributes:
        mass: Virtual mass matrix (6x6) [kg, kg*m^2].
        damping: Virtual damping matrix (6x6) [Ns/m, Nms/rad].
        stiffness: Virtual stiffness matrix (6x6) [N/m, Nm/rad].
    """

    def __init__(
        self,
        robot: "Robot",
        mass: Optional[np.ndarray] = None,
        damping: Optional[np.ndarray] = None,
        stiffness: Optional[np.ndarray] = None,
    ) -> None:
        """Initialize the admittance controller.

        Args:
            robot: Robot instance to control.
            mass: 6x6 virtual mass matrix or scalar. If None, uses
                default light mass for responsive motion.
            damping: 6x6 virtual damping matrix or scalar. If None, uses
                moderate damping.
            stiffness: 6x6 virtual stiffness matrix or scalar. If None,
                uses zero stiffness (pure admittance).
        """
        super().__init__(robot)

        self.mass = self._process_admittance_param(mass, default_value=5.0)
        self.damping = self._process_admittance_param(
            damping, default_value=20.0
        )
        self.stiffness = self._process_admittance_param(
            stiffness, default_value=0.0
        )

        self._validate_admittance_matrices()

        # State variables
        self._velocity = np.zeros(6)
        self._position = np.zeros(6)
        self._reference_pos: Optional[np.ndarray] = None

    def execute(
        self,
        duration: float = 10.0,
        force_threshold: float = 1.0,
    ) -> None:
        """Execute admittance-controlled motion following applied forces.

        Args:
            duration: Duration to run admittance control (seconds).
            force_threshold: Minimum force magnitude to trigger motion (N).
        """
        if duration <= 0:
            raise ValueError("Duration must be positive")
        if force_threshold < 0:
            raise ValueError("Force threshold must be non-negative")

        self._initialize_state()
        self._run_admittance_control(duration, force_threshold)

    def follow_force(
        self,
        force_source,
        duration: float = 10.0,
        velocity_limit: float = 0.1,
    ) -> None:
        """Follow applied forces from external source.

        Args:
            force_source: Callable that returns current applied force
                vector [Fx, Fy, Fz, Tx, Ty, Tz].
            duration: Duration to follow forces (seconds).
            velocity_limit: Maximum allowed velocity (m/s or rad/s).
        """
        if velocity_limit <= 0:
            raise ValueError("Velocity limit must be positive")

        self._initialize_state()
        steps = int(duration / self.robot.dt)

        for _ in range(steps):
            external_force = force_source()

            if external_force is None:
                continue

            if not isinstance(external_force, np.ndarray):
                continue
            if external_force.shape != (6,):
                continue

            # Compute admittance response
            acceleration = self._compute_acceleration(external_force)
            self._velocity += acceleration * self.robot.dt

            # Apply velocity limits
            velocity_magnitude = np.linalg.norm(self._velocity[:3])
            if velocity_magnitude > velocity_limit:
                self._velocity[:3] *= velocity_limit / velocity_magnitude

            # Update position
            self._position += self._velocity * self.robot.dt

            # Apply to robot
            self._apply_position_update()

    def backdrive_motion(
        self,
        duration: float = 5.0,
        sensitivity: float = 0.5,
    ) -> list[tuple[np.ndarray, np.ndarray]]:
        """Enable backdrivable mode for kinesthetic teaching.

        Args:
            duration: Duration to record motion (seconds).
            sensitivity: Motion sensitivity (0.0 to 1.0).

        Returns:
            List of recorded (position, orientation) tuples.
        """
        if not 0.0 < sensitivity <= 1.0:
            raise ValueError("Sensitivity must be in (0, 1]")

        # Reduce damping for easier backdriving
        original_damping = self.damping.copy()
        self.damping = self.damping * sensitivity

        recorded_trajectory = []

        try:
            self._initialize_state()
            steps = int(duration / self.robot.dt)

            for _ in range(steps):
                external_force = self._measure_external_force()
                acceleration = self._compute_acceleration(external_force)

                self._velocity += acceleration * self.robot.dt
                self._position += self._velocity * self.robot.dt

                self._apply_position_update()

                # Record current pose
                pos, orient = self.robot.get_body_pose(
                    self.robot.end_effector_name
                )
                recorded_trajectory.append((pos.copy(), orient.copy()))

        finally:
            self.damping = original_damping

        return recorded_trajectory

    def kinesthetic_teaching(
        self,
        duration: float = 10.0,
    ) -> list[np.ndarray]:
        """Record demonstration through physical guidance.

        Args:
            duration: Duration to record demonstration (seconds).

        Returns:
            List of recorded joint configurations.
        """
        self._initialize_state()
        recorded_joints = []

        steps = int(duration / self.robot.dt)

        for _ in range(steps):
            external_force = self._measure_external_force()
            acceleration = self._compute_acceleration(external_force)

            self._velocity += acceleration * self.robot.dt
            self._position += self._velocity * self.robot.dt

            self._apply_position_update()

            # Record current joint configuration
            joint_pos = self.robot.get_robot_joint_positions()
            recorded_joints.append(joint_pos.copy())

        return recorded_joints

    def _process_admittance_param(
        self,
        param: Optional[np.ndarray | float],
        default_value: float = 1.0,
    ) -> np.ndarray:
        """Process admittance parameter into 6x6 matrix.

        Args:
            param: Scalar or 6x6 matrix.
            default_value: Default value if param is None.

        Returns:
            6x6 admittance matrix.
        """
        if param is None:
            return np.eye(6) * default_value

        if isinstance(param, (int, float)):
            return np.eye(6) * float(param)

        if isinstance(param, np.ndarray):
            if param.shape == (6, 6):
                return param
            if param.shape == (6,):
                return np.diag(param)
            raise ValueError("Admittance parameter must be 6x6 or 6D")

        raise TypeError("Admittance parameter must be scalar or array")

    def _validate_admittance_matrices(self) -> None:
        """Validate that admittance matrices are valid."""
        for name, matrix in [
            ("mass", self.mass),
            ("damping", self.damping),
            ("stiffness", self.stiffness),
        ]:
            if matrix.shape != (6, 6):
                raise ValueError(f"{name} must be 6x6 matrix")

            # Mass and damping must be positive
            if name in ["mass", "damping"]:
                if np.any(np.diag(matrix) <= 0):
                    raise ValueError(f"{name} must be positive")

            # Stiffness can be zero or positive
            if name == "stiffness":
                if np.any(np.diag(matrix) < 0):
                    raise ValueError(f"{name} must be non-negative")

    def _initialize_state(self) -> None:
        """Initialize admittance controller state."""
        current_pos, current_orient = self.robot.get_body_pose(
            self.robot.end_effector_name
        )

        self._reference_pos = current_pos.copy()
        self._position[:3] = current_pos
        self._velocity = np.zeros(6)

    def _run_admittance_control(
        self,
        duration: float,
        force_threshold: float,
    ) -> None:
        """Run admittance control loop.

        Args:
            duration: Duration to run (seconds).
            force_threshold: Minimum force to trigger motion.
        """
        steps = int(duration / self.robot.dt)

        for _ in range(steps):
            external_force = self._measure_external_force()
            force_magnitude = np.linalg.norm(external_force[:3])

            if force_magnitude < force_threshold:
                # Apply damping to slow down
                self._velocity *= 0.95
                continue

            acceleration = self._compute_acceleration(external_force)
            self._velocity += acceleration * self.robot.dt
            self._position += self._velocity * self.robot.dt

            self._apply_position_update()

    def _measure_external_force(self) -> np.ndarray:
        """Measure external forces/torques on end-effector.

        Returns:
            6D force/torque vector [Fx, Fy, Fz, Tx, Ty, Tz].
        """
        # Placeholder - would integrate with force/torque sensor
        # or estimate from joint torques in simulation
        external_wrench = np.zeros(6)

        # In MuJoCo, could compute from contact forces or
        # sensor readings
        for contact in self.robot.data.contact[: self.robot.data.ncon]:
            external_wrench[:3] += contact.frame[:3]

        return external_wrench

    def _compute_acceleration(
        self,
        external_force: np.ndarray,
    ) -> np.ndarray:
        """Compute acceleration from external force.

        Admittance: M*ddx + B*dx + K*x = F_ext
        Solve for: ddx = M^-1 * (F_ext - B*dx - K*x)

        Args:
            external_force: External wrench [Fx, Fy, Fz, Tx, Ty, Tz].

        Returns:
            Acceleration vector [ax, ay, az, alpha_x, alpha_y, alpha_z].
        """
        # Compute restoring force from stiffness
        position_error = (
            self._position - np.concatenate([self._reference_pos, np.zeros(3)])
            if self._reference_pos is not None
            else np.zeros(6)
        )

        stiffness_force = self.stiffness @ position_error
        damping_force = self.damping @ self._velocity

        net_force = external_force - damping_force - stiffness_force

        try:
            acceleration = np.linalg.solve(self.mass, net_force)
        except np.linalg.LinAlgError:
            acceleration = np.zeros(6)

        return acceleration

    def _apply_position_update(self) -> None:
        """Apply computed position update to robot."""
        target_pos = self._position[:3]

        current_pos, current_orient = self.robot.get_body_pose(
            self.robot.end_effector_name
        )

        # Apply small step towards target
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction)

        if distance > 0.001:
            step_size = min(distance, 0.01)
            next_pos = current_pos + (direction / distance) * step_size

            try:
                self.move_to_pose(
                    next_pos,
                    current_orient,
                    duration=self.robot.dt,
                )
            except RuntimeError:
                pass

    def reset(self) -> None:
        """Reset admittance controller state."""
        self._velocity = np.zeros(6)
        self._position = np.zeros(6)
        self._reference_pos = None


__all__ = ["AdmittanceController"]
