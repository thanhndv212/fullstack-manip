"""Impedance controller for compliant and safe robot interaction."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import numpy as np

from .base_controller import BaseController

if TYPE_CHECKING:  # pragma: no cover - circular import guard
    from .robot import Robot


class ImpedanceController(BaseController):
    """Variable impedance control for safe human-robot interaction.

    Implements mechanical impedance control where the robot behaves like
    a mass-spring-damper system. Allows compliant motion and safe
    interaction in uncertain or collaborative environments.

    Attributes:
        stiffness: Cartesian stiffness matrix (6x6) [N/m, Nm/rad].
        damping: Cartesian damping matrix (6x6) [Ns/m, Nms/rad].
        inertia: Cartesian inertia matrix (6x6) [kg, kg*m^2].
    """

    def __init__(
        self,
        robot: "Robot",
        stiffness: Optional[np.ndarray] = None,
        damping: Optional[np.ndarray] = None,
        inertia: Optional[np.ndarray] = None,
    ) -> None:
        """Initialize the impedance controller.

        Args:
            robot: Robot instance to control.
            stiffness: 6x6 stiffness matrix or scalar for uniform
                stiffness. If None, uses default medium stiffness.
            damping: 6x6 damping matrix or scalar. If None, uses
                critical damping.
            inertia: 6x6 inertia matrix or scalar. If None, uses default.
        """
        super().__init__(robot)

        # Set default or validate impedance parameters
        self.stiffness = self._process_impedance_param(
            stiffness, default_value=500.0
        )
        self.damping = self._process_impedance_param(
            damping, default_value=50.0
        )
        self.inertia = self._process_impedance_param(
            inertia, default_value=1.0
        )

        self._validate_impedance_matrices()

        # Desired equilibrium pose
        self._desired_pos: Optional[np.ndarray] = None
        self._desired_orient: Optional[np.ndarray] = None

    def execute(
        self,
        desired_pos: np.ndarray,
        desired_orient: Optional[np.ndarray] = None,
        duration: float = 5.0,
    ) -> None:
        """Execute impedance-controlled motion to desired pose.

        Args:
            desired_pos: Desired equilibrium position [x, y, z].
            desired_orient: Desired equilibrium orientation (quaternion).
            duration: Duration to maintain impedance (seconds).
        """
        if not isinstance(desired_pos, np.ndarray):
            raise ValueError("Desired position must be numpy array")
        if desired_pos.shape != (3,):
            raise ValueError("Desired position must be 3D")

        self._desired_pos = desired_pos
        self._desired_orient = desired_orient

        self._run_impedance_control(duration)

    def set_stiffness(
        self,
        stiffness: np.ndarray | float,
        axis: Optional[str] = None,
    ) -> None:
        """Set impedance stiffness parameters.

        Args:
            stiffness: Stiffness value or 6x6 matrix.
            axis: If specified, sets stiffness for single axis
                ('x', 'y', 'z', 'rx', 'ry', 'rz').
        """
        if axis is not None:
            axis_map = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}
            if axis not in axis_map:
                raise ValueError(f"Invalid axis: {axis}")

            idx = axis_map[axis]
            if isinstance(stiffness, (int, float)):
                self.stiffness[idx, idx] = float(stiffness)
            else:
                raise ValueError("Stiffness must be scalar for axis mode")
        else:
            self.stiffness = self._process_impedance_param(stiffness)

        self._validate_impedance_matrices()

    def set_damping(
        self,
        damping: np.ndarray | float,
        axis: Optional[str] = None,
    ) -> None:
        """Set impedance damping parameters.

        Args:
            damping: Damping value or 6x6 matrix.
            axis: If specified, sets damping for single axis.
        """
        if axis is not None:
            axis_map = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}
            if axis not in axis_map:
                raise ValueError(f"Invalid axis: {axis}")

            idx = axis_map[axis]
            if isinstance(damping, (int, float)):
                self.damping[idx, idx] = float(damping)
            else:
                raise ValueError("Damping must be scalar for axis mode")
        else:
            self.damping = self._process_impedance_param(damping)

        self._validate_impedance_matrices()

    def compliant_approach(
        self,
        target_pos: np.ndarray,
        low_stiffness: float = 100.0,
    ) -> None:
        """Approach target with low stiffness for safety.

        Args:
            target_pos: Target position to approach [x, y, z].
            low_stiffness: Reduced stiffness value for compliance.
        """
        if not isinstance(target_pos, np.ndarray):
            raise ValueError("Target position must be numpy array")
        if target_pos.shape != (3,):
            raise ValueError("Target position must be 3D")

        # Store original stiffness
        original_stiffness = self.stiffness.copy()

        # Set low stiffness for compliant motion
        self.set_stiffness(low_stiffness)

        try:
            self.execute(target_pos, duration=3.0)
        finally:
            # Restore original stiffness
            self.stiffness = original_stiffness

    def absorb_impact(
        self,
        duration: float = 2.0,
        damping_ratio: float = 2.0,
    ) -> None:
        """Increase damping to absorb unexpected impacts.

        Args:
            duration: Duration to maintain high damping (seconds).
            damping_ratio: Multiplier for current damping values.
        """
        if damping_ratio <= 0:
            raise ValueError("Damping ratio must be positive")

        original_damping = self.damping.copy()
        self.damping = self.damping * damping_ratio

        try:
            current_pos, current_orient = self.robot.get_body_pose(
                self.robot.end_effector_name
            )
            self.execute(current_pos, current_orient, duration=duration)
        finally:
            self.damping = original_damping

    def _process_impedance_param(
        self,
        param: Optional[np.ndarray | float],
        default_value: float = 1.0,
    ) -> np.ndarray:
        """Process impedance parameter into 6x6 matrix.

        Args:
            param: Scalar or 6x6 matrix.
            default_value: Default value if param is None.

        Returns:
            6x6 impedance matrix.
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
            raise ValueError("Impedance parameter must be 6x6 or 6D array")

        raise TypeError("Impedance parameter must be scalar or array")

    def _validate_impedance_matrices(self) -> None:
        """Validate that impedance matrices are positive definite."""
        for name, matrix in [
            ("stiffness", self.stiffness),
            ("damping", self.damping),
            ("inertia", self.inertia),
        ]:
            if matrix.shape != (6, 6):
                raise ValueError(f"{name} must be 6x6 matrix")

            # Check positive definiteness (simplified - check diagonal)
            if np.any(np.diag(matrix) <= 0):
                raise ValueError(f"{name} matrix must be positive definite")

    def _run_impedance_control(self, duration: float) -> None:
        """Run impedance control loop for specified duration.

        Args:
            duration: Duration to run control loop (seconds).
        """
        steps = int(duration / self.robot.dt)

        for _ in range(steps):
            # Get current state
            current_pos, current_orient = self.robot.get_body_pose(
                self.robot.end_effector_name
            )

            # Compute pose error
            pos_error = self._desired_pos - current_pos

            # Compute impedance force: F = K*x + B*dx + M*ddx
            # Simplified: only proportional term
            impedance_force = self.stiffness[:3, :3] @ pos_error

            # Convert force to motion (simplified)
            control_velocity = impedance_force * self.robot.dt

            # Apply control
            new_pos = current_pos + control_velocity
            self.robot.move_to_position(
                new_pos,
                (
                    self._desired_orient
                    if self._desired_orient is not None
                    else current_orient
                ),
                duration=self.robot.dt,
            )

    def get_impedance_parameters(self) -> dict:
        """Return current impedance parameters.

        Returns:
            Dictionary with stiffness, damping, and inertia matrices.
        """
        return {
            "stiffness": self.stiffness.copy(),
            "damping": self.damping.copy(),
            "inertia": self.inertia.copy(),
        }

    def reset(self) -> None:
        """Reset impedance controller state."""
        self._desired_pos = None
        self._desired_orient = None


__all__ = ["ImpedanceController"]
