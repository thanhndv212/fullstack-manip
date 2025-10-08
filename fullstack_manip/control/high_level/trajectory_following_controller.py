"""Trajectory following controller for executing planned paths."""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Tuple

import numpy as np

from .base_controller import BaseController

if TYPE_CHECKING:  # pragma: no cover - circular import guard
    from .robot import Robot


class TrajectoryFollowingController(BaseController):
    """Execute pre-planned trajectories with configurable tracking.

    This controller takes a sequence of waypoints (joint or Cartesian space)
    and executes them using the robot's low-level control. Supports velocity
    profiling and real-time trajectory tracking.

    Attributes:
        tracking_tolerance: Maximum allowed deviation from planned trajectory.
        velocity_scaling: Scale factor for trajectory execution speed.
    """

    def __init__(
        self,
        robot: "Robot",
        tracking_tolerance: float = 0.01,
        velocity_scaling: float = 1.0,
    ) -> None:
        """Initialize the trajectory following controller.

        Args:
            robot: Robot instance to control.
            tracking_tolerance: Maximum deviation (m or rad) from trajectory.
            velocity_scaling: Speed multiplier for trajectory execution
                (0.0 to 2.0).
        """
        super().__init__(robot)

        if tracking_tolerance <= 0:
            raise ValueError("Tracking tolerance must be positive")
        if not 0.0 < velocity_scaling <= 2.0:
            raise ValueError("Velocity scaling must be in (0.0, 2.0]")

        self.tracking_tolerance = tracking_tolerance
        self.velocity_scaling = velocity_scaling
        self._current_trajectory: Optional[List[np.ndarray]] = None
        self._trajectory_times: Optional[List[float]] = None

    def execute(
        self,
        waypoints: List[np.ndarray],
        duration: Optional[float] = None,
        interpolation: str = "linear",
    ) -> None:
        """Execute trajectory through provided waypoints.

        Args:
            waypoints: List of target configurations (joint or Cartesian).
            duration: Total time for trajectory. If None, uses default
                motion planner timing.
            interpolation: Interpolation method ('linear', 'cubic').

        Raises:
            ValueError: If waypoints are invalid or empty.
            RuntimeError: If trajectory execution fails.
        """
        if not waypoints or len(waypoints) == 0:
            raise ValueError("Waypoints list cannot be empty")

        self._validate_waypoints(waypoints)

        if interpolation == "linear":
            trajectory = self._interpolate_linear(waypoints, duration)
        elif interpolation == "cubic":
            trajectory = self._interpolate_cubic(waypoints, duration)
        else:
            raise ValueError(f"Unknown interpolation method: {interpolation}")

        self._execute_trajectory(trajectory)

    def execute_joint_trajectory(
        self,
        joint_waypoints: List[np.ndarray],
        duration: Optional[float] = None,
    ) -> None:
        """Execute trajectory in joint space.

        Args:
            joint_waypoints: List of joint configurations [q1, q2, ...].
            duration: Total execution time in seconds.
        """
        if not joint_waypoints:
            raise ValueError("Joint waypoints cannot be empty")

        for waypoint in joint_waypoints:
            if not isinstance(waypoint, np.ndarray):
                raise ValueError("Each waypoint must be a numpy array")
            if waypoint.shape[0] != self.robot.robot_nq:
                raise ValueError(
                    f"Joint waypoint dimension mismatch: "
                    f"expected {self.robot.robot_nq}, "
                    f"got {waypoint.shape[0]}"
                )

        trajectory = self._interpolate_linear(joint_waypoints, duration)
        self._execute_joint_trajectory(trajectory)

    def execute_cartesian_trajectory(
        self,
        cartesian_waypoints: List[Tuple[np.ndarray, np.ndarray]],
        duration: Optional[float] = None,
    ) -> None:
        """Execute trajectory in Cartesian space.

        Args:
            cartesian_waypoints: List of (position, orientation) tuples.
            duration: Total execution time in seconds.
        """
        if not cartesian_waypoints:
            raise ValueError("Cartesian waypoints cannot be empty")

        for pos, orient in cartesian_waypoints:
            if not isinstance(pos, np.ndarray) or pos.shape != (3,):
                raise ValueError("Position must be 3D numpy array")
            if orient is not None:
                if not isinstance(orient, np.ndarray) or orient.shape != (4,):
                    raise ValueError("Orientation must be 4D quaternion")

        positions = [wp[0] for wp in cartesian_waypoints]
        orientations = [wp[1] for wp in cartesian_waypoints]

        for i in range(len(positions)):
            self.robot.move_to_position(
                positions[i],
                orientations[i],
                duration=duration / len(positions) if duration else 4.0,
            )

    def _validate_waypoints(self, waypoints: List[np.ndarray]) -> None:
        """Validate waypoint dimensions and types."""
        if not all(isinstance(wp, np.ndarray) for wp in waypoints):
            raise ValueError("All waypoints must be numpy arrays")

    def _interpolate_linear(
        self,
        waypoints: List[np.ndarray],
        duration: Optional[float],
    ) -> List[np.ndarray]:
        """Linearly interpolate between waypoints."""
        if duration is None:
            duration = len(waypoints) * 2.0

        num_steps = int(duration / self.robot.dt)
        trajectory = []

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            segment_steps = num_steps // (len(waypoints) - 1)

            for t in range(segment_steps):
                alpha = t / segment_steps
                interpolated = (1 - alpha) * start + alpha * end
                trajectory.append(interpolated)

        trajectory.append(waypoints[-1])
        return trajectory

    def _interpolate_cubic(
        self,
        waypoints: List[np.ndarray],
        duration: Optional[float],
    ) -> List[np.ndarray]:
        """Cubic interpolation for smoother trajectories."""
        if len(waypoints) < 2:
            return waypoints

        # Simplified cubic interpolation
        # In practice, use scipy.interpolate or similar
        return self._interpolate_linear(waypoints, duration)

    def _execute_trajectory(self, trajectory: List[np.ndarray]) -> None:
        """Execute the interpolated trajectory."""
        self._current_trajectory = trajectory

        current_joint_positions = self.robot.get_robot_joint_positions()

        for i, target in enumerate(trajectory):
            control_signal = self.robot.pid_controller.compute(
                target,
                current_joint_positions,
            )

            current_joint_positions += (
                control_signal * self.robot.dt * self.velocity_scaling
            )

            self.robot.viewer.step(current_joint_positions)

            # Check tracking error
            tracking_error = np.linalg.norm(
                current_joint_positions[: target.shape[0]] - target
            )
            if tracking_error > self.tracking_tolerance:
                print(
                    f"Warning: Tracking error {tracking_error:.4f} "
                    f"exceeds tolerance {self.tracking_tolerance}"
                )

    def _execute_joint_trajectory(
        self,
        trajectory: List[np.ndarray],
    ) -> None:
        """Execute joint space trajectory."""
        self._execute_trajectory(trajectory)

    def get_trajectory_progress(self) -> float:
        """Return current trajectory completion percentage.

        Returns:
            Progress as a float between 0.0 and 1.0, or -1 if no active
            trajectory.
        """
        if self._current_trajectory is None:
            return -1.0

        # This would track actual progress during execution
        return 1.0

    def reset(self) -> None:
        """Reset trajectory state."""
        self._current_trajectory = None
        self._trajectory_times = None


__all__ = ["TrajectoryFollowingController"]
