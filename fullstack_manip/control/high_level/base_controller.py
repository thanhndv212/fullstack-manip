"""Base controller class for high-level robot behaviors."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Dict, Optional

import numpy as np

from ..motion_executor import MotionExecutor
from ...utils.rate_config import RateConfig

if TYPE_CHECKING:  # pragma: no cover - circular import guard
    from ...core.robot import Robot


class BaseController(ABC):
    """Abstract base class for all high-level robot controllers.

    This class defines the common interface and shared functionality that all
    high-level controllers (pick-place, trajectory following, etc.) should
    implement or inherit.

    Attributes:
        robot: The Robot instance this controller operates on.
        reach_threshold: Maximum distance (m) for considering a target reached.
    """

    def __init__(
        self,
        robot: "Robot",
        reach_threshold: Optional[np.ndarray] = None,
        motion_executor: Optional[MotionExecutor] = None,
    ) -> None:
        """Initialize the base controller.

        Args:
            robot: Robot instance to control.
            reach_threshold: 3D array defining reach tolerances [x, y, z].
                Defaults to [0.1, 0.1, 0.1] meters.
            motion_executor: Optional helper responsible for planning and
                executing Cartesian motions. If None, a default executor is
                created using the provided robot instance.
        """
        if robot is None:
            raise ValueError("Robot instance must be provided")

        self.robot = robot
        self.rates = getattr(robot, "rates", RateConfig())
        self.control_dt = getattr(
            self.rates,
            "control",
            getattr(robot, "dt", 0.01),
        )
        self.reach_threshold = (
            reach_threshold
            if reach_threshold is not None
            else np.array([0.1, 0.1, 0.1])
        )

        self._validate_reach_threshold()
        self.motion_executor = (
            motion_executor
            if motion_executor is not None
            else MotionExecutor(robot)
        )
        self.dt = self.control_dt

    def _validate_reach_threshold(self) -> None:
        """Validate that reach_threshold is properly formatted."""
        if not isinstance(self.reach_threshold, np.ndarray):
            raise TypeError("reach_threshold must be a numpy array")
        if self.reach_threshold.shape != (3,):
            raise ValueError("reach_threshold must be a 3D array")
        if np.any(self.reach_threshold <= 0):
            raise ValueError("reach_threshold values must be positive")

    def is_within_reach(
        self,
        current_pos: Optional[np.ndarray] = None,
        target_pos: Optional[np.ndarray] = None,
    ) -> bool:
        """Check if a target position is within reach of the end-effector.

        Args:
            current_pos: Current position as [x, y, z]. If None, queries
                the robot's current end-effector pose.
            target_pos: Target position as [x, y, z].

        Returns:
            True if target is within reach threshold, False otherwise.

        Raises:
            ValueError: If target_pos is not a valid 3D position.
        """
        if target_pos is None:
            raise ValueError("Target position must be provided")
        if not isinstance(target_pos, np.ndarray) or target_pos.shape != (3,):
            raise ValueError("Target position must be a 3D numpy array")

        if current_pos is None:
            current_pos, _ = self.robot.get_body_pose(
                self.robot.end_effector_name
            )

        if not isinstance(current_pos, np.ndarray):
            raise ValueError("Current position must be a 3D numpy array")
        if current_pos.shape != (3,):
            raise ValueError("Current position must be a 3D numpy array")

        return np.all(np.abs(current_pos - target_pos) <= self.reach_threshold)

    def get_current_end_effector_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Retrieve current end-effector position and orientation.

        Returns:
            Tuple of (position, orientation) where position is [x, y, z]
            and orientation is a quaternion [w, x, y, z].
        """
        return self.robot.get_body_pose(self.robot.end_effector_name)

    def move_to_pose(
        self,
        target_pos: np.ndarray,
        target_orient: Optional[np.ndarray] = None,
        duration: float = 4.0,
    ) -> None:
        """Move the end-effector to a target pose.

        The method delegates to the configured :class:`MotionExecutor`
        which coordinates planning and low-level control to reach the
        desired Cartesian pose. Motion execution will stop early if
        contact is detected with a grasped object.

        Args:
            target_pos: Target position [x, y, z].
            target_orient: Target orientation as quaternion [w, x, y, z].
                If None, maintains current orientation.
            duration: Duration (seconds) for the motion. Default is 4.0s.

        Raises:
            ValueError: If the provided pose parameters are invalid.
            RuntimeError: If motion planning or execution fails.

        Example:
            >>> controller.move_to_pose(
            ...     target_pos=np.array([0.5, 0.0, 0.3]),
            ...     target_orient=None,  # maintain current orientation
            ...     duration=3.0
            ... )
        """
        try:
            self.motion_executor.move_to_pose(
                target_pos=target_pos,
                target_orient=target_orient,
                duration=duration,
            )
        except RuntimeError:
            raise
        except Exception as exc:  # pragma: no cover - defensive
            raise RuntimeError(
                f"Unexpected error during motion execution: {exc}"
            ) from exc

    def get_controller_state(self) -> Dict[str, Any]:
        """Return the current state of the controller.

        Returns:
            Dictionary containing controller state information such as
            reach threshold, robot status, etc.
        """
        ee_pos, ee_orient = self.get_current_end_effector_pose()
        return {
            "reach_threshold": self.reach_threshold.tolist(),
            "end_effector_position": ee_pos.tolist(),
            "end_effector_orientation": ee_orient.tolist(),
            "robot_joint_positions": (
                self.robot.get_robot_joint_positions().tolist()
            ),
        }

    @abstractmethod
    def execute(self, *args, **kwargs) -> None:
        """Execute the primary behavior of this controller.

        This method must be implemented by all concrete controller subclasses
        to define their specific high-level behavior.

        Args:
            *args: Positional arguments specific to the controller.
            **kwargs: Keyword arguments specific to the controller.

        Raises:
            NotImplementedError: If the subclass does not implement this
                method.
        """
        raise NotImplementedError(
            "Subclasses must implement the execute() method"
        )

    def reset(self) -> None:
        """Reset the controller to its initial state.

        Override this method in subclasses if specific reset logic is needed.
        """
        pass

    def __repr__(self) -> str:
        """Return string representation of the controller."""
        return (
            f"{self.__class__.__name__}("
            f"reach_threshold={self.reach_threshold.tolist()})"
        )


__all__ = ["BaseController"]
