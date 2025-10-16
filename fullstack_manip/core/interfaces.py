"""Component interfaces for manipulation plant architecture.

Defines protocols (structural subtyping) for pluggable components.
"""

from typing import Any, Dict, List, Optional, Protocol, runtime_checkable

import numpy as np


# === Robot Interface ===


@runtime_checkable
class RobotInterface(Protocol):
    """Protocol for robot components."""

    def get_robot_joint_positions(self) -> Optional[np.ndarray]:
        """Get current joint positions."""
        ...

    def set_robot_joint_positions(self, positions: np.ndarray) -> None:
        """Set joint positions."""
        ...

    def get_body_pose(self, body_name: str) -> tuple[np.ndarray, np.ndarray]:
        """Get body position and orientation (quaternion)."""
        ...


# === Motion Planner Interface ===


@runtime_checkable
class MotionPlannerInterface(Protocol):
    """Protocol for motion planning components."""

    def plan_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        start_orient: Optional[np.ndarray] = None,
        end_orient: Optional[np.ndarray] = None,
        **kwargs,
    ) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Plan trajectory from start to end pose.

        Returns:
            (trajectory, times) or (None, None) if planning failed
        """
        ...

    def solve_ik_for_qpos(
        self,
        frame_name: str,
        frame_type: str,
        target_pos: np.ndarray,
        target_orient: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """
        Solve inverse kinematics for target pose.

        Returns:
            Joint positions or None if no solution found
        """
        ...


# === Controller Interface ===


@runtime_checkable
class ControllerInterface(Protocol):
    """Protocol for controller components."""

    def execute(self, **kwargs) -> bool:
        """
        Execute control action.

        Returns:
            True if successful, False otherwise
        """
        ...

    def reset(self) -> None:
        """Reset controller state."""
        ...


# === State Manager Interface ===


@runtime_checkable
class StateManagerInterface(Protocol):
    """Protocol for state management components."""

    def update_robot_state(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        **kwargs,
    ) -> None:
        """Update robot state."""
        ...

    def get_robot_state(self) -> Any:
        """Get current robot state."""
        ...

    def update_object_state(
        self,
        name: str,
        position: np.ndarray,
        orientation: np.ndarray,
        **kwargs,
    ) -> None:
        """Update object state."""
        ...

    def get_object_state(self, name: str) -> Any:
        """Get object state by name."""
        ...


# === Gripper Interface ===


@runtime_checkable
class GripperInterface(Protocol):
    """Protocol for gripper components."""

    def open(self, **kwargs) -> bool:
        """Open gripper."""
        ...

    def close(self, **kwargs) -> bool:
        """Close gripper."""
        ...

    def get_position(self) -> Optional[float]:
        """Get current gripper position."""
        ...


# === Sensor Interface ===


@runtime_checkable
class SensorInterface(Protocol):
    """Protocol for sensor components."""

    def read(self) -> Any:
        """Read sensor data."""
        ...

    def get_name(self) -> str:
        """Get sensor name."""
        ...


# === Object Interface ===


@runtime_checkable
class ObjectInterface(Protocol):
    """Protocol for manipulatable objects."""

    name: str

    def get_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Get object position and orientation."""
        ...

    def set_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """Set object pose."""
        ...


# === Task Planner Interface ===


@runtime_checkable
class TaskPlannerInterface(Protocol):
    """Protocol for task planning components."""

    def execute_skill(self, skill_name: str, **kwargs) -> bool:
        """Execute a named skill."""
        ...

    def get_status(self) -> str:
        """Get current task status."""
        ...


# === Scene Understanding Interface ===


@runtime_checkable
class SceneUnderstandingInterface(Protocol):
    """Protocol for scene understanding/perception components."""

    def detect_objects(self) -> List[Dict[str, Any]]:
        """
        Detect objects in scene.

        Returns:
            List of object detections with name, pose, confidence
        """
        ...

    def estimate_pose(self, object_name: str) -> Optional[np.ndarray]:
        """Estimate pose of specific object."""
        ...


# === Collision Checker Interface ===


@runtime_checkable
class CollisionCheckerInterface(Protocol):
    """Protocol for collision detection components."""

    def check_collision(
        self, trajectory: np.ndarray, obstacles: List[str]
    ) -> bool:
        """
        Check if trajectory is collision-free.

        Returns:
            True if collision-free, False if collision detected
        """
        ...


__all__ = [
    "RobotInterface",
    "MotionPlannerInterface",
    "ControllerInterface",
    "StateManagerInterface",
    "GripperInterface",
    "SensorInterface",
    "ObjectInterface",
    "TaskPlannerInterface",
    "SceneUnderstandingInterface",
    "CollisionCheckerInterface",
]
