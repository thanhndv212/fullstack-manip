"""State management for manipulation systems.

Provides centralized state estimation and state tracking for all components
in a manipulation system.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Protocol

import numpy as np


class StateType(Enum):
    """Types of states tracked in the system."""

    ROBOT = "robot"
    OBJECT = "object"
    SENSOR = "sensor"
    TASK = "task"
    ENVIRONMENT = "environment"
    GRIPPER = "gripper"


@dataclass
class RobotState:
    """Robot state information."""

    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    joint_efforts: Optional[np.ndarray] = None
    end_effector_pose: Optional[np.ndarray] = None
    end_effector_velocity: Optional[np.ndarray] = None
    timestamp: float = 0.0


@dataclass
class ObjectState:
    """Object state information."""

    name: str
    position: np.ndarray
    orientation: np.ndarray  # quaternion [w, x, y, z]
    velocity: Optional[np.ndarray] = None
    angular_velocity: Optional[np.ndarray] = None
    in_contact: bool = False
    grasped: bool = False
    timestamp: float = 0.0


@dataclass
class GripperState:
    """Gripper state information."""

    position: float
    force: Optional[float] = None
    is_open: bool = True
    is_closed: bool = False
    is_grasping: bool = False
    timestamp: float = 0.0


@dataclass
class TaskState:
    """High-level task state."""

    name: str
    status: str  # "idle", "running", "success", "failure", "aborted"
    current_phase: Optional[str] = None
    progress: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = 0.0


class StateObserver(Protocol):
    """Protocol for state observers."""

    def on_state_update(self, state_type: StateType, state_data: Any) -> None:
        """Called when state is updated."""
        ...


class StateManager:
    """
    Centralized state management for manipulation systems.

    Tracks robot state, object states, gripper state, task state,
    and sensor data. Implements observer pattern for state updates.
    """

    def __init__(self):
        """Initialize state manager."""
        # State storage
        self._robot_state: Optional[RobotState] = None
        self._object_states: Dict[str, ObjectState] = {}
        self._gripper_state: Optional[GripperState] = None
        self._task_state: Optional[TaskState] = None
        self._sensor_data: Dict[str, Any] = {}
        self._environment_state: Dict[str, Any] = {}

        # Observers
        self._observers: Dict[StateType, List[StateObserver]] = {
            state_type: [] for state_type in StateType
        }

    # === Robot State ===

    def update_robot_state(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        joint_efforts: Optional[np.ndarray] = None,
        end_effector_pose: Optional[np.ndarray] = None,
        end_effector_velocity: Optional[np.ndarray] = None,
        timestamp: Optional[float] = None,
    ) -> None:
        """Update robot state."""
        if timestamp is None:
            import time

            timestamp = time.time()

        self._robot_state = RobotState(
            joint_positions=joint_positions,
            joint_velocities=joint_velocities,
            joint_efforts=joint_efforts,
            end_effector_pose=end_effector_pose,
            end_effector_velocity=end_effector_velocity,
            timestamp=timestamp,
        )
        self._notify_observers(StateType.ROBOT, self._robot_state)

    def get_robot_state(self) -> Optional[RobotState]:
        """Get current robot state."""
        return self._robot_state

    # === Object State ===

    def update_object_state(
        self,
        name: str,
        position: np.ndarray,
        orientation: np.ndarray,
        velocity: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
        in_contact: bool = False,
        grasped: bool = False,
        timestamp: Optional[float] = None,
    ) -> None:
        """Update object state."""
        if timestamp is None:
            import time

            timestamp = time.time()

        self._object_states[name] = ObjectState(
            name=name,
            position=position,
            orientation=orientation,
            velocity=velocity,
            angular_velocity=angular_velocity,
            in_contact=in_contact,
            grasped=grasped,
            timestamp=timestamp,
        )
        self._notify_observers(StateType.OBJECT, self._object_states[name])

    def get_object_state(self, name: str) -> Optional[ObjectState]:
        """Get state of a specific object."""
        return self._object_states.get(name)

    def get_all_object_states(self) -> Dict[str, ObjectState]:
        """Get all object states."""
        return self._object_states.copy()

    def remove_object(self, name: str) -> None:
        """Remove an object from tracking."""
        if name in self._object_states:
            del self._object_states[name]

    # === Gripper State ===

    def update_gripper_state(
        self,
        position: float,
        force: Optional[float] = None,
        is_open: bool = False,
        is_closed: bool = False,
        is_grasping: bool = False,
        timestamp: Optional[float] = None,
    ) -> None:
        """Update gripper state."""
        if timestamp is None:
            import time

            timestamp = time.time()

        self._gripper_state = GripperState(
            position=position,
            force=force,
            is_open=is_open,
            is_closed=is_closed,
            is_grasping=is_grasping,
            timestamp=timestamp,
        )
        self._notify_observers(StateType.GRIPPER, self._gripper_state)

    def get_gripper_state(self) -> Optional[GripperState]:
        """Get current gripper state."""
        return self._gripper_state

    # === Task State ===

    def update_task_state(
        self,
        name: str,
        status: str,
        current_phase: Optional[str] = None,
        progress: float = 0.0,
        metadata: Optional[Dict[str, Any]] = None,
        timestamp: Optional[float] = None,
    ) -> None:
        """Update task state."""
        if timestamp is None:
            import time

            timestamp = time.time()

        self._task_state = TaskState(
            name=name,
            status=status,
            current_phase=current_phase,
            progress=progress,
            metadata=metadata or {},
            timestamp=timestamp,
        )
        self._notify_observers(StateType.TASK, self._task_state)

    def get_task_state(self) -> Optional[TaskState]:
        """Get current task state."""
        return self._task_state

    # === Sensor Data ===

    def update_sensor_data(
        self, sensor_name: str, data: Any, timestamp: Optional[float] = None
    ) -> None:
        """Update sensor data."""
        if timestamp is None:
            import time

            timestamp = time.time()

        self._sensor_data[sensor_name] = {"data": data, "timestamp": timestamp}
        self._notify_observers(
            StateType.SENSOR, self._sensor_data[sensor_name]
        )

    def get_sensor_data(self, sensor_name: str) -> Optional[Any]:
        """Get data from a specific sensor."""
        sensor_info = self._sensor_data.get(sensor_name)
        return sensor_info["data"] if sensor_info else None

    # === Environment State ===

    def update_environment_state(self, key: str, value: Any) -> None:
        """Update environment state."""
        self._environment_state[key] = value
        self._notify_observers(StateType.ENVIRONMENT, {key: value})

    def get_environment_state(self, key: str) -> Optional[Any]:
        """Get environment state value."""
        return self._environment_state.get(key)

    # === Observer Pattern ===

    def register_observer(
        self, state_type: StateType, observer: StateObserver
    ) -> None:
        """Register an observer for state updates."""
        if observer not in self._observers[state_type]:
            self._observers[state_type].append(observer)

    def unregister_observer(
        self, state_type: StateType, observer: StateObserver
    ) -> None:
        """Unregister an observer."""
        if observer in self._observers[state_type]:
            self._observers[state_type].remove(observer)

    def _notify_observers(
        self, state_type: StateType, state_data: Any
    ) -> None:
        """Notify all observers of state update."""
        for observer in self._observers[state_type]:
            observer.on_state_update(state_type, state_data)

    # === Utility Methods ===

    def get_state_summary(self) -> Dict[str, Any]:
        """Get summary of all states."""
        return {
            "robot": self._robot_state,
            "objects": self._object_states,
            "gripper": self._gripper_state,
            "task": self._task_state,
            "sensors": list(self._sensor_data.keys()),
            "environment": self._environment_state,
        }

    def reset(self) -> None:
        """Reset all states."""
        self._robot_state = None
        self._object_states.clear()
        self._gripper_state = None
        self._task_state = None
        self._sensor_data.clear()
        self._environment_state.clear()


__all__ = [
    "StateManager",
    "StateType",
    "RobotState",
    "ObjectState",
    "GripperState",
    "TaskState",
    "StateObserver",
]
