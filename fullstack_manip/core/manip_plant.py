"""Manipulation Plant - Main orchestrator for manipulation systems.

Provides a unified interface for composing and managing all components
of a manipulation system.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Union

import mujoco

from .interfaces import (
    ControllerInterface,
    MotionPlannerInterface,
    RobotInterface,
    SceneUnderstandingInterface,
    SensorInterface,
    StateManagerInterface,
    TaskPlannerInterface,
)
from .objects import ObjectManager
from .state import StateManager


class ManipulationPlant:
    """
    Main orchestration class for manipulation systems.

    Composes and manages:
    - Robot(s)
    - Motion planner(s)
    - Controller(s)
    - State manager
    - Object manager
    - Sensors
    - Task planner
    - Scene understanding

    Use the builder pattern for construction or load from configuration.
    """

    def __init__(
        self,
        name: str = "manipulation_plant",
        model: Optional[mujoco.MjModel] = None,
        data: Optional[mujoco.MjData] = None,
    ):
        """
        Initialize manipulation plant.

        Args:
            name: Plant identifier
            model: MuJoCo model (optional)
            data: MuJoCo data (optional)
        """
        self.name = name
        self.model = model
        self.data = data

        # Core components
        self._robot: Optional[RobotInterface] = None
        self._motion_planner: Optional[MotionPlannerInterface] = None
        self._state_manager: Optional[StateManagerInterface] = None
        self._object_manager: Optional[ObjectManager] = None

        # Optional components
        self._controllers: Dict[str, ControllerInterface] = {}
        self._sensors: Dict[str, SensorInterface] = {}
        self._task_planner: Optional[TaskPlannerInterface] = None
        self._scene_understanding: Optional[SceneUnderstandingInterface] = None

        # Component registry
        self._components: Dict[str, Any] = {}

    # === Core Component Setters ===

    def set_robot(self, robot: RobotInterface) -> ManipulationPlant:
        """Set robot component."""
        self._robot = robot
        self._components["robot"] = robot
        return self

    def set_motion_planner(
        self, planner: MotionPlannerInterface
    ) -> ManipulationPlant:
        """Set motion planner component."""
        self._motion_planner = planner
        self._components["motion_planner"] = planner
        return self

    def set_state_manager(
        self, state_manager: StateManagerInterface
    ) -> ManipulationPlant:
        """Set state manager component."""
        self._state_manager = state_manager
        self._components["state_manager"] = state_manager
        return self

    def set_object_manager(
        self, object_manager: ObjectManager
    ) -> ManipulationPlant:
        """Set object manager component."""
        self._object_manager = object_manager
        self._components["object_manager"] = object_manager
        return self

    # === Optional Component Setters ===

    def add_controller(
        self, name: str, controller: ControllerInterface
    ) -> ManipulationPlant:
        """Add a controller."""
        self._controllers[name] = controller
        self._components[f"controller_{name}"] = controller
        return self

    def add_sensor(
        self, name: str, sensor: SensorInterface
    ) -> ManipulationPlant:
        """Add a sensor."""
        self._sensors[name] = sensor
        self._components[f"sensor_{name}"] = sensor
        return self

    def set_task_planner(
        self, task_planner: TaskPlannerInterface
    ) -> ManipulationPlant:
        """Set task planner component."""
        self._task_planner = task_planner
        self._components["task_planner"] = task_planner
        return self

    def set_scene_understanding(
        self, scene_understanding: SceneUnderstandingInterface
    ) -> ManipulationPlant:
        """Set scene understanding component."""
        self._scene_understanding = scene_understanding
        self._components["scene_understanding"] = scene_understanding
        return self

    # === Component Getters ===

    @property
    def robot(self) -> Optional[RobotInterface]:
        """Get robot component."""
        return self._robot

    @property
    def motion_planner(self) -> Optional[MotionPlannerInterface]:
        """Get motion planner component."""
        return self._motion_planner

    @property
    def state_manager(self) -> Optional[StateManagerInterface]:
        """Get state manager component."""
        return self._state_manager

    @property
    def object_manager(self) -> Optional[ObjectManager]:
        """Get object manager component."""
        return self._object_manager

    @property
    def controllers(self) -> Dict[str, ControllerInterface]:
        """Get all controllers."""
        return self._controllers.copy()

    @property
    def sensors(self) -> Dict[str, SensorInterface]:
        """Get all sensors."""
        return self._sensors.copy()

    @property
    def task_planner(self) -> Optional[TaskPlannerInterface]:
        """Get task planner component."""
        return self._task_planner

    @property
    def scene_understanding(self) -> Optional[SceneUnderstandingInterface]:
        """Get scene understanding component."""
        return self._scene_understanding

    def get_controller(self, name: str) -> Optional[ControllerInterface]:
        """Get controller by name."""
        return self._controllers.get(name)

    def get_sensor(self, name: str) -> Optional[SensorInterface]:
        """Get sensor by name."""
        return self._sensors.get(name)

    # === Lifecycle Methods ===

    def initialize(self) -> None:
        """Initialize all components."""
        # Validate required components
        if self._robot is None:
            raise RuntimeError("Robot component is required")
        if self._state_manager is None:
            # Auto-create state manager if not provided
            self._state_manager = StateManager()

        # Initialize object manager if not provided
        if self._object_manager is None:
            self._object_manager = ObjectManager(self.model, self.data)

        # TODO: Initialize other components as needed

    def reset(self) -> None:
        """Reset all components to initial state."""
        if self._state_manager is not None:
            self._state_manager.reset()

        for controller in self._controllers.values():
            if hasattr(controller, "reset"):
                controller.reset()

    def step(self) -> None:
        """Execute one step of the manipulation system."""
        # Update robot state
        if self._robot is not None and self._state_manager is not None:
            joint_pos = self._robot.get_robot_joint_positions()
            if joint_pos is not None:
                get_vel = getattr(
                    self._robot, "get_robot_joint_velocities", lambda: None
                )
                joint_vel = get_vel()
                if joint_vel is None:
                    joint_vel = joint_pos * 0.0  # Zero velocity fallback

                self._state_manager.update_robot_state(
                    joint_positions=joint_pos,
                    joint_velocities=joint_vel,
                )

        # Update object states
        if self._object_manager is not None:
            if self._state_manager is not None:
                self._object_manager.update_all_states(self._state_manager)

        # Read sensors
        for name, sensor in self._sensors.items():
            if self._state_manager is not None:
                data = sensor.read()
                self._state_manager.update_sensor_data(name, data)

    # === High-Level Operations ===

    def execute_skill(self, skill_name: str, **kwargs) -> bool:
        """
        Execute a high-level skill.

        Args:
            skill_name: Name of skill to execute
            **kwargs: Skill parameters

        Returns:
            True if successful, False otherwise
        """
        if self._task_planner is not None:
            return self._task_planner.execute_skill(skill_name, **kwargs)
        else:
            raise RuntimeError("No task planner configured")

    def move_to_pose(
        self,
        target_pos: List[float],
        target_orient: Optional[List[float]] = None,
        controller_name: Optional[str] = None,
    ) -> bool:
        """
        Move robot to target pose.

        Args:
            target_pos: Target position [x, y, z]
            target_orient: Target orientation (quaternion)
            controller_name: Controller to use (default: first available)

        Returns:
            True if successful
        """
        import numpy as np

        if controller_name is None:
            if not self._controllers:
                raise RuntimeError("No controllers configured")
            controller_name = list(self._controllers.keys())[0]

        controller = self._controllers.get(controller_name)
        if controller is None:
            raise ValueError(f"Controller {controller_name} not found")

        return controller.execute(
            target_pose=np.array(target_pos),
            target_orient=np.array(target_orient) if target_orient else None,
        )

    # === Utility Methods ===

    def get_state_summary(self) -> Dict[str, Any]:
        """Get summary of entire system state."""
        summary = {
            "name": self.name,
            "components": {
                "robot": self._robot is not None,
                "motion_planner": self._motion_planner is not None,
                "state_manager": self._state_manager is not None,
                "object_manager": self._object_manager is not None,
                "controllers": list(self._controllers.keys()),
                "sensors": list(self._sensors.keys()),
                "task_planner": self._task_planner is not None,
                "scene_understanding": self._scene_understanding is not None,
            },
        }

        if self._state_manager is not None:
            summary["state"] = self._state_manager.get_state_summary()

        if self._object_manager is not None:
            summary["scene"] = self._object_manager.get_scene_summary()

        return summary

    def visualize(self, output_path: Optional[str] = None) -> str:
        """
        Generate component diagram.

        Args:
            output_path: Path to save diagram (optional)

        Returns:
            Diagram as string (DOT format)
        """
        # Create simple text diagram
        lines = [f"ManipulationPlant: {self.name}", "=" * 50]

        lines.append("\nCore Components:")
        lines.append(f"  Robot: {'✓' if self._robot else '✗'}")
        lines.append(
            f"  Motion Planner: {'✓' if self._motion_planner else '✗'}"
        )
        lines.append(f"  State Manager: {'✓' if self._state_manager else '✗'}")
        lines.append(
            f"  Object Manager: {'✓' if self._object_manager else '✗'}"
        )

        if self._controllers:
            lines.append(f"\nControllers ({len(self._controllers)}):")
            for name in self._controllers:
                lines.append(f"  - {name}")

        if self._sensors:
            lines.append(f"\nSensors ({len(self._sensors)}):")
            for name in self._sensors:
                lines.append(f"  - {name}")

        if self._task_planner:
            lines.append("\nTask Planner: ✓")

        if self._scene_understanding:
            lines.append("Scene Understanding: ✓")

        diagram = "\n".join(lines)

        if output_path:
            Path(output_path).write_text(diagram)

        return diagram

    # === Builder Pattern ===

    @staticmethod
    def builder() -> ManipulationPlantBuilder:
        """Create a builder for constructing manipulation plant."""
        return ManipulationPlantBuilder()

    @staticmethod
    def from_config(config_path: Union[str, Path]) -> ManipulationPlant:
        """
        Load manipulation plant from configuration file.

        Args:
            config_path: Path to YAML/JSON configuration

        Returns:
            Configured manipulation plant

        Note:
            Configuration system implementation pending
        """
        raise NotImplementedError(
            "Configuration loading will be implemented in next phase"
        )


class ManipulationPlantBuilder:
    """Builder for constructing ManipulationPlant instances."""

    def __init__(self):
        """Initialize builder."""
        self._name = "manipulation_plant"
        self._model: Optional[mujoco.MjModel] = None
        self._data: Optional[mujoco.MjData] = None
        self._robot: Optional[RobotInterface] = None
        self._motion_planner: Optional[MotionPlannerInterface] = None
        self._state_manager: Optional[StateManagerInterface] = None
        self._object_manager: Optional[ObjectManager] = None
        self._controllers: Dict[str, ControllerInterface] = {}
        self._sensors: Dict[str, SensorInterface] = {}
        self._task_planner: Optional[TaskPlannerInterface] = None
        self._scene_understanding: Optional[SceneUnderstandingInterface] = None

    def with_name(self, name: str) -> ManipulationPlantBuilder:
        """Set plant name."""
        self._name = name
        return self

    def with_model(
        self, model: mujoco.MjModel, data: Optional[mujoco.MjData] = None
    ) -> ManipulationPlantBuilder:
        """Set MuJoCo model and data."""
        self._model = model
        self._data = data or mujoco.MjData(model)
        return self

    def with_robot(self, robot: RobotInterface) -> ManipulationPlantBuilder:
        """Set robot component."""
        self._robot = robot
        return self

    def with_motion_planner(
        self, planner: MotionPlannerInterface
    ) -> ManipulationPlantBuilder:
        """Set motion planner."""
        self._motion_planner = planner
        return self

    def with_state_manager(
        self, state_manager: StateManagerInterface
    ) -> ManipulationPlantBuilder:
        """Set state manager."""
        self._state_manager = state_manager
        return self

    def with_object_manager(
        self, object_manager: ObjectManager
    ) -> ManipulationPlantBuilder:
        """Set object manager."""
        self._object_manager = object_manager
        return self

    def with_controller(
        self, name: str, controller: ControllerInterface
    ) -> ManipulationPlantBuilder:
        """Add a controller."""
        self._controllers[name] = controller
        return self

    def with_sensor(
        self, name: str, sensor: SensorInterface
    ) -> ManipulationPlantBuilder:
        """Add a sensor."""
        self._sensors[name] = sensor
        return self

    def with_task_planner(
        self, task_planner: TaskPlannerInterface
    ) -> ManipulationPlantBuilder:
        """Set task planner."""
        self._task_planner = task_planner
        return self

    def with_scene_understanding(
        self, scene_understanding: SceneUnderstandingInterface
    ) -> ManipulationPlantBuilder:
        """Set scene understanding."""
        self._scene_understanding = scene_understanding
        return self

    def build(self) -> ManipulationPlant:
        """Build the manipulation plant."""
        plant = ManipulationPlant(self._name, self._model, self._data)

        if self._robot is not None:
            plant.set_robot(self._robot)
        if self._motion_planner is not None:
            plant.set_motion_planner(self._motion_planner)
        if self._state_manager is not None:
            plant.set_state_manager(self._state_manager)
        if self._object_manager is not None:
            plant.set_object_manager(self._object_manager)

        for name, controller in self._controllers.items():
            plant.add_controller(name, controller)

        for name, sensor in self._sensors.items():
            plant.add_sensor(name, sensor)

        if self._task_planner is not None:
            plant.set_task_planner(self._task_planner)
        if self._scene_understanding is not None:
            plant.set_scene_understanding(self._scene_understanding)

        # Initialize plant
        plant.initialize()

        return plant


__all__ = ["ManipulationPlant", "ManipulationPlantBuilder"]
