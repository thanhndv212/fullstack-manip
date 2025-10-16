"""Configuration system for ManipulationPlant.

This module provides YAML/JSON-based configuration loading for creating
manipulation plants declaratively.
"""

import json
from pathlib import Path
from typing import Any, Dict, Optional, Union

import numpy as np

try:
    import yaml

    HAS_YAML = True
except ImportError:
    HAS_YAML = False


class ConfigurationError(Exception):
    """Raised when configuration is invalid."""

    pass


class PlantConfig:
    """Configuration for a ManipulationPlant.

    Supports loading from YAML or JSON files and provides
    validation of configuration structure.
    """

    def __init__(self, config_dict: Dict[str, Any]):
        """Initialize configuration.

        Args:
            config_dict: Dictionary containing plant configuration
        """
        self.config = config_dict
        self._validate()

    @classmethod
    def from_yaml(cls, path: Union[str, Path]) -> "PlantConfig":
        """Load configuration from YAML file.

        Args:
            path: Path to YAML configuration file

        Returns:
            PlantConfig instance

        Raises:
            ConfigurationError: If YAML not available or file invalid
        """
        if not HAS_YAML:
            raise ConfigurationError(
                "PyYAML not installed. Install with: pip install pyyaml"
            )

        path = Path(path)
        if not path.exists():
            raise ConfigurationError(f"Config file not found: {path}")

        try:
            with open(path, "r") as f:
                config_dict = yaml.safe_load(f)
            return cls(config_dict)
        except yaml.YAMLError as e:
            raise ConfigurationError(f"Invalid YAML: {e}")

    @classmethod
    def from_json(cls, path: Union[str, Path]) -> "PlantConfig":
        """Load configuration from JSON file.

        Args:
            path: Path to JSON configuration file

        Returns:
            PlantConfig instance

        Raises:
            ConfigurationError: If file invalid
        """
        path = Path(path)
        if not path.exists():
            raise ConfigurationError(f"Config file not found: {path}")

        try:
            with open(path, "r") as f:
                config_dict = json.load(f)
            return cls(config_dict)
        except json.JSONDecodeError as e:
            raise ConfigurationError(f"Invalid JSON: {e}")

    def _validate(self) -> None:
        """Validate configuration structure.

        Raises:
            ConfigurationError: If configuration is invalid
        """
        # Check required fields
        if "name" not in self.config:
            raise ConfigurationError("Configuration must have 'name' field")

        if "robot" not in self.config:
            raise ConfigurationError("Configuration must have 'robot' field")

        # Validate robot config
        robot_config = self.config["robot"]
        required_robot_fields = ["type", "model_path", "base_link", "ee_link"]
        for field in required_robot_fields:
            if field not in robot_config:
                raise ConfigurationError(
                    f"Robot config missing required field: {field}"
                )

    @property
    def name(self) -> str:
        """Get plant name."""
        return self.config["name"]

    @property
    def robot_config(self) -> Dict[str, Any]:
        """Get robot configuration."""
        return self.config["robot"]

    @property
    def gripper_config(self) -> Optional[Dict[str, Any]]:
        """Get gripper configuration."""
        return self.config.get("gripper")

    @property
    def motion_planner_config(self) -> Optional[Dict[str, Any]]:
        """Get motion planner configuration."""
        return self.config.get("motion_planner")

    @property
    def controllers_config(self) -> Dict[str, Dict[str, Any]]:
        """Get controllers configuration."""
        return self.config.get("controllers", {})

    @property
    def sensors_config(self) -> Dict[str, Dict[str, Any]]:
        """Get sensors configuration."""
        return self.config.get("sensors", {})

    @property
    def objects_config(self) -> list[Dict[str, Any]]:
        """Get objects configuration."""
        return self.config.get("objects", [])

    @property
    def state_manager_config(self) -> Optional[Dict[str, Any]]:
        """Get state manager configuration."""
        return self.config.get("state_manager")

    def to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary.

        Returns:
            Configuration dictionary
        """
        return self.config.copy()

    def to_yaml(self, path: Union[str, Path]) -> None:
        """Save configuration to YAML file.

        Args:
            path: Path to save YAML file

        Raises:
            ConfigurationError: If YAML not available
        """
        if not HAS_YAML:
            raise ConfigurationError(
                "PyYAML not installed. Install with: pip install pyyaml"
            )

        path = Path(path)
        with open(path, "w") as f:
            yaml.dump(self.config, f, default_flow_style=False)

    def to_json(self, path: Union[str, Path]) -> None:
        """Save configuration to JSON file.

        Args:
            path: Path to save JSON file
        """
        path = Path(path)
        with open(path, "w") as f:
            json.dump(self.config, f, indent=2)


class ComponentFactory:
    """Factory for creating components from configuration.

    This factory creates instances of components based on
    configuration dictionaries.
    """

    @staticmethod
    def create_robot(config: Dict[str, Any], model, data):
        """Create robot from configuration.

        Args:
            config: Robot configuration dictionary
            model: MuJoCo model
            data: MuJoCo data

        Returns:
            Robot instance

        Raises:
            ConfigurationError: If robot type not supported
        """
        from fullstack_manip.core import Robot

        robot_type = config.get("type", "generic")

        if robot_type == "generic":
            return Robot(
                model=model,
                data=data,
                model_path=config["model_path"],
                base_link=config["base_link"],
                ee_link=config["ee_link"],
            )
        else:
            raise ConfigurationError(f"Unsupported robot type: {robot_type}")

    @staticmethod
    def create_gripper(config: Dict[str, Any], model, data):
        """Create gripper from configuration.

        Args:
            config: Gripper configuration dictionary
            model: MuJoCo model
            data: MuJoCo data

        Returns:
            Gripper instance
        """
        from fullstack_manip.core import Gripper

        gripper = Gripper(model, data)

        # Configure joint names
        if "joint_names" in config:
            gripper.set_joint_names(config["joint_names"])

        # Configure positions
        if "open_position" in config and "closed_position" in config:
            gripper.set_positions(
                open_pos=config["open_position"],
                closed_pos=config["closed_position"],
            )

        # Configure force thresholds
        if "min_force" in config and "max_force" in config:
            gripper.set_force_thresholds(
                min_force=config["min_force"],
                max_force=config["max_force"],
            )

        return gripper

    @staticmethod
    def create_state_manager(config: Optional[Dict[str, Any]] = None):
        """Create state manager from configuration.

        Args:
            config: State manager configuration dictionary (optional)

        Returns:
            StateManager instance
        """
        from fullstack_manip.core import StateManager

        # For now, StateManager doesn't take configuration
        # In the future, could configure update rates, etc.
        return StateManager()

    @staticmethod
    def create_object_manager(config: Optional[Dict[str, Any]], model, data):
        """Create object manager from configuration.

        Args:
            config: Object manager configuration dictionary (optional)
            model: MuJoCo model
            data: MuJoCo data

        Returns:
            ObjectManager instance
        """
        from fullstack_manip.core import ObjectManager

        return ObjectManager(model, data)

    @staticmethod
    def create_objects(configs: list[Dict[str, Any]], object_manager) -> None:
        """Create and register objects from configuration.

        Args:
            configs: List of object configuration dictionaries
            object_manager: ObjectManager to register objects with
        """
        from fullstack_manip.core import ObjectType

        for obj_config in configs:
            # Create object
            obj = object_manager.register_object(
                name=obj_config["name"],
                object_type=ObjectType[obj_config["type"].upper()],
            )

            # Set pose
            if "position" in obj_config:
                position = np.array(obj_config["position"])
                orientation = np.array(
                    obj_config.get("orientation", [1, 0, 0, 0])
                )
                obj.set_pose(position, orientation)

            # Set properties
            if "properties" in obj_config:
                props = obj_config["properties"]
                if "size" in props:
                    obj.properties.size = props["size"]
                if "mass" in props:
                    obj.properties.mass = props["mass"]
                if "graspable" in props:
                    obj.properties.graspable = props["graspable"]
                if "color" in props:
                    obj.properties.color = props["color"]


def create_plant_from_config(
    config: PlantConfig, model, data
) -> "ManipulationPlant":
    """Create ManipulationPlant from configuration.

    Args:
        config: PlantConfig instance
        model: MuJoCo model
        data: MuJoCo data

    Returns:
        Configured ManipulationPlant instance

    Example:
        >>> config = PlantConfig.from_yaml("config.yaml")
        >>> plant = create_plant_from_config(config, model, data)
    """
    from fullstack_manip.core import ManipulationPlant

    # Create components using factory
    factory = ComponentFactory()

    # Create robot
    robot = factory.create_robot(config.robot_config, model, data)

    # Create gripper if configured
    if config.gripper_config:
        gripper = factory.create_gripper(config.gripper_config, model, data)
        robot.gripper = gripper

    # Create state manager
    state_manager = factory.create_state_manager(config.state_manager_config)

    # Create object manager
    object_manager = factory.create_object_manager(None, model, data)

    # Build plant
    builder = ManipulationPlant.builder()
    builder.with_name(config.name)
    builder.with_robot(robot)
    builder.with_state_manager(state_manager)
    builder.with_object_manager(object_manager)

    # TODO: Add motion planner creation when available
    # TODO: Add controller creation when available
    # TODO: Add sensor creation when available

    plant = builder.build()

    # Register objects
    if config.objects_config:
        factory.create_objects(config.objects_config, object_manager)

    return plant
