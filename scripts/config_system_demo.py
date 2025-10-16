"""Example: Using configuration files to create ManipulationPlants.

This example demonstrates how to load plants from YAML/JSON
configuration files.
"""

from fullstack_manip.core.config import (
    PlantConfig,
    ConfigurationError,
)


def example_1_load_yaml():
    """Example 1: Load plant from YAML file."""
    print("=" * 60)
    print("Example 1: Load from YAML Configuration")
    print("=" * 60)

    try:
        # Load configuration
        config = PlantConfig.from_yaml("configs/minimal.yaml")
        print(f"✓ Loaded config: {config.name}")

        # Show configuration
        print("\nRobot config:")
        print(f"  Type: {config.robot_config['type']}")
        print(f"  Model: {config.robot_config['model_path']}")
        print(f"  Base link: {config.robot_config['base_link']}")
        print(f"  EE link: {config.robot_config['ee_link']}")

        if config.gripper_config:
            print("\nGripper config:")
            print(f"  Joints: {config.gripper_config['joint_names']}")
            print(f"  Open: {config.gripper_config['open_position']}")
            print(f"  Closed: {config.gripper_config['closed_position']}")

        if config.objects_config:
            print(f"\nObjects ({len(config.objects_config)}):")
            for obj in config.objects_config:
                print(f"  - {obj['name']} ({obj['type']})")

        # Note: To actually create the plant, need MuJoCo model/data
        print("\nTo create plant:")
        print("  plant = create_plant_from_config(config, model, data)")

    except ConfigurationError as e:
        print(f"❌ Configuration error: {e}")

    return config


def example_2_load_json():
    """Example 2: Load plant from JSON file."""
    print("\n" + "=" * 60)
    print("Example 2: Load from JSON Configuration")
    print("=" * 60)

    try:
        # Load configuration
        config = PlantConfig.from_json("configs/assembly.json")
        print(f"✓ Loaded config: {config.name}")

        # Show configuration
        print(f"\nRobot type: {config.robot_config['type']}")

        if config.motion_planner_config:
            print("\nMotion planner:")
            planner_type = config.motion_planner_config["type"]
            max_time = config.motion_planner_config.get("max_planning_time")
            print(f"  Type: {planner_type}")
            print(f"  Max time: {max_time}s")

        print("\nObjects:")
        for obj in config.objects_config:
            name = obj["name"]
            obj_type = obj["type"]
            graspable = obj.get("properties", {}).get("graspable", False)
            print(f"  - {name} ({obj_type}, graspable={graspable})")

    except ConfigurationError as e:
        print(f"❌ Configuration error: {e}")

    return config


def example_3_validate_config():
    """Example 3: Configuration validation."""
    print("\n" + "=" * 60)
    print("Example 3: Configuration Validation")
    print("=" * 60)

    # Valid configuration
    valid_config = {
        "name": "test_plant",
        "robot": {
            "type": "generic",
            "model_path": "robot.xml",
            "base_link": "base",
            "ee_link": "ee",
        },
    }

    try:
        config = PlantConfig(valid_config)
        print("✓ Valid configuration accepted")
    except ConfigurationError as e:
        print(f"❌ Error: {e}")

    # Invalid configuration (missing name)
    invalid_config_1 = {
        "robot": {
            "type": "generic",
            "model_path": "robot.xml",
            "base_link": "base",
            "ee_link": "ee",
        }
    }

    print("\nTesting invalid config (missing name)...")
    try:
        config = PlantConfig(invalid_config_1)
        print("❌ Should have failed!")
    except ConfigurationError as e:
        print(f"✓ Caught error: {e}")

    # Invalid configuration (missing robot fields)
    invalid_config_2 = {
        "name": "test_plant",
        "robot": {
            "type": "generic",
            # Missing required fields
        },
    }

    print("\nTesting invalid config (missing robot fields)...")
    try:
        PlantConfig(invalid_config_2)
        print("❌ Should have failed!")
    except ConfigurationError as e:
        print(f"✓ Caught error: {e}")


def example_4_create_and_save_config():
    """Example 4: Create and save configuration."""
    print("\n" + "=" * 60)
    print("Example 4: Create and Save Configuration")
    print("=" * 60)

    # Create configuration programmatically
    config_dict = {
        "name": "dynamic_plant",
        "robot": {
            "type": "generic",
            "model_path": "hardware/urdf/robot.xml",
            "base_link": "base_link",
            "ee_link": "end_effector",
        },
        "gripper": {
            "joint_names": ["gripper_left", "gripper_right"],
            "open_position": 0.04,
            "closed_position": 0.0,
        },
        "objects": [
            {
                "name": "cube_1",
                "type": "box",
                "position": [0.5, 0.0, 0.05],
                "properties": {
                    "size": [0.05, 0.05, 0.05],
                    "graspable": True,
                },
            }
        ],
    }

    config = PlantConfig(config_dict)
    print(f"✓ Created config: {config.name}")

    # Save to YAML
    try:
        config.to_yaml("configs/dynamic_plant.yaml")
        print("✓ Saved to: configs/dynamic_plant.yaml")
    except ConfigurationError as e:
        print(f"⚠ Could not save YAML: {e}")

    # Save to JSON
    try:
        config.to_json("configs/dynamic_plant.json")
        print("✓ Saved to: configs/dynamic_plant.json")
    except Exception as e:
        print(f"⚠ Could not save JSON: {e}")


def example_5_config_hierarchy():
    """Example 5: Show configuration hierarchy."""
    print("\n" + "=" * 60)
    print("Example 5: Configuration Hierarchy")
    print("=" * 60)

    config = PlantConfig.from_yaml("configs/pickplace.yaml")

    print("Configuration structure:")
    print(
        f"""
ManipulationPlant: {config.name}
├── Robot: {config.robot_config.get('type')}
│   ├── Model: {config.robot_config.get('model_path')}
│   ├── Base: {config.robot_config.get('base_link')}
│   └── EE: {config.robot_config.get('ee_link')}
├── Gripper: {'✓' if config.gripper_config else '✗'}
├── Motion Planner: {'✓' if config.motion_planner_config else '✗'}
├── Controllers: {len(config.controllers_config)}
├── Sensors: {len(config.sensors_config)}
└── Objects: {len(config.objects_config)}
    """
    )

    if config.objects_config:
        print("Objects:")
        for obj in config.objects_config:
            print(f"  └── {obj['name']} ({obj['type']})")


def main():
    """Run all configuration examples."""
    print("\n🔧 ManipulationPlant Configuration System Examples\n")

    try:
        example_1_load_yaml()
        example_2_load_json()
        example_3_validate_config()
        example_4_create_and_save_config()
        example_5_config_hierarchy()

        print("\n" + "=" * 60)
        print("✅ All examples completed successfully!")
        print("=" * 60)
        print("\nKey Takeaways:")
        print("  1. Load plants from YAML or JSON files")
        print("  2. Configuration is validated automatically")
        print("  3. Create plants with: create_plant_from_config()")
        print("  4. Save configs programmatically")
        print("  5. Easy to version control and share configs")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
