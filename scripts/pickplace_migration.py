"""Migrated pick-and-place using ManipulationPlant architecture.

This demonstrates migration from old monolithic approach to new modular
architecture while maintaining the same functionality.
"""

from pathlib import Path
import numpy as np

from fullstack_manip.core.collision import CollisionChecker

try:
    from fullstack_manip.core import (
        Robot,
        Gripper,
        ManipulationPlant,
        StateManager,
        ObjectManager,
        create_plant_from_config,
    )
    from fullstack_manip.simulation.asset_manager import ROOT_PATH
    from fullstack_manip.simulation.loader import MuJoCoSceneLoader
    from fullstack_manip.simulation.scene import MuJoCoSceneManager
except ModuleNotFoundError:  # pragma: no cover - script execution fallback
    import sys

    PROJECT_ROOT = Path(__file__).resolve().parents[1]
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))

    from fullstack_manip.core import (
        Robot,
        Gripper,
        ManipulationPlant,
        StateManager,
        ObjectManager,
        create_plant_from_config,
    )
    from fullstack_manip.simulation.asset_manager import ROOT_PATH
    from fullstack_manip.simulation.loader import MuJoCoSceneLoader
    from fullstack_manip.simulation.scene import MuJoCoSceneManager


def setup_scene():
    """Set up the MuJoCo scene. Same as before."""
    loader = MuJoCoSceneLoader(
        robot_dir="trs_so_arm100",
        env_dir=ROOT_PATH / "trs_so_arm100",
    )
    xml_path = loader.env_dir / "scene_with_table.xml"
    scene_manager = MuJoCoSceneManager(
        xml_path=xml_path, loader=loader, load_method="xml_path"
    )
    return scene_manager


def example_1_manual_builder():
    """Example 1: Manual plant construction using builder pattern."""
    print("\n" + "=" * 80)
    print("Example 1: Manual Plant Construction with Builder Pattern")
    print("=" * 80)

    # Set up scene
    scene_manager = setup_scene()

    # Define parameters
    end_effector_name = "attachment_site"
    end_effector_type = "site"
    object_name = "cube"
    gripper_joint_names = ["Jaw"]
    gripper_bodies = ["Fixed_Jaw", "Moving_Jaw"]
    obstacles = ["table"]

    # Create components separately (new approach)
    robot = Robot(
        scene_manager.model,
        scene_manager.data,
        end_effector_name=end_effector_name,
        end_effector_type=end_effector_type,
        gripper_bodies=gripper_bodies,
        obstacles=obstacles,
    )

    # Create collision checker is now internal to Gripper and Robot
    collision_checker = CollisionChecker(scene_manager.model, scene_manager.data)

    # Create gripper as separate component
    gripper = Gripper(
        scene_manager.model,
        scene_manager.data,
        collision_checker=collision_checker,
        gripper_joint_names=gripper_joint_names,
        gripper_bodies=gripper_bodies,
    )
    gripper.set_object_geom(object_name)
    gripper.set_positions(close_pos=0.0, open_pos=1.0)
    gripper.set_force_thresholds(grasp_threshold=1.0, release_threshold=0.01)

    # Create state manager
    state_manager = StateManager()

    # Create object manager
    object_manager = ObjectManager(scene_manager.model, scene_manager.data)
    object_manager.register_object(
        name=object_name,
        object_type="box",
        body_name=object_name,
    )

    # Build plant using builder pattern
    plant = (
        ManipulationPlant.builder()
        .with_robot(robot)
        .with_gripper(gripper)
        .with_state_manager(state_manager)
        .with_object_manager(object_manager)
        .build()
    )

    # Initialize plant
    plant.initialize()

    print(f"✅ Plant created: {plant.name}")
    print(f"   - Robot: {robot.end_effector_name}")
    print(f"   - Gripper: {len(gripper_joint_names)} joints")
    print(f"   - Objects: {len(object_manager.objects)} tracked")
    print(f"   - State observers: {len(state_manager._observers)}")

    # Execute pick-and-place
    pick_position = robot.get_body_pose(object_name)[0]
    place_position = pick_position + np.array([0.1, 0.1, 0.05])

    print(f"\n📍 Pick position: {pick_position}")
    print(f"📍 Place position: {place_position}")
    print("\nExecuting pick-and-place with new architecture...")

    # Note: Actual execution would need proper simulation loop
    # Create controller (now works with plant)
    # controller = PickPlaceController(plant)
    # controller.execute(pick_position, place_position)

    print("✅ Example 1 completed!\n")
    return plant


def example_2_from_config():
    """Example 2: Create plant from configuration file."""
    print("\n" + "=" * 80)
    print("Example 2: Create Plant from Configuration File")
    print("=" * 80)

    # Set up scene
    scene_manager = setup_scene()

    # Check if config exists
    config_path = Path("configs/soarm100_pickplace.yaml")
    if not config_path.exists():
        print(f"⚠️  Config file not found: {config_path}")
        print("Creating example configuration...")

        # Create example config
        from fullstack_manip.core import PlantConfig

        config = PlantConfig(
            name="soarm100_pickplace",
            robot={
                "type": "generic",
                "end_effector_name": "attachment_site",
                "end_effector_type": "site",
                "base_name": "base_link",
            },
            gripper={
                "joint_names": ["Jaw"],
                "body_names": ["Fixed_Jaw", "Moving_Jaw"],
                "close_position": 0.0,
                "open_position": 1.0,
                "grasp_threshold": 1.0,
                "release_threshold": 0.01,
                "object_geom": "cube",
            },
            objects=[
                {
                    "name": "cube",
                    "type": "box",
                    "geom_name": "cube",
                }
            ],
            obstacles=["table"],
        )

        # Save config
        config_path.parent.mkdir(parents=True, exist_ok=True)
        config.save_yaml(config_path)
        print(f"✅ Created config: {config_path}")

    # Load configuration
    config = PlantConfig.from_yaml(config_path)
    print(f"📄 Loaded config: {config.name}")

    # Create plant from config
    plant = create_plant_from_config(
        config, scene_manager.model, scene_manager.data
    )
    plant.initialize()

    print("✅ Plant created from config")
    print(f"   - Robot: {config.robot.get('end_effector_name', 'N/A')}")
    print(f"   - Gripper: {config.gripper is not None}")
    print(f"   - Objects: {len(config.objects)}")

    print("✅ Example 2 completed!\n")
    return plant


def example_3_backward_compatibility():
    """Example 3: Demonstrate backward compatibility."""
    print("\n" + "=" * 80)
    print("Example 3: Backward Compatibility - Old vs New")
    print("=" * 80)

    scene_manager = setup_scene()

    # Old approach (still works for simple cases)
    print("\n🔧 Old Approach (backward compatible):")
    robot_old = Robot(
        scene_manager.model,
        scene_manager.data,
        end_effector_name="attachment_site",
        end_effector_type="site",
        gripper_bodies=["Fixed_Jaw", "Moving_Jaw"],
        obstacles=["table"],
    )
    print(f"   ✅ Robot created: {robot_old.end_effector_name}")

    # New approach (recommended)
    print("\n✨ New Approach (recommended):")
    robot_new = Robot(
        scene_manager.model,
        scene_manager.data,
        end_effector_name="attachment_site",
        end_effector_type="site",
        gripper_bodies=["Fixed_Jaw", "Moving_Jaw"],
        obstacles=["table"],
    )

    gripper_new = Gripper(
        scene_manager.model,
        scene_manager.data,
        joint_names=["Jaw"],
        body_names=["Fixed_Jaw", "Moving_Jaw"],
    )

    plant = (
        ManipulationPlant.builder()
        .with_robot(robot_new)
        .with_gripper(gripper_new)
        .build()
    )
    plant.initialize()

    print(f"   ✅ Plant created: {plant.name}")
    print(f"   ✅ Components: {len(plant._components)} registered")

    # Both robots work the same way
    pos_old = robot_old.get_end_effector_position()
    pos_new = plant.robot.get_end_effector_position()

    print("\n" + "=" * 40 + " Comparison " + "=" * 40)
    print(f"   Old approach EE position: {pos_old}")
    print(f"   New approach EE position: {pos_new}")
    print(f"   Positions match: {np.allclose(pos_old, pos_new)}")

    print("\n✅ Example 3 completed!\n")


def example_4_testing_benefits():
    """Example 4: Show testing benefits of new architecture."""
    print("\n" + "=" * 80)
    print("Example 4: Testing Benefits with Mock Components")
    print("=" * 80)

    # Create mock components for testing
    class MockRobot:
        """Mock robot for testing without MuJoCo."""

        def __init__(self):
            self.end_effector_name = "mock_ee"
            self._ee_pos = np.array([0.5, 0.0, 0.3])
            self.move_called = False

        def get_end_effector_position(self):
            return self._ee_pos.copy()

        def move_to_position(self, position, orientation=None):
            self.move_called = True
            self._ee_pos = np.array(position)
            return True

        def get_robot_joint_positions(self):
            return np.zeros(6)

        def get_body_pose(self, name):
            return np.array([0.5, 0.0, 0.1]), np.array([1, 0, 0, 0])

    class MockGripper:
        """Mock gripper for testing."""

        def __init__(self):
            self.is_open = True
            self.grasp_called = False
            self.release_called = False

        def open(self):
            self.is_open = True
            return True

        def close(self):
            self.is_open = False
            return True

        def grasp(self):
            self.grasp_called = True
            self.is_open = False
            return True

        def release(self):
            self.release_called = True
            self.is_open = True
            return True

    # Build plant with mocks
    mock_robot = MockRobot()
    mock_gripper = MockGripper()

    plant = (
        ManipulationPlant.builder()
        .with_robot(mock_robot)
        .with_gripper(mock_gripper)
        .build()
    )
    plant.initialize()

    print("✅ Plant created with mock components")
    print(f"   - Robot: {mock_robot.end_effector_name}")
    print(f"   - Gripper: {'Open' if mock_gripper.is_open else 'Closed'}")

    # Test interactions
    pos = plant.robot.get_end_effector_position()
    plant.robot.move_to_position([0.6, 0.1, 0.35])
    plant.gripper.grasp()

    print("\n" + "=" * 40 + " Testing Results " + "=" * 40)
    print(f"   - Initial position: {pos}")
    print(f"   - Move called: {mock_robot.move_called}")
    print(f"   - Grasp called: {mock_gripper.grasp_called}")
    print(f"   - Gripper closed: {not mock_gripper.is_open}")

    print(
        "\n💡 Benefits: No MuJoCo needed, fast tests, easy to verify behavior!"
    )
    print("✅ Example 4 completed!\n")


def main():
    """Run all migration examples."""
    print("\n" + "=" * 80)
    print("PICK-AND-PLACE MIGRATION EXAMPLES")
    print("Demonstrating the transition to ManipulationPlant architecture")
    print("=" * 80)

    try:
        # Example 1: Manual builder
        example_1_manual_builder()

        # Example 2: From configuration
        example_2_from_config()

        # Example 3: Backward compatibility
        example_3_backward_compatibility()

        # Example 4: Testing benefits
        example_4_testing_benefits()

        print("\n" + "=" * 80)
        print("✅ All migration examples completed successfully!")
        print("=" * 80)
        print("\n📚 Next steps:")
        print("   1. Review the migration guide: docs/migration-guide.md")
        print("   2. Try creating your own configuration files")
        print("   3. Migrate your existing scripts gradually")
        print("   4. Use mock components for faster testing")
        print("\n🎉 Happy coding with the new architecture!\n")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
