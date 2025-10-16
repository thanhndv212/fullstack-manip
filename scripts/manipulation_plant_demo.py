"""Example: Using the new ManipulationPlant architecture.

This example demonstrates how to build a manipulation system
using the modular, composable architecture.
"""

import numpy as np

from fullstack_manip.core import (
    ManipulationPlant,
    ObjectManager,
    ObjectType,
    StateManager,
)


# === Mock Components for Examples ===


class MockRobot:
    """Mock robot for testing/examples."""

    def __init__(self):
        self._joint_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def get_robot_joint_positions(self):
        return self._joint_positions

    def set_robot_joint_positions(self, positions):
        self._joint_positions = positions

    def get_body_pose(self, body_name):
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])


class MockController:
    """Mock controller for testing/examples."""

    def execute(self, **kwargs):
        return True

    def reset(self):
        pass


# === Examples ===


def example_1_basic_plant():
    """Example 1: Create a basic manipulation plant."""
    print("=" * 60)
    print("Example 1: Basic ManipulationPlant")
    print("=" * 60)

    # Method 1: Using builder pattern
    plant = (
        ManipulationPlant.builder()
        .with_name("basic_plant")
        .with_robot(MockRobot())  # Required component
        .with_state_manager(StateManager())
        .with_object_manager(ObjectManager())
        .build()
    )

    print(f"\nPlant name: {plant.name}")
    print("State manager:", "✓" if plant.state_manager else "✗")
    print("Object manager:", "✓" if plant.object_manager else "✗")

    # Get state summary
    summary = plant.get_state_summary()
    print("\nPlant summary:")
    print(f"  Components: {summary['components']}")

    return plant


def example_2_with_objects():
    """Example 2: Plant with object tracking."""
    print("\n" + "=" * 60)
    print("Example 2: Plant with Object Management")
    print("=" * 60)

    # Create plant
    state_mgr = StateManager()
    obj_mgr = ObjectManager()

    plant = (
        ManipulationPlant.builder()
        .with_name("object_tracking_plant")
        .with_robot(MockRobot())
        .with_state_manager(state_mgr)
        .with_object_manager(obj_mgr)
        .build()
    )

    # Register objects
    cube = obj_mgr.register_object(
        name="cube",
        object_type=ObjectType.BOX,
    )
    cube.set_pose(
        position=np.array([0.5, 0.0, 0.1]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # w,x,y,z
    )
    cube.properties.size = [0.05, 0.05, 0.05]
    cube.properties.graspable = True

    sphere = obj_mgr.register_object(
        name="sphere",
        object_type=ObjectType.SPHERE,
    )
    sphere.set_pose(
        position=np.array([0.3, 0.2, 0.1]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    sphere.properties.size = [0.03]
    sphere.properties.graspable = True

    print(f"\nRegistered {len(obj_mgr.get_all_objects())} objects")

    # Query objects
    graspable = obj_mgr.get_graspable_objects()
    print(f"Graspable objects: {[obj.name for obj in graspable]}")

    # Find nearest to gripper
    gripper_pos = np.array([0.4, 0.1, 0.15])
    nearest = obj_mgr.find_nearest_object(gripper_pos)
    if nearest:
        print(f"Nearest object to gripper: {nearest.name}")
        grasp_points = nearest.get_grasp_points()
        print(f"  Suggested grasp points: {len(grasp_points)}")

    # Scene summary
    scene = obj_mgr.get_scene_summary()
    print("\nScene summary:")
    print(f"  Total objects: {scene['total_objects']}")
    print(f"  Graspable: {scene['graspable_count']}")

    return plant


def example_3_state_tracking():
    """Example 3: State tracking with observers."""
    print("\n" + "=" * 60)
    print("Example 3: State Tracking with Observers")
    print("=" * 60)

    from fullstack_manip.core.state import StateType

    # Create observer
    class PrintObserver:
        """Observer that prints state updates."""

        def on_state_update(self, state_type, state_data):
            print(f"  [{state_type.value}] State updated")

    # Create plant with state management
    state_mgr = StateManager()
    observer = PrintObserver()

    # Register observer
    state_mgr.register_observer(StateType.ROBOT, observer)
    state_mgr.register_observer(StateType.OBJECT, observer)

    plant = (
        ManipulationPlant.builder()
        .with_name("state_tracking_plant")
        .with_robot(MockRobot())
        .with_state_manager(state_mgr)
        .build()
    )

    print("\nUpdating states (observer will print):")

    # Update robot state
    state_mgr.update_robot_state(
        joint_positions=np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0]),
        joint_velocities=np.zeros(6),
    )

    # Update object state
    state_mgr.update_object_state(
        name="cube",
        position=np.array([0.5, 0.0, 0.1]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )

    # Query states
    robot_state = state_mgr.get_robot_state()
    if robot_state:
        print("\nRobot state:")
        print(f"  Joint positions: {robot_state.joint_positions}")

    obj_state = state_mgr.get_object_state("cube")
    if obj_state:
        print("\nObject state:")
        print(f"  Name: {obj_state.name}")
        print(f"  Position: {obj_state.position}")

    return plant


def example_4_plant_visualization():
    """Example 4: Visualize plant components."""
    print("\n" + "=" * 60)
    print("Example 4: Plant Visualization")
    print("=" * 60)

    # Create complex plant with multiple components
    plant = (
        ManipulationPlant.builder()
        .with_name("complex_plant")
        .with_robot(MockRobot())
        .with_state_manager(StateManager())
        .with_object_manager(ObjectManager())
        .build()
    )

    # Visualize
    diagram = plant.visualize()
    print("\n" + diagram)

    return plant


def example_5_mock_components():
    """Example 5: Using custom components."""
    print("\n" + "=" * 60)
    print("Example 5: Custom Components")
    print("=" * 60)

    # Build plant with custom components
    mock_robot = MockRobot()
    mock_controller = MockController()

    plant = (
        ManipulationPlant.builder()
        .with_name("test_plant")
        .with_robot(mock_robot)
        .with_controller("test_controller", mock_controller)
        .with_state_manager(StateManager())
        .build()
    )

    print(f"\nPlant '{plant.name}' created with:")
    print(f"  Robot: {type(plant.robot).__name__}")
    print(f"  Controllers: {list(plant.controllers.keys())}")

    # Use controller
    print("\nExecuting controller:")
    controller = plant.get_controller("test_controller")
    controller.execute(target=[0.5, 0.0, 0.2])

    return plant


def main():
    """Run all examples."""
    print("\n🚀 ManipulationPlant Architecture Examples\n")

    try:
        example_1_basic_plant()
        example_2_with_objects()
        example_3_state_tracking()
        example_4_plant_visualization()
        example_5_mock_components()

        print("\n" + "=" * 60)
        print("✅ All examples completed successfully!")
        print("=" * 60)

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
