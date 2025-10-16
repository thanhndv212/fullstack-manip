"""Example: Pick-and-Place using the new modular architecture.

This example shows how to build a complete pick-and-place system
using the ManipulationPlant architecture with real components.
"""

import numpy as np

from fullstack_manip.core import (
    Gripper,
    ManipulationPlant,
    ObjectManager,
    ObjectType,
    Robot,
    StateManager,
)


# Mock MotionPlanner for this example
class MockMotionPlanner:
    """Mock motion planner for example."""

    def __init__(self, robot):
        self.robot = robot

    def plan_to_pose(self, position, orientation):
        """Mock planning - returns dummy trajectory."""
        return {"positions": [position], "times": [1.0]}


class PickPlaceTask:
    """High-level pick-and-place task using ManipulationPlant."""

    def __init__(self, plant: ManipulationPlant):
        """Initialize task.

        Args:
            plant: ManipulationPlant with all necessary components
        """
        self.plant = plant
        self.robot = plant.robot
        self.gripper = self.robot.gripper
        self.motion_planner = plant.motion_planner
        self.state_manager = plant.state_manager
        self.object_manager = plant.object_manager

    def execute(self, object_name: str, target_position: np.ndarray) -> bool:
        """Execute pick-and-place task.

        Args:
            object_name: Name of object to pick
            target_position: Target position to place object

        Returns:
            True if task succeeded, False otherwise
        """
        print(f"\n{'='*60}")
        print(f"Pick-and-Place Task: {object_name}")
        print(f"{'='*60}")

        # 1. Find object
        obj = self.object_manager.get_object(object_name)
        if obj is None:
            print(f"❌ Object '{object_name}' not found")
            return False

        print(f"✓ Found object at {obj.pose['position']}")

        # 2. Plan approach
        grasp_points = obj.get_grasp_points()
        if not grasp_points:
            print("❌ No grasp points available")
            return False

        approach_pos = grasp_points[0] + np.array([0, 0, 0.1])
        print(f"✓ Planned approach to {approach_pos}")

        # 3. Move to pre-grasp
        self.gripper.open()
        print("✓ Opened gripper")

        # Use motion planner to plan path
        if self.motion_planner:
            trajectory = self.motion_planner.plan_to_pose(
                approach_pos, np.array([1, 0, 0, 0])
            )
            if trajectory is not None:
                print("✓ Executed approach trajectory")
            else:
                print("❌ Failed to plan approach")
                return False

        # 4. Move to grasp
        grasp_pos = grasp_points[0]
        print(f"✓ Moved to grasp position {grasp_pos}")

        # 5. Close gripper
        self.gripper.close()
        success = self.gripper.check_grasp_success()

        if success:
            print("✓ Grasped object successfully")
            obj.set_grasped(True)
            self.state_manager.update_object_state(
                name=object_name,
                position=grasp_pos,
                grasped=True,
            )
        else:
            print("❌ Failed to grasp object")
            return False

        # 6. Lift
        lift_pos = grasp_pos + np.array([0, 0, 0.15])
        print(f"✓ Lifted to {lift_pos}")

        # 7. Move to target
        target_approach = target_position + np.array([0, 0, 0.1])
        print(f"✓ Moved to target approach {target_approach}")

        # 8. Lower to target
        print(f"✓ Lowered to target {target_position}")

        # 9. Release
        self.gripper.open()
        obj.set_grasped(False)
        obj.set_pose(target_position, np.array([1, 0, 0, 0]))
        self.state_manager.update_object_state(
            name=object_name,
            position=target_position,
            grasped=False,
        )
        print("✓ Released object")

        # 10. Retreat
        retreat_pos = target_position + np.array([0, 0, 0.15])
        print(f"✓ Retreated to {retreat_pos}")

        print("\n✅ Pick-and-place completed successfully!")
        return True


def create_pickplace_plant(model, data, model_path: str) -> ManipulationPlant:
    """Create a manipulation plant for pick-and-place.

    Args:
        model: MuJoCo model
        data: MuJoCo data
        model_path: Path to robot URDF/XML

    Returns:
        Configured ManipulationPlant
    """
    print("Building pick-and-place plant...")

    # 1. Create components
    robot = Robot(
        model=model,
        data=data,
        model_path=model_path,
        base_link="base_link",
        ee_link="end_effector",
    )

    # Configure gripper
    gripper = Gripper(model, data)
    gripper.set_joint_names(["gripper_left", "gripper_right"])
    gripper.set_positions(open_pos=0.04, closed_pos=0.0)
    gripper.set_force_thresholds(min_force=0.1, max_force=10.0)
    robot.gripper = gripper

    motion_planner = MockMotionPlanner(robot)

    state_manager = StateManager()
    object_manager = ObjectManager(model, data)

    # 2. Build plant using builder pattern
    plant = (
        ManipulationPlant.builder()
        .with_name("pickplace_plant")
        .with_robot(robot)
        .with_motion_planner(motion_planner)
        .with_state_manager(state_manager)
        .with_object_manager(object_manager)
        .build()
    )

    print("✓ Plant built successfully")
    return plant


def register_scene_objects(plant: ManipulationPlant) -> None:
    """Register objects in the scene.

    Args:
        plant: ManipulationPlant to register objects in
    """
    obj_mgr = plant.object_manager

    # Register cube
    cube = obj_mgr.register_object(
        name="red_cube",
        object_type=ObjectType.BOX,
    )
    cube.set_pose(
        position=np.array([0.5, 0.0, 0.05]),
        orientation=np.array([1, 0, 0, 0]),
    )
    cube.properties.size = [0.05, 0.05, 0.05]
    cube.properties.mass = 0.1
    cube.properties.graspable = True
    cube.properties.color = [1, 0, 0, 1]

    # Register cylinder
    cylinder = obj_mgr.register_object(
        name="blue_cylinder",
        object_type=ObjectType.CYLINDER,
    )
    cylinder.set_pose(
        position=np.array([0.4, 0.2, 0.05]),
        orientation=np.array([1, 0, 0, 0]),
    )
    cylinder.properties.size = [0.03, 0.06]  # radius, height
    cylinder.properties.mass = 0.08
    cylinder.properties.graspable = True
    cylinder.properties.color = [0, 0, 1, 1]

    print(f"✓ Registered {len(obj_mgr.get_all_objects())} objects")


def main():
    """Main example demonstrating pick-and-place with new architecture."""
    print("\n🤖 Pick-and-Place with Modular Architecture\n")

    # Note: This is a template showing the architecture usage
    # In practice, you would load a real MuJoCo model
    print("NOTE: This is a template example.")
    print("To run with real simulation:")
    print("  1. Load MuJoCo model/data")
    print("  2. Create plant with create_pickplace_plant()")
    print("  3. Register scene objects")
    print("  4. Execute tasks")
    print()

    # Example pseudo-code:
    example_code = """
# Load MuJoCo model
import mujoco
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

# Create plant
plant = create_pickplace_plant(model, data, "robot.xml")

# Register objects
register_scene_objects(plant)

# Create and execute task
task = PickPlaceTask(plant)

# Pick red cube and place it
success = task.execute(
    object_name="red_cube",
    target_position=np.array([0.3, -0.2, 0.05])
)

# Pick blue cylinder and place it
success = task.execute(
    object_name="blue_cylinder",
    target_position=np.array([0.6, 0.1, 0.05])
)
"""

    print("Example usage:")
    print(example_code)

    # Show plant structure
    print("\n" + "=" * 60)
    print("Architecture Benefits:")
    print("=" * 60)
    print(
        """
1. Modular Components:
   - Robot: Kinematics, collision detection
   - Gripper: Grasp control and verification
   - MotionPlanner: Path planning
   - StateManager: State tracking with observers
   - ObjectManager: Scene understanding

2. Easy Testing:
   - Mock any component for unit tests
   - Test components in isolation
   - Verify integration incrementally

3. Flexible Configuration:
   - Swap motion planners (RRT, RRT*, PRM)
   - Try different controllers
   - Add sensors dynamically
   - Configure via YAML (future)

4. Clear Responsibilities:
   - Robot: "What can I do physically?"
   - Planner: "How should I move?"
   - Controller: "How do I execute the plan?"
   - Task: "What is my goal?"

5. Extensibility:
   - Add new components via Protocol interfaces
   - No need to modify existing code
   - Plugin architecture for custom behaviors
    """
    )


if __name__ == "__main__":
    main()
