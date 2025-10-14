from pathlib import Path
import numpy as np

try:
    from fullstack_manip.control.high_level.pick_place_controller import (
        PickPlaceController,
    )
    from fullstack_manip.core.robot import Robot
    from fullstack_manip.simulation.asset_manager import ROOT_PATH
    from fullstack_manip.simulation.loader import MuJoCoSceneLoader
    from fullstack_manip.simulation.scene import MuJoCoSceneManager
except ModuleNotFoundError:  # pragma: no cover - script execution fallback
    import sys

    PROJECT_ROOT = Path(__file__).resolve().parents[1]
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))

    from fullstack_manip.control.high_level.pick_place_controller import (
        PickPlaceController,
    )
    from fullstack_manip.core.robot import Robot
    from fullstack_manip.simulation.asset_manager import ROOT_PATH
    from fullstack_manip.simulation.loader import MuJoCoSceneLoader
    from fullstack_manip.simulation.scene import MuJoCoSceneManager


# Load scene
loader = MuJoCoSceneLoader(
    robot_dir="trs_so_arm100",
    env_dir=ROOT_PATH / "trs_so_arm100",
)
xml_path = loader.env_dir / "scene_with_table.xml"
scene_manager = MuJoCoSceneManager(
    xml_path=xml_path, loader=loader, load_method="xml_path"
)

# Initialize parameters
end_effector_name = "attachment_site"
end_effector_type = "site"
object_name = "cube"
gripper_joint_names = ["Jaw"]
gripper_bodies = ["Fixed_Jaw", "Moving_Jaw"]
obstacles = ["table"]

# Initialize robot and controller
robot = Robot(
    scene_manager.model,
    scene_manager.data,
    end_effector_name=end_effector_name,
    end_effector_type=end_effector_type,
    gripper_bodies=gripper_bodies,
    obstacles=obstacles,
)
controller = PickPlaceController(
    robot,
    gripper_joint_names=gripper_joint_names,
    object_geom=object_name,
)


def main():
    # Define pick and place positions
    pick_position = robot.get_body_pose(object_name)[0]
    place_position = pick_position + np.array([0.1, 0.1, 0.05])

    # Perform pick
    print("Starting pick operation...")
    print(
        "Current end-effector position:",
        robot.get_body_pose(robot.end_effector_name)[0],
    )
    print("Pick position:", pick_position)
    controller.pick_object(pick_position)

    # Place
    print("Starting place operation...")
    controller.place_object(place_position)

    print("Pick and place completed successfully.")


if __name__ == "__main__":
    main()
