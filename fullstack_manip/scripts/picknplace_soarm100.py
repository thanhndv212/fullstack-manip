import numpy as np
from robot import Robot
from pick_place_controller import PickPlaceController
from scene import MuJoCoSceneManager
from loader import MuJoCoSceneLoader
from assets import ROOT_PATH
from viewer import MuJoCoViewer


# Load scene
loader = MuJoCoSceneLoader(
    robot_dir="trs_so_arm100",
    env_dir=ROOT_PATH / "trs_so_arm100",
)
xml_path = loader.env_dir / "scene_with_table.xml"
scene_manager = MuJoCoSceneManager(
    xml_path=xml_path, loader=loader, load_method="xml_path"
)


# Initialize robot and controller
robot = Robot(scene_manager.model, scene_manager.data, end_effector_name="attachment_site")
controller = PickPlaceController(
    robot, gripper_joint_names=["Jaw"], object_geom="cube"
)


def main():
    
    # Define pick and place positions
    pick_position = np.array([0.1, -0.2, 0.1])
    place_position = np.array([0.1, -0.21, 0.1])

    # Perform pick
    print("Starting pick operation...")
    controller.pick_object(pick_position)

    # Place
    print("Starting place operation...")
    controller.place_object(place_position)

    print("Pick and place completed successfully.")

if __name__ == "__main__":
    main()
