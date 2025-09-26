import mujoco
import numpy as np
import time
import mujoco.viewer
from loader import MuJoCoSceneLoader
from scene import MuJoCoSceneManager
from assets import ROOT_PATH


# Example usage
loader = MuJoCoSceneLoader(
    robot_dir="universal_robots_ur5e",
    gripper_dir="robotiq_2f85",
    env_dir=ROOT_PATH / "ur5e",
)

# 1/ Load the basic robot model from robot_descriptions
# model = loader.load_scene("robot_descriptions", robot_name="ur5e")

# # 2/ Load a scene model partially from mujoco_menagerie
xml_path = loader.env_dir / "xmls/ur5e_robotiq2f85.xml"
# model = loader.load_scene("xml_string", xml_path=xml_path)

# # 3/ Load the scene model directly from an absolute path
# xml_path = loader.env_dir / "scene_ur5e_rg2_d435i.xml"
# model = loader.load_scene("xml_path", xml_path=xml_path)

# data = mujoco.MjData(model)

# create scene manager
scene_manager = MuJoCoSceneManager(xml_path=xml_path, loader=loader, load_method="xml_string")
model = scene_manager.model
data = scene_manager.data

# Define a simple joint trajectory (sinusoidal motion for all joints)
t = np.linspace(0, 5, 500)  # 5 seconds, 500 steps
q_traj = np.zeros((len(t), model.nq))
for j in range(model.nq):
    q_traj[:, j] = 0.3 * np.sin(
        2 * np.pi * t / 5 + j * np.pi / 6
    )  # Phase shift for each joint

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        for i in range(len(t)):
            data.qpos[:] = q_traj[i]
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.01)  # ~100Hz
