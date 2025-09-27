"""
## Key Mappings
| Key | Action |
|-----|--------|
| `9` | Toggle teleoperation On/Off. |
| `n` | Toggle between manual and non-manual mode. |
| `.` | Toggle between rotation and translation mode. |
| `8` | Cycle through different mocap data. |
| `+` | Increase movement step size or movement speed. |
| `-` | Decrease movement step size or movement speed. |
| **Arrow Keys** | **Move (rotation / translation) along the X, Y, and Z axes** |
| `Up` | Move forward (+X) or rotates around X-axis in positive direction. |
| `Down` | Move backward (-X) or rotates around X-axis in negative direction. |
| `Right` | Move right (+Y) or rotates around Y-axis in positive direction. |
| `Left` | Move left (-Y) or rotates around Y-axis in negative direction. |
| `7` | Move up (+Z) or rotates around Z-axis in positive direction. |
| `6` | Move down (-Z) or rotates around Z-axis in negative direction. |

---

## Modes
### **Manual vs. Non-Manual Mode:**
- **Manual Mode**: Iterative movement using arrow keys.
- **Non-Manual Mode**: Continuous movement using arrow keys (to stop continuous movement, re-click the arrow key).

### **Rotation vs. Translation Mode:**
- **Rotation Mode**: Rotation around an axis.
- **Translation Mode**: Movement along an axis.

Keyword arguments:
argument -- description
Return: return_description
"""

import mujoco
import numpy as np
import time
import mujoco.viewer
from loader import MuJoCoSceneLoader
from scene import MuJoCoSceneManager
from assets import ROOT_PATH

from loop_rate_limiters import RateLimiter

import mink
from mink.contrib import TeleopMocap

loader = MuJoCoSceneLoader(
    robot_dir="trs_so_arm100",
    env_dir=ROOT_PATH / "trs_so_arm100",
)
xml_path = loader.env_dir / "scene_with_table.xml"
scene_manager = MuJoCoSceneManager(
    xml_path=xml_path, loader=loader, load_method="xml_path"
)
model = scene_manager.model
data = scene_manager.data


configuration = mink.Configuration(model, model.key("home").qpos)

tasks = [
    end_effector_task := mink.FrameTask(
        frame_name="attachment_site",
        frame_type="site",
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1e-6,
    ),
    posture_task := mink.PostureTask(model, cost=1e-3),
]

# Enable collision avoidance between (moving_finger, table).
moving_finger = mink.get_body_geom_ids(model, model.body("Moving_Jaw").id)
cube = mink.get_body_geom_ids(model, model.body("cube").id)
collision_pairs = [
    (moving_finger, ["table"]),
    (cube, ["table"]),
]

limits = [
    mink.ConfigurationLimit(model=configuration.model),
    mink.CollisionAvoidanceLimit(
        model=configuration.model,
        geom_pairs=collision_pairs,
    ),
]

max_velocities = {
    "Rotation": np.pi,
    "Pitch": np.pi,
    "Elbow": np.pi,
    "Wrist_Pitch": np.pi,
    "Wrist_Roll": np.pi,
    "Jaw": np.pi,
}
velocity_limit = mink.VelocityLimit(model, max_velocities)
limits.append(velocity_limit)

## =================== ##

# mid = model.body("target").mocapid[0]

# IK settings.
solver = "daqp"
pos_threshold = 1e-4
ori_threshold = 1e-4
max_iters = 100

# Initialize key_callback function.
key_callback = TeleopMocap(data)

with mujoco.viewer.launch_passive(
    model=model,
    data=data,
    show_left_ui=True,
    show_right_ui=False,
    key_callback=key_callback,
) as viewer:
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)

    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    configuration.update(data.qpos)
    mujoco.mj_forward(model, data)

    posture_task.set_target_from_configuration(configuration)

    # Initialize the mocap target at the end-effector site.
    mink.move_mocap_to_frame(model, data, "target", "attachment_site", "site")

    rate = RateLimiter(frequency=200.0, warn=False)
    while viewer.is_running():
        # Update task target.
        T_wt = mink.SE3.from_mocap_name(model, data, "target")
        end_effector_task.set_target(T_wt)

        # Continuously check for autonomous key movement.
        key_callback.auto_key_move()

        # Compute velocity and integrate into the next configuration.
        for i in range(max_iters):
            vel = mink.solve_ik(
                configuration, tasks, rate.dt, solver, limits=limits
            )
            configuration.integrate_inplace(vel, rate.dt)
            err = end_effector_task.compute_error(configuration)
            pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
            ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold
            if pos_achieved and ori_achieved:
                break
        print(f"IK iters: {i}, pos err: {err[:3]}, ori err: {err[3:]}")
        data.ctrl = configuration.q[:6]
        mujoco.mj_step(model, data)

        # Visualize at fixed FPS.
        viewer.sync()
        rate.sleep()
