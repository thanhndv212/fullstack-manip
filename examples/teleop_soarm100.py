"""Keyboard mappings for teleoperation.

Key bindings:
- 9: toggle teleoperation on/off
- n: switch manual/automatic mode
- .: toggle rotation vs. translation
- 8: cycle mocap targets
- +/-: adjust movement speed
- Arrow keys: move/rotate in the horizontal plane
- 7/6: move along positive/negative Z

Modes:
- Manual: discrete movement per key press
- Non-manual: continuous motion until the same key is pressed again
- Rotation mode: rotate around axes
- Translation mode: translate along axes
"""

import mujoco
import mujoco.viewer
import numpy as np

try:
    from fullstack_manip.simulation.asset_manager import ROOT_PATH
    from fullstack_manip.simulation.loader import MuJoCoSceneLoader
    from fullstack_manip.simulation.scene import MuJoCoSceneManager
except ModuleNotFoundError:  # pragma: no cover - script execution fallback
    import sys
    from pathlib import Path

    PROJECT_ROOT = Path(__file__).resolve().parents[1]
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))

    from fullstack_manip.simulation.asset_manager import ROOT_PATH
    from fullstack_manip.simulation.loader import MuJoCoSceneLoader
    from fullstack_manip.simulation.scene import MuJoCoSceneManager

from fullstack_manip.utils.loop_rate_limiters import RateLimiter

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
        orientation_cost=0.0,
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

# ===================

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
