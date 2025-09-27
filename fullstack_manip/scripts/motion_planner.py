import numpy as np
from scipy.interpolate import CubicSpline
import mink
import mujoco

from loop_rate_limiters import RateLimiter

# IK settings.
SOLVER = "daqp"
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 100
RATE = RateLimiter(frequency=100.0, warn=False)


class MotionPlanner:

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        end_effector_name="Fixed_Jaw",
        max_velocity=0.5,
        max_acceleration=1.0,
        dt=0.01,
    ):
        """
        Initialize motion planner with MuJoCo model and Mink integration

        Args:
            model: MuJoCo model
            data: MuJoCo data
            end_effector_name: Name of the end effector (default: attachment_site)
            max_velocity: Maximum allowed velocity
            max_acceleration: Maximum allowed acceleration
            dt: Time step for trajectory
        """
        self.model = model
        self.data = data
        self.end_effector_name = end_effector_name
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.dt = dt
        # Initialize Mink configuration
        self.configuration = mink.Configuration(self.model, self.model.key("home").qpos)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.data.qpos[:] = self.model.key("home").qpos
        mujoco.mj_forward(self.model, self.data)
        self.tasks = []
        self.set_limits()

    def set_limits(self, collision_pairs: list[tuple[str, str]] = None):
        # Enable collision avoidance between (fixed_finger, table).
        fixed_finger = mink.get_body_geom_ids(self.model, self.model.body("Fixed_Jaw").id)
        cube = mink.get_body_geom_ids(self.model, self.model.body("cube").id)
        collision_pairs = [
            (fixed_finger, ["table"]),
            (fixed_finger, cube),
            (cube, ["table"]),
        ]
        limits = [
            mink.ConfigurationLimit(model=self.configuration.model),
            mink.CollisionAvoidanceLimit(
                model=self.configuration.model,
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
        velocity_limit = mink.VelocityLimit(self.model, max_velocities)
        limits.append(velocity_limit)
        self.limits = limits

    def plan_trajectory(
        self, start_pos, end_pos, start_vel=None, end_vel=None, duration=4.0
    ):
        """
        Plan a smooth trajectory using Mink for IK waypoints

        Args:
            start_pos: Starting position (array)
            end_pos: Ending position (array)
            start_vel: Starting velocity (optional)
            end_vel: Ending velocity (optional)

        Returns:
            Trajectory as array of joint positions over time
        """
        # Solve IK for start and end poses
        start_joints = self.solve_ik_for_pose(start_pos)
        end_joints = self.solve_ik_for_pose(end_pos)

        # Generate trajectory in joint space
        distance = np.linalg.norm(end_joints - start_joints)
        duration = max(distance / self.max_velocity, duration) 

        num_steps = int(duration / self.dt)
        t = np.linspace(0, duration, num_steps)

        # Cubic spline in joint space for smoothness
        cs = CubicSpline([0, duration], [start_joints, end_joints], axis=0)
        trajectory = cs(t)

        return trajectory, t

    def solve_ik_for_pose(self, target_pos, target_orient=None):
        """Helper to solve IK for a pose using Mink"""
        tasks = []

        self.configuration.update(self.data.qpos)

        target_rotation = mink.SO3(target_orient) if target_orient is not None else mink.SO3.identity()

        target_pose = mink.SE3.from_rotation_and_translation(
            target_rotation,
            target_pos,
        )
        pose_task = mink.FrameTask(
            frame_name=self.end_effector_name,
            frame_type="body",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1e-6,
        )
        pose_task.set_target(target_pose)

        tasks.append(pose_task)

        IK_SUCCESS = False
        for i in range(MAX_ITERS):
            vel = mink.solve_ik(
                self.configuration, tasks, RATE.dt, solver=SOLVER, limits=self.limits
            )
            self.configuration.integrate_inplace(vel, RATE.dt)
            err = pose_task.compute_error(self.configuration)
            pos_achieved = np.linalg.norm(err[:3]) <= POS_THRESHOLD
            ori_achieved = np.linalg.norm(err[3:]) <= ORI_THRESHOLD
            if pos_achieved and ori_achieved:
                IK_SUCCESS = True
                break

        return self.configuration.q.copy()[:6]
        # if IK_SUCCESS:
        #     return self.configuration.q.copy()
        # else:
        #     raise ValueError("IK failed for pose")

    def check_contact_pair(self, geom1_name: str, geom2_name: str) -> bool:
        """
        Check if two specific geometries are in contact

        Args:
            geom1_name: Name of first geometry
            geom2_name: Name of second geometry

        Returns:
            True if in contact (penetration), False otherwise
        """
        geom1_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, geom1_name
        )
        geom2_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, geom2_name
        )
        for contact in self.data.contact[: self.data.ncon]:
            if (contact.geom1 == geom1_id and contact.geom2 == geom2_id) or (
                contact.geom1 == geom2_id and contact.geom2 == geom1_id
            ):
                if contact.dist < 0:  # Penetration indicates contact
                    return True
        return False

    def check_collision(self, trajectory, obstacles):
        """
        Check trajectory for collisions using MuJoCo

        Args:
            trajectory: Planned trajectory (joint positions)
            obstacles: List of obstacle geometry names

        Returns:
            True if collision-free, False otherwise
        """
        for joints in trajectory:
            self.data.qpos[:] = joints
            mujoco.mj_forward(self.model, self.data)
            # Check for collisions with obstacles (generalized to pairs)
            for obstacle in obstacles:
                if self.check_contact_pair(
                    "robot_geom", obstacle
                ):  # Assuming "robot_geom" is a representative geom
                    return False
        return True

    def plot_trajectory(self, trajectory, t):
        """
        Plot the planned trajectory in joint space

        Args:
            trajectory: Planned trajectory (joint positions)
            t: Time array corresponding to trajectory
        """
        import matplotlib.pyplot as plt

        plt.figure()
        for i in range(trajectory.shape[1]):
            plt.plot(t, trajectory[:, i], label=f"Joint {i+1}")
        plt.xlabel("Time (s)")
        plt.ylabel("Joint Position (rad)")
        plt.title("Planned Joint Trajectory")
        plt.legend()
        plt.grid()
        plt.show()
