import numpy as np
from scipy.interpolate import CubicSpline
import mujoco

from fullstack_manip.core.collision import CollisionChecker
from fullstack_manip.core.ik import IKSolver
from fullstack_manip.core.limit import LimitManager


class MotionPlanner:

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        end_effector_name=None,
        end_effector_type=None,
        gripper_bodies=None,
        obstacles=None,
        max_velocity=0.5,
        max_acceleration=1.0,
        dt=0.01,
    ):
        """
        Initialize motion planner with MuJoCo model and Mink integration

        Args:
            model: MuJoCo model
            data: MuJoCo data
            end_effector_name: Name of the end effector
            end_effector_type: Type of the end effector
            gripper_bodies: List of gripper body names for collision checking
            obstacles: List of obstacle geometry names for collision checking
            max_velocity: Maximum allowed velocity
            max_acceleration: Maximum allowed acceleration
            dt: Time step for trajectory
        """
        self.model = model
        self.data = data
        self.end_effector_name = end_effector_name
        self.end_effector_type = end_effector_type
        self.gripper_bodies = gripper_bodies
        self.obstacles = obstacles
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.dt = dt

        # Initialize core modules
        self.collision_checker = CollisionChecker(model, data)
        self.limit_manager = LimitManager(model, gripper_bodies, obstacles)
        self.limits = self.limit_manager.set_limits()
        self.ik_solver = IKSolver(model, data, self.limits)

        self.tasks = []

    def plan_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        start_orient: np.ndarray | None = None,
        end_orient: np.ndarray | None = None,
        start_vel: np.ndarray | None = None,
        end_vel: np.ndarray | None = None,
        duration: float = 4.0,
        solve_for_startpos: bool = False,
    ):
        """
        Plan a smooth trajectory using Mink for IK waypoints

        Args:
            start_pos: Starting position (array)
            end_pos: Ending position (array)
            start_orient: Starting orientation (optional)
            end_orient: Ending orientation (optional)
            start_vel: Starting velocity (optional)
            end_vel: Ending velocity (optional)
            duration: Desired duration of the trajectory
            solve_for_startpos: Whether to solve IK for start position

        Returns:
            Trajectory as array of joint positions over time
        """
        # Solve IK for start and end poses
        if solve_for_startpos:
            start_joints = self.solve_ik_for_qpos(
                self.end_effector_name,
                self.end_effector_type,
                start_pos,
                start_orient,
            )
        else:
            start_joints = self.data.qpos[:6]
        end_joints = self.solve_ik_for_qpos(
            self.end_effector_name, self.end_effector_type, end_pos, end_orient
        )

        # Generate trajectory in joint space
        distance = np.linalg.norm(end_joints - start_joints)
        duration = max(distance / self.max_velocity, duration)

        num_steps = int(duration / self.dt)
        t = np.linspace(0, duration, num_steps)

        # Cubic spline in joint space for smoothness
        cs = CubicSpline([0, duration], [start_joints, end_joints], axis=0)
        trajectory = cs(t)

        return trajectory, t

    def solve_ik_for_qpos(
        self,
        frame_name: str,
        frame_type: str,
        target_pos: np.ndarray,
        target_orient: np.ndarray | None = None,
    ):
        """Helper to solve IK for a pose using Mink."""
        return self.ik_solver.solve_ik_for_qpos(
            frame_name, frame_type, target_pos, target_orient
        )

    def check_contact_pair(self, geom1_name: str, geom2_name: str) -> bool:
        """
        Check if two specific geometries are in contact

        Args:
            geom1_name: Name of first geometry
            geom2_name: Name of second geometry

        Returns:
            True if in contact (penetration), False otherwise
        """
        return self.collision_checker.check_contact_pair(
            geom1_name, geom2_name
        )

    def check_collision(self, trajectory, obstacles):
        """
        Check trajectory for collisions using MuJoCo

        Args:
            trajectory: Planned trajectory (joint positions)
            obstacles: List of obstacle geometry names

        Returns:
            True if collision-free, False otherwise
        """
        return self.collision_checker.check_collision(trajectory, obstacles)

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
            plt.plot(t, trajectory[:, i], label=f"Joint {i + 1}")
        plt.xlabel("Time (s)")
        plt.ylabel("Joint Position (rad)")
        plt.title("Planned Joint Trajectory")
        plt.legend()
        plt.grid()
        plt.show()
