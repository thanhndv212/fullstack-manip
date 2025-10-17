"""Robot class for manipulation tasks.

Focused on core robot functionality:
- Kinematics (FK/IK via external planner)
- Dynamics
- Collision detection
- Gripper control
- State management

Motion planning, control strategies, and task planning are external components.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import mujoco
import numpy as np

from ..simulation.viewer import MuJoCoViewer
from .collision import CollisionChecker
from .gripper import Gripper
from ..utils.rate_config import RateConfig, load_rate_config


class Robot:
    """
    Robot instance with core manipulation capabilities.

    Responsibilities:
    - Joint position/velocity control
    - Forward kinematics (body poses)
    - Collision detection
    - Gripper management
    - Workspace computation (with external IK)

    Does NOT handle:
    - Motion planning (use MotionPlanner)
    - High-level control (use Controllers)
    - State tracking (use StateManager)
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: Optional[mujoco.MjData] = None,
        end_effector_name: str = None,
        end_effector_type: str = None,
        gripper_bodies: List[str] = None,
        obstacles: List[str] = None,
        motion_planner=None,  # Deprecated, kept for backward compatibility
        rate_config: Optional[RateConfig] = None,
    ):
        try:
            self.rates = rate_config or load_rate_config(defaults=RateConfig())
            self.model = model
            self.data = mujoco.MjData(self.model) if data is None else data
            self.end_effector_name = end_effector_name
            self.end_effector_type = end_effector_type
            self.gripper_bodies = gripper_bodies
            self.obstacles = obstacles

            # Warn about deprecated parameter
            if motion_planner is not None:
                import warnings

                warnings.warn(
                    "motion_planner parameter is deprecated. "
                    "Use ManipulationPlant to manage components.",
                    DeprecationWarning,
                    stacklevel=2,
                )

            # Initialize collision checker
            self.collision_checker = CollisionChecker(self.model, self.data)

            # Initialize gripper
            self.gripper = Gripper(
                model=self.model,
                data=self.data,
                collision_checker=self.collision_checker,
                viewer=None,  # Will be set after viewer initialization
                end_effector_name=self.end_effector_name,
                gripper_bodies=self.gripper_bodies,
                dt=self.rates.control,
            )

            # Initialize viewer
            self.viewer = MuJoCoViewer(self.model, self.data)
            self.viewer.launch_passive()
            self.viewer.sync()

            # Set viewer for gripper after initialization
            self.gripper.viewer = self.viewer

            # Robot properties
            self.scene_nq = self.model.nq
            self.scene_nu = self.model.nu
            self.robot_nq = self.model.nu
            self.sim_dt = self.rates.sim
            self.control_dt = self.rates.control
            self.planner_dt = self.rates.planner
            self.estimator_dt = self.rates.estimator
            self.sensor_dt = self.rates.sensor
            self.dt = self.control_dt

            # Set initial configuration
            self.data.qpos[:] = self.model.key("home").qpos
            mujoco.mj_forward(self.model, self.data)

        except Exception as err:  # pragma: no cover - defensive
            raise RuntimeError(f"Failed to initialize robot: {err}")

    def get_robot_joint_positions(self) -> Optional[np.ndarray]:
        """Get robot joint positions (assumes first robot_nq joints)."""
        if self.data:
            return self.data.qpos[: self.robot_nq]
        return None

    def set_robot_joint_positions(self, joint_positions) -> None:
        """Set robot joint positions (assumes first robot_nq joints)."""
        if self.data:
            self.data.qpos[: self.robot_nq] = joint_positions
            mujoco.mj_forward(self.model, self.data)

    def get_body_pose(self, body_name: str) -> Tuple[np.ndarray, np.ndarray]:
        """Get body position and orientation."""
        mujoco.mj_forward(self.model, self.data)
        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_name,
        )
        pos = self.data.xpos[body_id].copy()
        quat = self.data.xquat[body_id].copy()
        return pos, quat

    # Gripper control methods - delegate to Gripper object
    def check_grasp_success(self) -> bool:
        """Check if grasp is successful."""
        return self.gripper.check_grasp_success()

    def check_grasp_contact(self) -> bool:
        """Check if the grasp is successful based on contact forces."""
        return self.gripper.check_grasp_contact()

    def check_in_hand(self) -> bool:
        """Check if the object is still in hand based on relative position."""
        return self.gripper.check_in_hand()

    def _close_gripper(self) -> None:
        """Close robot gripper and check for successful grasp."""
        self.gripper.close(check_grasp=True)

    def _open_gripper(self) -> None:
        """Open robot gripper and check for successful release."""
        self.gripper.open(check_release=True)

    def compute_workspace(
        self,
        ik_solver,
        grid_size: int = 20,
        grid_range: float = 0.4,
        *,
        max_points: int = 4000,
        tolerance: float = 5e-3,
        center: Optional[np.ndarray] = None,
        seed: Optional[int] = 0,
    ) -> np.ndarray:
        """
        Estimate the robot's reachable workspace via inverse kinematics.

        Args:
            ik_solver: IK solver with solve_ik_for_qpos method
            grid_size: Number of points per axis
            grid_range: Range from center in meters
            max_points: Maximum points to sample
            tolerance: Distance tolerance for IK solution
            center: Center point for workspace (default: current EE pose)
            seed: Random seed for sampling

        Returns:
            Array of reachable points
        """
        if grid_size < 2:
            raise ValueError("grid_size must be at least 2")
        grid_size = min(grid_size, 60)
        grid_range = abs(grid_range)
        if center is None:
            center, _ = self.get_body_pose(self.end_effector_name)
        center = np.asarray(center, dtype=float)

        axis = np.linspace(-grid_range, grid_range, grid_size)
        offsets = np.stack(
            np.meshgrid(axis, axis, axis, indexing="ij"),
            axis=-1,
        )
        offsets = offsets.reshape(-1, 3)

        if max_points is not None and offsets.shape[0] > max_points:
            rng = np.random.default_rng(seed)
            selection = rng.choice(
                offsets.shape[0],
                size=max_points,
                replace=False,
            )
            offsets = offsets[selection]

        targets = center[None, :] + offsets

        reachable_points: list[np.ndarray] = []
        initial_joint_positions = self.get_robot_joint_positions().copy()
        seed_configuration = initial_joint_positions.copy()

        for target in targets:
            self.set_robot_joint_positions(seed_configuration)
            ik_solution = ik_solver.solve_ik_for_qpos(
                self.end_effector_name, self.end_effector_type, target
            )
            if ik_solution is None:
                continue

            candidate_qpos = seed_configuration.copy()
            dof = min(candidate_qpos.shape[0], ik_solution.shape[0])
            candidate_qpos[:dof] = ik_solution[:dof]
            self.set_robot_joint_positions(candidate_qpos)

            ee_pos, _ = self.get_body_pose(self.end_effector_name)
            if np.any(np.isnan(ee_pos)):
                continue
            if np.linalg.norm(ee_pos - target) > tolerance:
                continue

            reachable_points.append(ee_pos.copy())
            seed_configuration = candidate_qpos

        self.set_robot_joint_positions(initial_joint_positions)

        if not reachable_points:
            return np.empty((0, 3), dtype=float)

        points = np.asarray(reachable_points, dtype=float)
        rounded = np.round(points / tolerance).astype(int)
        _, unique_indices = np.unique(rounded, axis=0, return_index=True)
        return points[np.sort(unique_indices)]

    def visualize_workspace(self, points: np.ndarray) -> None:
        """Visualize the robot's reachable workspace in the viewer."""
        if self.viewer is None:
            raise RuntimeError("Viewer not initialized")

        mujoco.mj_step(self.model, self.data)
        while self.viewer.is_running():
            scene = getattr(self.viewer, "user_scn", None)
            if scene is None:
                raise RuntimeError("Viewer scene not available")

            scene.ngeom = 0
            rgba = np.array([0.0, 1.0, 0.0, 0.5], dtype=float)
            size = np.array([0.002, 0.0, 0.0], dtype=float)

            for i, point in enumerate(points):
                mujoco.mjv_initGeom(
                    scene.geoms[i],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=size,
                    pos=np.asarray(point, dtype=float),
                    rgba=rgba,
                )

            self.viewer.sync()


__all__ = ["Robot"]
