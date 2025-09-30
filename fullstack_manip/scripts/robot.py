import mujoco
import numpy as np
import mink
from typing import List, Optional, Tuple
from motion_planner import MotionPlanner
from pid_controller import PIDController
from viewer import MuJoCoViewer
import time


class Robot:

    def __init__(
        self,
        model: mujoco.MjModel,
        data: Optional[mujoco.MjData] = None,
        end_effector_name: str = "Fixed_Jaw",
    ):
        try:
            self.model = model
            self.data = mujoco.MjData(self.model)
            self.end_effector_name = end_effector_name
            self.gripper_bodies = ["Fixed_Jaw", "Moving_Jaw"]
            # Initialize motion planner
            self.motion_planner = MotionPlanner(
                self.model,
                self.data,
                self.end_effector_name,
            )
            # Initialize PID controller
            self.pid_controller = PIDController(
                kp=10.0,
                ki=0.1,
                kd=1.0,
                dt=0.01,
                integral_limit=1.0,
                output_limit=10.0,
            )
            # Initialize viewer
            self.viewer = MuJoCoViewer(self.model, self.data)
            self.viewer.launch_passive()
            self.viewer.sync()
            # Robot and scene dimensions
            self.scene_nq = self.model.nq
            self.scene_nu = self.model.nu
            self.robot_nq = self.model.nu
            self.dt = 0.01
        except Exception as e:
            raise RuntimeError(f"Failed to initialize robot: {e}")

    def get_robot_joint_positions(self):
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
        """Get body position and orientation"""
        mujoco.mj_forward(self.model, self.data)
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        pos = self.data.xpos[body_id].copy()
        rot = self.data.xmat[body_id].reshape(3, 3).copy()
        return pos, rot

    def move_to_position(
        self, target_pos: np.ndarray, duration: float = 4.0
    ) -> None:
        """Move robot end-effector to target position using Mink IK"""

        if not isinstance(target_pos, np.ndarray) or target_pos.shape != (3,):
            raise ValueError("Target position must be a 3D numpy array")

        current_ee_pos, _ = self.get_body_pose(self.end_effector_name)

        # Plan trajectory
        trajectory, t = self.motion_planner.plan_trajectory(
            current_ee_pos, target_pos, duration=duration
        )

        if trajectory is None:
            raise RuntimeError("Failed to plan trajectory")

        current_joint_positions = self.get_robot_joint_positions()
        # Execute trajectory with PID control
        count = 0
        for target in trajectory:
            target_joint_positions = target
            control_signal = self.pid_controller.compute(
                target_joint_positions, current_joint_positions
            )
            # Apply control signal (assuming direct position control for simplicity)
            current_joint_positions += control_signal * self.dt
            self.viewer.step(current_joint_positions)
            time.sleep(self.dt)
            count += 1
            if self.detect_contact(self.end_effector_name, self.object_geom):
                print("Contact detected with cube, stopping movement.")
                break
        print(f"Executed {count} control steps to reach target position.")

    def get_body_geom_ids(self, model: mujoco.MjModel, body_id: int) -> List[int]:
        """Get immediate geoms belonging to a given body.

        Here, immediate geoms are those directly attached to the body and not its
        descendants.

        Args:
            model: Mujoco model.
            body_id: ID of body.

        Returns:
            A list containing all body geom ids.
        """
        geom_start = model.body_geomadr[body_id]
        geom_end = geom_start + model.body_geomnum[body_id]
        return list(range(geom_start, geom_end))

    def compute_contact_force(self, geom1_name: str or List[int], geom2_name: str or List[int]) -> float:
        """
        Compute the total contact force magnitude between two geometries

        Args:
            geom1_name: Name of first geometry or list of geom ids
            geom2_name: Name of second geometry or list of geom ids

        Returns:
            Total force magnitude
        """
        geom1_ids = (
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom1_name)]
            if isinstance(geom1_name, str)
            else geom1_name
        )
        geom2_ids = (
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom2_name)]
            if isinstance(geom2_name, str)
            else geom2_name
        )
        print(f"Computing contact force between geoms {geom1_ids} and {geom2_ids}")
        total_force = 0.0
        for contact in self.data.contact[: self.data.ncon]:
            if (contact.geom1 in geom1_ids and contact.geom2 in geom2_ids) or (
                contact.geom1 in geom2_ids and contact.geom2 in geom1_ids
            ):
                total_force += np.linalg.norm(contact.frame[:3])  # Normal force component
        return total_force

    def check_grasp_success(self) -> bool:
        if self.check_grasp_contact() and self.check_in_hand():
            return True
        return False

    def check_grasp_contact(self) -> bool:
        """Check if the grasp is successful based on contact forces"""
        total_forces = {}
        for gripper_body in self.gripper_bodies:
            gripper_geoms = self.get_body_geom_ids(self.model, self.model.body(gripper_body).id)
            total_forces[gripper_body] = self.compute_contact_force(
                gripper_geoms, self.object_geom
            )
            print(f"Grasp contact forces of body {gripper_body}: {total_forces[gripper_body]}")
        # If both gripper body has contact force above threshold, consider grasp successful
        return all(force > self.grasp_force_threshold for force in total_forces.values())

    def check_in_hand(self) -> bool:
        """Check if the object is still in hand based on relative position between object and gripper"""
        object_pos, _ = self.get_body_pose(self.object_geom)
        gripper_pos, _ = self.get_body_pose(self.end_effector_name)
        distance = np.linalg.norm(object_pos - gripper_pos)
        print(f"Distance between object and gripper: {distance}")
        return distance < 0.02  # Threshold distance to consider "in hand"

    def _close_gripper(self) -> None:
        """Close robot gripper and check for successful grasp"""
        T = 0.5  # Time to close gripper
        # Set gripper joints to closed position
        grasp_count = 0
        for joint_name in self.gripper_joint_names:
            joint_id = self.model.joint(joint_name).id
            while abs(self.data.qpos[joint_id] - self.close_position) > 1e-3:
                current_gripper_position = self.data.qpos[joint_id]
                vel = (
                    self.close_position - current_gripper_position
                ) / T
                self.data.qpos[joint_id] = vel * self.dt + current_gripper_position
                current_joint_positions = self.get_robot_joint_positions()
                self.viewer.step(current_joint_positions)
                time.sleep(self.dt)

                # Check for grasp success: contact forces > threshold
                if self.check_grasp_success():
                    grasp_count += 1
                else:
                    grasp_count = 0  # Reset if not successful

                print(f"Grasp success count: {grasp_count}")
                if grasp_count == 100:
                    self.GRASP_SUCCESS = True
                    print("Grasp successful")
                    break

        if not self.GRASP_SUCCESS:
            print("Grasp failed")

    def _open_gripper(self) -> None:
        """Open robot gripper and check for successful release"""

        T = 0.5  # Time to close gripper
        # Set gripper joints to closed position
        for joint_name in self.gripper_joint_names:
            joint_id = self.model.joint(joint_name).id
            while abs(self.data.qpos[joint_id] - self.open_position) > 1e-3:
                current_gripper_position = self.data.qpos[joint_id]
                vel = (self.open_position - current_gripper_position) / T
                self.data.qpos[joint_id] = (
                    vel * self.dt + current_gripper_position
                )
                current_joint_positions = self.get_robot_joint_positions()
                self.viewer.step(current_joint_positions)
                time.sleep(self.dt)

        # Check for release success: contact forces < threshold
        total_force = 0.0
        for gripper_body in self.gripper_bodies:
            gripper_geoms = self.get_body_geom_ids(self.model, self.model.body(gripper_body).id)
            total_force += self.compute_contact_force(
                gripper_geoms, self.object_geom
            )
        if total_force < self.release_force_threshold:
            print("Release successful")
        else:
            print("Release failed")

    def detect_contact(self, geom1_name: str, geom2_name: str) -> bool:
        """
        Detect if two geometries are in contact

        Args:
            geom1_name: Name of first geometry
            geom2_name: Name of second geometry

        Returns:
            True if the geometries are in contact, False otherwise
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
                return True
        return False

    def compute_workspace(
        self,
        grid_size: int = 20,
        grid_range: float = 0.4,
        *,
        max_points: int = 4000,
        tolerance: float = 5e-3,
        center: Optional[np.ndarray] = None,
        seed: Optional[int] = 0,
    ) -> np.ndarray:
        """Estimate the robot's reachable workspace using inverse kinematics.

        The workspace is sampled within a cubic volume around the specified
        ``center`` (defaults to the current end-effector position). For each
        sampled Cartesian target the motion planner attempts to solve IK; the
        targets whose IK solutions reach within ``tolerance`` are considered
        reachable.

        Args:
            grid_size: Resolution per axis for the sampling grid. Values
                greater than 60 are automatically clipped to keep the sampling
                tractable.
            grid_range: Half-width of the cubic volume to sample (meters).
            max_points: Maximum number of Cartesian samples to evaluate when
                the full grid would become prohibitively large.
            tolerance: Maximum acceptable Cartesian error (meters) between the
                IK solution's end-effector position and the sampled target.
            center: Optional workspace center in world coordinates. When
                ``None`` the current end-effector position is used.
            seed: Optional random seed for down-sampling the grid.

        Returns:
            A ``(N, 3)`` array of reachable points in world coordinates.
        """
        if grid_size < 2:
            raise ValueError("grid_size must be at least 2")
        grid_size = min(grid_size, 60)
        grid_range = abs(grid_range)
        if center is None:
            center, _ = self.get_body_pose(self.end_effector_name)
        center = np.asarray(center, dtype=float)

        # Create Cartesian sampling grid (offsets around the center).
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
            # Reset seed for IK to the latest successful configuration to aid
            # convergence, but avoid mutating the actual configuration if the
            # current target is unreachable.
            self.set_robot_joint_positions(seed_configuration)
            ik_solution = self.motion_planner.solve_ik_for_pose(target)
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

        # Restore initial joint positions
        self.set_robot_joint_positions(initial_joint_positions)

        if not reachable_points:
            return np.empty((0, 3), dtype=float)

        points = np.asarray(reachable_points, dtype=float)
        # Remove near-duplicate points introduced by relaxed tolerance.
        rounded = np.round(points / tolerance).astype(int)
        _, unique_indices = np.unique(rounded, axis=0, return_index=True)
        return points[np.sort(unique_indices)]

    def visualize_workspace(self, points: np.ndarray) -> None:
        """
        Visualize the robot's reachable workspace in the viewer

        Args:
            points: A (N, 3) array of reachable points in 3D space
        """
        if self.viewer is None:
            raise RuntimeError("Viewer not initialized")

        # Add points as a new geom in the model
        for point in points:
            mujoco.mjv_addGeom(
                self.model,
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.002, 0, 0],
                pos=point.tolist(),
                rgba=[0, 1, 0, 0.5],  # Semi-transparent green
            )
        mujoco.mj_forward(self.model, self.data)
        self.viewer.sync()
