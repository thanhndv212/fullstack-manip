from __future__ import annotations

import time
from typing import List, Optional, Tuple

import mujoco
import numpy as np

from ..low_level.pid_controller import PIDController
from ...planning.motion_planner import MotionPlanner
from ...simulation.viewer import MuJoCoViewer


class Robot:

    def __init__(
        self,
        model: mujoco.MjModel,
        data: Optional[mujoco.MjData] = None,
        end_effector_name: str = None,
        end_effector_type: str = None,
        gripper_bodies: List[str] = None,
        obstacles: List[str] = None,
    ):
        try:
            self.model = model
            self.data = mujoco.MjData(self.model)
            self.end_effector_name = end_effector_name
            self.end_effector_type = end_effector_type
            self.gripper_bodies = gripper_bodies
            self.motion_planner = MotionPlanner(
                self.model,
                self.data,
                self.end_effector_name,
                self.end_effector_type,
                self.gripper_bodies,
                obstacles,
            )
            self.pid_controller = PIDController(
                kp=10.0,
                ki=0.1,
                kd=1.0,
                dt=0.01,
                integral_limit=1.0,
                output_limit=10.0,
            )
            self.viewer = MuJoCoViewer(self.model, self.data)
            self.viewer.launch_passive()
            self.viewer.sync()
            self.scene_nq = self.model.nq
            self.scene_nu = self.model.nu
            self.robot_nq = self.model.nu
            self.dt = 0.01
            self.data.qpos[:] = self.model.key("home").qpos
            mujoco.mj_forward(self.model, self.data)
        except Exception as err:  # pragma: no cover - defensive
            raise RuntimeError(f"Failed to initialize robot: {err}")

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

    def move_to_position(
        self,
        target_pos: np.ndarray,
        target_orient: np.ndarray = None,
        duration: float = 4.0,
    ) -> None:
        """Move robot end-effector to target position using Mink IK."""
        if not isinstance(target_pos, np.ndarray) or target_pos.shape != (3,):
            raise ValueError("Target position must be a 3D numpy array")

        current_ee_pos, current_ee_quat = self.get_body_pose(self.end_effector_name)

        trajectory, _ = self.motion_planner.plan_trajectory(
            current_ee_pos,
            target_pos,
            start_orient=current_ee_quat,
            end_orient=target_orient,
            duration=duration,
        )

        if trajectory is None:
            raise RuntimeError("Failed to plan trajectory")

        current_joint_positions = self.get_robot_joint_positions()
        count = 0
        for target in trajectory:
            control_signal = self.pid_controller.compute(
                target,
                current_joint_positions,
            )
            current_joint_positions += control_signal * self.dt
            self.viewer.step(current_joint_positions)
            time.sleep(self.dt)
            count += 1
            if self.detect_contact(self.end_effector_name, self.object_geom):
                print("Contact detected with cube, stopping movement.")
                break
        print(f"Executed {count} control steps to reach target position.")

    def get_body_geom_ids(
        self,
        model: mujoco.MjModel,
        body_id: int,
    ) -> List[int]:
        """Get immediate geoms belonging to a given body."""
        geom_start = model.body_geomadr[body_id]
        geom_end = geom_start + model.body_geomnum[body_id]
        return list(range(geom_start, geom_end))

    def compute_contact_force(
        self,
        geom1_name: str | List[int],
        geom2_name: str | List[int],
    ) -> float:
        """Compute the total contact force magnitude between two geometries."""
        geom1_ids = (
            [
                mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_GEOM,
                    geom1_name,
                )
            ]
            if isinstance(geom1_name, str)
            else geom1_name
        )
        geom2_ids = (
            [
                mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_GEOM,
                    geom2_name,
                )
            ]
            if isinstance(geom2_name, str)
            else geom2_name
        )
        print(
            "Computing contact force between geoms "
            f"{geom1_ids} and {geom2_ids}"
        )
        total_force = 0.0
        for contact in self.data.contact[: self.data.ncon]:
            if (contact.geom1 in geom1_ids and contact.geom2 in geom2_ids) or (
                contact.geom1 in geom2_ids and contact.geom2 in geom1_ids
            ):
                total_force += np.linalg.norm(contact.frame[:3])
        return total_force

    def check_grasp_success(self) -> bool:
        return self.check_grasp_contact() and self.check_in_hand()

    def check_grasp_contact(self) -> bool:
        """Check if the grasp is successful based on contact forces."""
        total_forces = {}
        for gripper_body in self.gripper_bodies:
            gripper_geoms = self.get_body_geom_ids(
                self.model,
                self.model.body(gripper_body).id,
            )
            total_forces[gripper_body] = self.compute_contact_force(
                gripper_geoms,
                self.object_geom,
            )
            print(
                "Grasp contact forces of body "
                f"{gripper_body}: {total_forces[gripper_body]}"
            )
        return all(
            force > self.grasp_force_threshold
            for force in total_forces.values()
        )

    def check_in_hand(self) -> bool:
        """Check if the object is still in hand based on relative position."""
        object_pos, _ = self.get_body_pose(self.object_geom)
        gripper_pos, _ = self.get_body_pose(self.end_effector_name)
        distance = np.linalg.norm(object_pos - gripper_pos)
        print(f"Debug info: gripper_pos = {gripper_pos}, object_pos = {object_pos}")
        print(f"Distance between object and gripper: {distance}")
        return distance < 0.015

    def _close_gripper(self) -> None:
        """Close robot gripper and check for successful grasp."""
        T = 0.5
        grasp_count = 0
        for joint_name in self.gripper_joint_names:
            joint_id = self.model.joint(joint_name).id
            while abs(self.data.qpos[joint_id] - self.close_position) > 1e-3:
                current_gripper_position = self.data.qpos[joint_id]
                vel = (
                    self.close_position - current_gripper_position
                ) / T
                self.data.qpos[joint_id] = (
                    vel * self.dt + current_gripper_position
                )
                current_joint_positions = self.get_robot_joint_positions()
                self.viewer.step(current_joint_positions)
                time.sleep(self.dt)

                if self.check_grasp_success():
                    grasp_count += 1
                else:
                    grasp_count = 0

                print(f"Grasp success count: {grasp_count}")
                if grasp_count == 100:
                    self.GRASP_SUCCESS = True
                    print("Grasp successful")
                    break

        if not self.GRASP_SUCCESS:
            print("Grasp failed")

    def _open_gripper(self) -> None:
        """Open robot gripper and check for successful release."""
        T = 0.5
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

        total_force = 0.0
        for gripper_body in self.gripper_bodies:
            gripper_geoms = self.get_body_geom_ids(
                self.model,
                self.model.body(gripper_body).id,
            )
            total_force += self.compute_contact_force(
                gripper_geoms,
                self.object_geom,
            )
        if total_force < self.release_force_threshold:
            print("Release successful")
        else:
            print("Release failed")

    def detect_contact(self, geom1_name: str, geom2_name: str) -> bool:
        """Detect if two geometries are in contact."""
        geom1_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            geom1_name,
        )
        geom2_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            geom2_name,
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
        """Estimate the robot's reachable workspace via inverse kinematics."""
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
            ik_solution = self.motion_planner.solve_ik_for_pose(self.end_effector_name, self.end_effector_type, target)
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
            max_geom = getattr(scene, "maxgeom", None)

            for i, point in enumerate(points):
                # if max_geom is not None and scene.ngeom >= max_geom:
                #     break

                mujoco.mjv_initGeom(
                    scene.geoms[i],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=size,
                    pos=np.asarray(point, dtype=float),
                    rgba=rgba,
                )

            self.viewer.sync()


__all__ = ["Robot"]
