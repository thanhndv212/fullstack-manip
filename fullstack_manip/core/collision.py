"""Collision detection and contact force computation."""

from typing import List, Union

import mujoco
import numpy as np


class CollisionChecker:
    """Handles collision detection and contact force computation for robots."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        """
        Initialize collision checker.

        Args:
            model: MuJoCo model
            data: MuJoCo data
        """
        self.model = model
        self.data = data

    def get_body_geom_ids(self, body_id: int) -> List[int]:
        """
        Get immediate geoms belonging to a given body.

        Args:
            body_id: ID of the body

        Returns:
            List of geometry IDs belonging to the body
        """
        geom_start = self.model.body_geomadr[body_id]
        geom_end = geom_start + self.model.body_geomnum[body_id]
        return list(range(geom_start, geom_end))

    def detect_contact(self, geom1_name: str, geom2_name: str) -> bool:
        """
        Detect if two geometries are in contact.

        Args:
            geom1_name: Name of first geometry
            geom2_name: Name of second geometry

        Returns:
            True if geometries are in contact, False otherwise
        """
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

    def check_contact_pair(self, geom1_name: str, geom2_name: str) -> bool:
        """
        Check if two specific geometries are in contact (with penetration).

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

    def compute_contact_force(
        self,
        geom1_name: Union[str, List[int]],
        geom2_name: Union[str, List[int]],
    ) -> float:
        """
        Compute the total contact force magnitude between two geometries.

        Args:
            geom1_name: Name of first geometry or list of geometry IDs
            geom2_name: Name of second geometry or list of geometry IDs

        Returns:
            Total contact force magnitude
        """
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

    def check_collision(self, trajectory, obstacles):
        """
        Check trajectory for collisions using MuJoCo.

        Args:
            trajectory: Planned trajectory (joint positions)
            obstacles: List of obstacle geometry names

        Returns:
            True if collision-free, False otherwise
        """
        for joints in trajectory:
            self.data.qpos[:] = joints
            mujoco.mj_forward(self.model, self.data)
            # Check for collisions with obstacles
            for obstacle in obstacles:
                # Assuming "robot_geom" is a representative geom
                if self.check_contact_pair("robot_geom", obstacle):
                    return False
        return True


__all__ = ["CollisionChecker"]
