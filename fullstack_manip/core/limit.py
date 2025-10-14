"""Robot limit management and configuration."""

from typing import List, Tuple, Optional

import mink
import mujoco
import numpy as np


class LimitManager:
    """Manages robot limits including velocity, configuration, and collision avoidance."""

    def __init__(
        self,
        model: mujoco.MjModel,
        gripper_bodies: Optional[List[str]] = None,
        obstacles: Optional[List[str]] = None,
    ):
        """
        Initialize limit manager.

        Args:
            model: MuJoCo model
            gripper_bodies: List of gripper body names for collision checking
            obstacles: List of obstacle geometry names for collision checking
        """
        self.model = model
        self.gripper_bodies = gripper_bodies or []
        self.obstacles = obstacles or []
        self.limits = []

    def set_limits(
        self,
        collision_pairs: Optional[List[Tuple[str, str]]] = None,
        max_velocities: Optional[dict] = None,
    ) -> List:
        """
        Set up robot limits including configuration, collision avoidance, and velocity.

        Args:
            collision_pairs: List of tuples of geometry pairs to check for collision
            max_velocities: Dictionary of joint names to maximum velocities

        Returns:
            List of limit objects
        """
        if collision_pairs is None:
            collision_pairs = []

        # Build collision pairs from gripper bodies and obstacles
        for body in self.gripper_bodies:
            body_geom_ids = mink.get_body_geom_ids(
                self.model,
                self.model.body(body).id,
            )
            for geom_id in body_geom_ids:
                for obstacle in self.obstacles:
                    collision_pairs.append(([geom_id], [obstacle]))

        # Configuration limits
        limits = [
            mink.ConfigurationLimit(model=self.model),
            mink.CollisionAvoidanceLimit(
                model=self.model,
                geom_pairs=collision_pairs,
            ),
        ]

        # Velocity limits
        if max_velocities is None:
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
        return self.limits

    def get_limits(self) -> List:
        """
        Get the configured limits.

        Returns:
            List of limit objects
        """
        return self.limits


__all__ = ["LimitManager"]
