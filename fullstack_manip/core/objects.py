"""Object management for manipulation tasks.

Tracks manipulatable objects in the scene, their properties,
poses, and affordances.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

import mujoco
import numpy as np


class ObjectType(Enum):
    """Types of manipulatable objects."""

    BOX = "box"
    SPHERE = "sphere"
    CYLINDER = "cylinder"
    MESH = "mesh"
    UNKNOWN = "unknown"


@dataclass
class ObjectProperties:
    """Physical and semantic properties of an object."""

    mass: float = 0.0
    friction: float = 0.5
    size: List[float] = field(default_factory=list)  # [x, y, z] dimensions
    color: Optional[List[float]] = None  # RGBA
    graspable: bool = True
    fragile: bool = False
    movable: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)


class ManipulationObject:
    """
    Represents a manipulatable object in the scene.

    Tracks object state, properties, and provides methods for
    pose estimation and affordance reasoning.
    """

    def __init__(
        self,
        name: str,
        object_type: ObjectType = ObjectType.UNKNOWN,
        model: Optional[mujoco.MjModel] = None,
        data: Optional[mujoco.MjData] = None,
        body_name: Optional[str] = None,
        properties: Optional[ObjectProperties] = None,
    ):
        """
        Initialize manipulation object.

        Args:
            name: Object identifier
            object_type: Type of object
            model: MuJoCo model (if simulated)
            data: MuJoCo data (if simulated)
            body_name: MuJoCo body name (if different from name)
            properties: Object properties
        """
        self.name = name
        self.object_type = object_type
        self.model = model
        self.data = data
        self.body_name = body_name or name
        self.properties = properties or ObjectProperties()

        # State
        self._position: Optional[np.ndarray] = None
        self._orientation: Optional[np.ndarray] = None  # quaternion
        self._velocity: Optional[np.ndarray] = None
        self._angular_velocity: Optional[np.ndarray] = None
        self._in_contact: bool = False
        self._grasped: bool = False
        self._grasped_by: Optional[str] = None

    def get_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Get object position and orientation.

        Returns:
            (position, orientation) where orientation is quaternion [w,x,y,z]
        """
        if self.model is not None and self.data is not None:
            # Get from simulation
            mujoco.mj_forward(self.model, self.data)
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, self.body_name
            )
            self._position = self.data.xpos[body_id].copy()
            self._orientation = self.data.xquat[body_id].copy()
        elif self._position is not None and self._orientation is not None:
            # Use cached values
            pass
        else:
            raise RuntimeError(f"Object {self.name}: No pose data available")

        return self._position, self._orientation

    def set_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """
        Set object pose.

        Args:
            position: [x, y, z]
            orientation: Quaternion [w, x, y, z]
        """
        self._position = np.array(position, dtype=float)
        self._orientation = np.array(orientation, dtype=float)

        if self.model is not None and self.data is not None:
            # Update simulation
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, self.body_name
            )
            # Find joint associated with body (freejoint assumed)
            joint_id = self.model.body_jntadr[body_id]
            if joint_id >= 0:
                qpos_adr = self.model.jnt_qposadr[joint_id]
                self.data.qpos[qpos_adr : qpos_adr + 3] = position
                self.data.qpos[qpos_adr + 3 : qpos_adr + 7] = orientation
                mujoco.mj_forward(self.model, self.data)

    def get_velocity(self) -> tuple[np.ndarray, np.ndarray]:
        """Get linear and angular velocity."""
        if self.model is not None and self.data is not None:
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, self.body_name
            )
            joint_id = self.model.body_jntadr[body_id]
            if joint_id >= 0:
                qvel_adr = self.model.jnt_dofadr[joint_id]
                self._velocity = self.data.qvel[qvel_adr : qvel_adr + 3].copy()
                self._angular_velocity = self.data.qvel[
                    qvel_adr + 3 : qvel_adr + 6
                ].copy()

        return self._velocity, self._angular_velocity

    def mark_grasped(self, gripper_name: str) -> None:
        """Mark object as grasped by a gripper."""
        self._grasped = True
        self._grasped_by = gripper_name

    def mark_released(self) -> None:
        """Mark object as released."""
        self._grasped = False
        self._grasped_by = None

    def is_grasped(self) -> bool:
        """Check if object is currently grasped."""
        return self._grasped

    def set_contact_state(self, in_contact: bool) -> None:
        """Set contact state."""
        self._in_contact = in_contact

    def is_in_contact(self) -> bool:
        """Check if object is in contact with something."""
        return self._in_contact

    def get_bounding_box(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Get axis-aligned bounding box.

        Returns:
            (min_corner, max_corner) in world coordinates
        """
        pos, _ = self.get_pose()
        half_size = np.array(self.properties.size) / 2.0
        return pos - half_size, pos + half_size

    def get_grasp_points(self) -> List[np.ndarray]:
        """
        Get suggested grasp points for this object.

        Returns:
            List of grasp positions in world coordinates
        """
        if not self.properties.graspable:
            return []

        pos, _ = self.get_pose()

        # Simple heuristic: grasp from sides
        if self.object_type == ObjectType.BOX:
            size = np.array(self.properties.size)
            grasp_points = [
                pos + [size[0] / 2, 0, 0],  # Right side
                pos - [size[0] / 2, 0, 0],  # Left side
                pos + [0, size[1] / 2, 0],  # Front
                pos - [0, size[1] / 2, 0],  # Back
            ]
            return grasp_points
        else:
            # Default: grasp from current position
            return [pos]

    def to_dict(self) -> Dict[str, Any]:
        """Convert object to dictionary representation."""
        pos, quat = self.get_pose()
        return {
            "name": self.name,
            "type": self.object_type.value,
            "position": pos.tolist(),
            "orientation": quat.tolist(),
            "grasped": self._grasped,
            "grasped_by": self._grasped_by,
            "in_contact": self._in_contact,
            "properties": {
                "mass": self.properties.mass,
                "friction": self.properties.friction,
                "size": self.properties.size,
                "graspable": self.properties.graspable,
                "fragile": self.properties.fragile,
                "movable": self.properties.movable,
            },
        }


class ObjectManager:
    """
    Manages multiple manipulatable objects in the scene.

    Provides centralized object tracking, querying, and state management.
    """

    def __init__(
        self,
        model: Optional[mujoco.MjModel] = None,
        data: Optional[mujoco.MjData] = None,
    ):
        """
        Initialize object manager.

        Args:
            model: MuJoCo model (optional)
            data: MuJoCo data (optional)
        """
        self.model = model
        self.data = data
        self._objects: Dict[str, ManipulationObject] = {}

    def register_object(
        self,
        name: str,
        object_type: ObjectType = ObjectType.UNKNOWN,
        body_name: Optional[str] = None,
        properties: Optional[ObjectProperties] = None,
    ) -> ManipulationObject:
        """
        Register a new object.

        Args:
            name: Object identifier
            object_type: Type of object
            body_name: MuJoCo body name (if different from name)
            properties: Object properties

        Returns:
            Created object
        """
        obj = ManipulationObject(
            name=name,
            object_type=object_type,
            model=self.model,
            data=self.data,
            body_name=body_name,
            properties=properties,
        )
        self._objects[name] = obj
        return obj

    def unregister_object(self, name: str) -> None:
        """Remove object from tracking."""
        if name in self._objects:
            del self._objects[name]

    def get_object(self, name: str) -> Optional[ManipulationObject]:
        """Get object by name."""
        return self._objects.get(name)

    def get_all_objects(self) -> Dict[str, ManipulationObject]:
        """Get all registered objects."""
        return self._objects.copy()

    def get_objects_by_type(
        self, object_type: ObjectType
    ) -> List[ManipulationObject]:
        """Get all objects of a specific type."""
        return [
            obj
            for obj in self._objects.values()
            if obj.object_type == object_type
        ]

    def get_graspable_objects(self) -> List[ManipulationObject]:
        """Get all graspable objects."""
        return [
            obj for obj in self._objects.values() if obj.properties.graspable
        ]

    def get_grasped_objects(self) -> List[ManipulationObject]:
        """Get all currently grasped objects."""
        return [obj for obj in self._objects.values() if obj.is_grasped()]

    def find_nearest_object(
        self, position: np.ndarray, max_distance: float = np.inf
    ) -> Optional[ManipulationObject]:
        """
        Find nearest object to a position.

        Args:
            position: Query position
            max_distance: Maximum search distance

        Returns:
            Nearest object or None if none within max_distance
        """
        nearest = None
        min_dist = max_distance

        for obj in self._objects.values():
            obj_pos, _ = obj.get_pose()
            dist = np.linalg.norm(obj_pos - position)
            if dist < min_dist:
                min_dist = dist
                nearest = obj

        return nearest

    def update_all_states(self, state_manager) -> None:
        """
        Update all object states in state manager.

        Args:
            state_manager: StateManager instance to update
        """
        for obj in self._objects.values():
            pos, quat = obj.get_pose()
            vel, ang_vel = obj.get_velocity()
            state_manager.update_object_state(
                name=obj.name,
                position=pos,
                orientation=quat,
                velocity=vel,
                angular_velocity=ang_vel,
                in_contact=obj.is_in_contact(),
                grasped=obj.is_grasped(),
            )

    def get_scene_summary(self) -> Dict[str, Any]:
        """Get summary of all objects in scene."""
        return {
            "total_objects": len(self._objects),
            "objects": {
                name: obj.to_dict() for name, obj in self._objects.items()
            },
            "graspable_count": len(self.get_graspable_objects()),
            "grasped_count": len(self.get_grasped_objects()),
        }


__all__ = [
    "ManipulationObject",
    "ObjectManager",
    "ObjectType",
    "ObjectProperties",
]
