"""Force control controller for contact-rich manipulation tasks."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import numpy as np

from .base_controller import BaseController

if TYPE_CHECKING:  # pragma: no cover - circular import guard
    from ...core.robot import Robot


class ForceControlController(BaseController):
    """Execute contact-rich tasks with force/torque feedback.

    This controller uses MuJoCo's contact dynamics to perform tasks
    requiring force control such as assembly, polishing, or compliant
    motion. Monitors contact forces and adjusts motion accordingly.

    Attributes:
        target_force: Desired contact force magnitude (N).
        force_tolerance: Acceptable deviation from target force (N).
        contact_bodies: List of body names to monitor for contact.
    """

    def __init__(
        self,
        robot: "Robot",
        target_force: float = 5.0,
        force_tolerance: float = 1.0,
        contact_bodies: Optional[list[str]] = None,
    ) -> None:
        """Initialize the force control controller.

        Args:
            robot: Robot instance to control.
            target_force: Desired contact force in Newtons.
            force_tolerance: Acceptable force deviation in Newtons.
            contact_bodies: Body names to monitor for contact forces.
        """
        super().__init__(robot)

        if target_force < 0:
            raise ValueError("Target force must be non-negative")
        if force_tolerance <= 0:
            raise ValueError("Force tolerance must be positive")

        self.target_force = target_force
        self.force_tolerance = force_tolerance
        self.contact_bodies = (
            contact_bodies if contact_bodies is not None else []
        )
        self._contact_threshold = 0.1  # Minimum force to consider contact

    def execute(
        self,
        target_pos: np.ndarray,
        target_orient: Optional[np.ndarray] = None,
        force_direction: Optional[np.ndarray] = None,
    ) -> None:
        """Execute force-controlled motion to target pose.

        Args:
            target_pos: Target position [x, y, z].
            target_orient: Target orientation as quaternion.
            force_direction: Direction to apply force (unit vector).
                If None, applies force normal to contact surface.
        """
        if not isinstance(target_pos, np.ndarray) or target_pos.shape != (3,):
            raise ValueError("Target position must be 3D numpy array")

        if force_direction is not None:
            if not isinstance(force_direction, np.ndarray):
                raise ValueError("Force direction must be 3D numpy array")
            if force_direction.shape != (3,):
                raise ValueError("Force direction must be 3D numpy array")
            force_direction = force_direction / np.linalg.norm(force_direction)

        self._move_with_force_control(
            target_pos, target_orient, force_direction
        )

    def apply_constant_force(
        self,
        force_vector: np.ndarray,
        duration: float = 5.0,
    ) -> None:
        """Apply constant force in specified direction.

        Args:
            force_vector: Force to apply [Fx, Fy, Fz] in Newtons.
            duration: Duration to maintain force (seconds).

        Raises:
            ValueError: If force_vector is invalid.
        """
        if not isinstance(force_vector, np.ndarray):
            raise ValueError("Force vector must be numpy array")
        if force_vector.shape != (3,):
            raise ValueError("Force vector must be 3D")

        steps = int(duration / self.dt)

        for _ in range(steps):
            # Convert Cartesian force to joint torques (simplified)
            # In practice, use Jacobian transpose: tau = J^T * F
            force_magnitude = np.linalg.norm(force_vector)

            # Apply small motion in force direction
            ee_pos, _ = self.robot.get_body_pose(self.robot.end_effector_name)
            direction = force_vector / force_magnitude
            target = ee_pos + direction * 0.001  # Small step

            self.move_to_pose(target, duration=self.dt)

    def maintain_contact(
        self,
        contact_body: str,
        duration: float = 5.0,
    ) -> bool:
        """Maintain contact with specified body at target force.

        Args:
            contact_body: Name of body to maintain contact with.
            duration: Duration to maintain contact (seconds).

        Returns:
            True if contact was successfully maintained, False otherwise.
        """
        steps = int(duration / self.dt)
        contact_maintained = True

        for _ in range(steps):
            current_force = self._measure_contact_force(contact_body)

            if current_force < self._contact_threshold:
                print(f"Lost contact with {contact_body}")
                contact_maintained = False
                break

            force_error = self.target_force - current_force

            if abs(force_error) > self.force_tolerance:
                # Adjust position to maintain target force
                self._adjust_for_force_error(force_error)

        return contact_maintained

    def compliant_motion(
        self,
        target_pos: np.ndarray,
        max_force: float,
        approach_speed: float = 0.01,
    ) -> bool:
        """Move towards target with compliance, stopping at max force.

        Args:
            target_pos: Target position [x, y, z].
            max_force: Maximum allowable contact force (N).
            approach_speed: Approach velocity (m/s).

        Returns:
            True if reached target, False if stopped by force limit.
        """
        if not isinstance(target_pos, np.ndarray) or target_pos.shape != (3,):
            raise ValueError("Target position must be 3D numpy array")
        if max_force <= 0:
            raise ValueError("Max force must be positive")

        ee_pos, _ = self.robot.get_body_pose(self.robot.end_effector_name)
        distance = np.linalg.norm(target_pos - ee_pos)

        while distance > self.reach_threshold[0]:
            contact_force = self._measure_total_contact_force()

            if contact_force > max_force:
                print(
                    f"Stopped: contact force {contact_force:.2f}N "
                    f"exceeds limit {max_force:.2f}N"
                )
                return False

            # Take small step towards target
            direction = (target_pos - ee_pos) / distance
            step_size = min(approach_speed * self.dt, distance)
            next_pos = ee_pos + direction * step_size

            self.move_to_pose(next_pos, duration=self.dt)

            ee_pos, _ = self.robot.get_body_pose(self.robot.end_effector_name)
            distance = np.linalg.norm(target_pos - ee_pos)

        return True

    def _move_with_force_control(
        self,
        target_pos: np.ndarray,
        target_orient: Optional[np.ndarray],
        force_direction: Optional[np.ndarray],
    ) -> None:
        """Internal method for force-controlled motion."""
        # Simplified force control - would use proper hybrid
        # position/force control in practice
        self.move_to_pose(target_pos, target_orient)

    def _measure_contact_force(self, body_name: str) -> float:
        """Measure total contact force on specified body.

        Args:
            body_name: Name of body to measure forces on.

        Returns:
            Total contact force magnitude in Newtons.
        """
        if not hasattr(self.robot, "compute_contact_force"):
            return 0.0

        total_force = 0.0
        for contact in self.robot.data.contact[: self.robot.data.ncon]:
            # Check if contact involves the body
            # Simplified - would need proper geom-to-body mapping
            total_force += np.linalg.norm(contact.frame[:3])

        return total_force

    def _measure_total_contact_force(self) -> float:
        """Measure total contact force across all monitored bodies."""
        total = 0.0
        for body_name in self.contact_bodies:
            total += self._measure_contact_force(body_name)
        return total

    def _adjust_for_force_error(self, force_error: float) -> None:
        """Adjust robot position based on force error."""
        # Simplified compliance - move slightly in/out based on error
        ee_pos, ee_orient = self.robot.get_body_pose(
            self.robot.end_effector_name
        )

        # Move away if force too high, towards if too low
        adjustment = 0.001 * np.sign(force_error)
        adjusted_pos = ee_pos + np.array([0, 0, adjustment])
        self.move_to_pose(adjusted_pos, ee_orient, duration=self.dt)

    def reset(self) -> None:
        """Reset force controller state."""
        pass


__all__ = ["ForceControlController"]
