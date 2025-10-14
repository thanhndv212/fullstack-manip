"""
Safety behaviors for monitoring limits and emergency conditions.

These behaviors provide BT leaf nodes for safety checks and
emergency procedures.
"""

from typing import Optional, List
import numpy as np

from fullstack_manip.execution.behaviors import BaseBehavior, Status
from fullstack_manip.execution.blackboard import Blackboard


class MonitorJointLimits(BaseBehavior):
    """
    Monitor robot joint positions against limits.

    Blackboard Inputs:
        - current_joint_positions: np.ndarray
        - safety_margin: float (default 0.05 rad)

    Blackboard Outputs:
        - joint_limits_ok: bool
        - joints_near_limit: List[int] (indices of joints near limits)
        - limit_violation: bool
    """

    def __init__(
        self,
        name: str,
        robot_model,
        blackboard: Blackboard,
        safety_margin: float = 0.05,
    ) -> None:
        """
        Initialize joint limit monitor.

        Args:
            name: Behavior name
            robot_model: Robot model with joint limits
            blackboard: Shared state storage
            safety_margin: Safety margin from limits (radians)
        """
        super().__init__(name, blackboard)
        self.robot = robot_model
        self.safety_margin = safety_margin

    def update(self) -> Status:
        """Check joint limits."""
        joint_positions = self.blackboard.get("current_joint_positions")

        if joint_positions is None:
            self.feedback_message = "No joint positions available"
            return Status.FAILURE

        try:
            # Get joint limits from robot model
            q_min = self.robot.get_joint_limits_lower()
            q_max = self.robot.get_joint_limits_upper()

            margin = self.blackboard.get("safety_margin", self.safety_margin)

            # Check for violations
            violations = (joint_positions < q_min) | (joint_positions > q_max)

            # Check for approaching limits
            near_lower = joint_positions < (q_min + margin)
            near_upper = joint_positions > (q_max - margin)
            near_limits = near_lower | near_upper

            # Store results
            has_violation = np.any(violations)
            joints_near = np.where(near_limits)[0].tolist()

            self.blackboard.set("joint_limits_ok", not has_violation)
            self.blackboard.set("joints_near_limit", joints_near)
            self.blackboard.set("limit_violation", has_violation)

            if has_violation:
                violated_joints = np.where(violations)[0]
                self.feedback_message = (
                    f"Joint limit violated: joints {violated_joints.tolist()}"
                )
                return Status.FAILURE
            elif len(joints_near) > 0:
                self.feedback_message = (
                    f"Joints {joints_near} approaching limits"
                )
                return Status.SUCCESS  # Warning but not failure
            else:
                self.feedback_message = "All joints within safe limits"
                return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"Joint limit check error: {str(e)}"
            return Status.FAILURE


class MonitorWorkspaceBounds(BaseBehavior):
    """
    Monitor end-effector position within workspace bounds.

    Blackboard Inputs:
        - current_ee_position: np.ndarray [x, y, z]
        - workspace_bounds: Dict with 'min' and 'max' keys

    Blackboard Outputs:
        - workspace_ok: bool
        - distance_to_boundary: float
        - workspace_violation: bool
    """

    def __init__(
        self,
        name: str,
        blackboard: Blackboard,
        workspace_min: Optional[np.ndarray] = None,
        workspace_max: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize workspace monitor.

        Args:
            name: Behavior name
            blackboard: Shared state storage
            workspace_min: Minimum workspace bounds [x, y, z]
            workspace_max: Maximum workspace bounds [x, y, z]
        """
        super().__init__(name, blackboard)
        self.workspace_min = workspace_min
        self.workspace_max = workspace_max

    def update(self) -> Status:
        """Check workspace bounds."""
        ee_position = self.blackboard.get("current_ee_position")

        if ee_position is None:
            self.feedback_message = "No end-effector position available"
            return Status.FAILURE

        # Get workspace bounds (from blackboard or defaults)
        bounds = self.blackboard.get("workspace_bounds", {})
        ws_min = bounds.get("min", self.workspace_min)
        ws_max = bounds.get("max", self.workspace_max)

        if ws_min is None or ws_max is None:
            self.feedback_message = "Workspace bounds not defined"
            return Status.FAILURE

        try:
            # Check bounds
            ws_min = np.array(ws_min)
            ws_max = np.array(ws_max)

            violations = (ee_position < ws_min) | (ee_position > ws_max)
            has_violation = np.any(violations)

            # Calculate distance to nearest boundary
            dist_to_min = ee_position - ws_min
            dist_to_max = ws_max - ee_position
            min_distance = np.min(np.concatenate([dist_to_min, dist_to_max]))

            # Store results
            self.blackboard.set("workspace_ok", not has_violation)
            self.blackboard.set("distance_to_boundary", min_distance)
            self.blackboard.set("workspace_violation", has_violation)

            if has_violation:
                self.feedback_message = f"Workspace violation at {ee_position}"
                return Status.FAILURE
            else:
                self.feedback_message = (
                    f"Within workspace (margin: {min_distance:.3f}m)"
                )
                return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"Workspace check error: {str(e)}"
            return Status.FAILURE


class CheckForceLimit(BaseBehavior):
    """
    Monitor force/torque readings against safety limits.

    Blackboard Inputs:
        - current_force: np.ndarray or float
        - force_limit: float (maximum allowed force)
        - torque_limit: float (optional, for full F/T sensor)

    Blackboard Outputs:
        - force_ok: bool
        - force_magnitude: float
        - force_exceeded: bool
    """

    def __init__(
        self,
        name: str,
        force_sensor,
        blackboard: Blackboard,
        force_limit: float = 100.0,
        torque_limit: Optional[float] = None,
    ) -> None:
        """
        Initialize force limit monitor.

        Args:
            name: Behavior name
            force_sensor: Force/torque sensor instance
            blackboard: Shared state storage
            force_limit: Maximum force (N)
            torque_limit: Maximum torque (Nm, optional)
        """
        super().__init__(name, blackboard)
        self.force_sensor = force_sensor
        self.force_limit = force_limit
        self.torque_limit = torque_limit

    def update(self) -> Status:
        """Check force limits."""
        try:
            # Get current force/torque
            force = self.force_sensor.get_force()

            if force is None:
                self.feedback_message = "Force sensor not available"
                return Status.FAILURE

            # Calculate magnitude
            if isinstance(force, np.ndarray):
                force_mag = np.linalg.norm(force[:3])  # Force component
            else:
                force_mag = abs(force)

            # Get limits (from blackboard or defaults)
            f_limit = self.blackboard.get("force_limit", self.force_limit)

            # Check force
            force_exceeded = force_mag > f_limit

            # Store results
            self.blackboard.set("force_ok", not force_exceeded)
            self.blackboard.set("force_magnitude", force_mag)
            self.blackboard.set("force_exceeded", force_exceeded)

            # Check torque if available
            if (
                self.torque_limit is not None
                and isinstance(force, np.ndarray)
                and len(force) >= 6
            ):
                torque_mag = np.linalg.norm(force[3:6])
                t_limit = self.blackboard.get(
                    "torque_limit", self.torque_limit
                )
                torque_exceeded = torque_mag > t_limit
                self.blackboard.set("torque_magnitude", torque_mag)
                self.blackboard.set("torque_exceeded", torque_exceeded)

                if torque_exceeded:
                    self.feedback_message = (
                        f"Torque limit exceeded: {torque_mag:.2f}Nm "
                        f"> {t_limit:.2f}Nm"
                    )
                    return Status.FAILURE

            if force_exceeded:
                self.feedback_message = (
                    f"Force limit exceeded: {force_mag:.2f}N "
                    f"> {f_limit:.2f}N"
                )
                return Status.FAILURE
            else:
                self.feedback_message = (
                    f"Force within limits ({force_mag:.2f}N)"
                )
                return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"Force check error: {str(e)}"
            return Status.FAILURE


class EmergencyStop(BaseBehavior):
    """
    Trigger emergency stop condition.

    This behavior immediately stops all robot motion
    and sets emergency flags.

    Blackboard Inputs:
        - emergency_reason: str (optional)

    Blackboard Outputs:
        - emergency_stop_active: bool
        - emergency_timestamp: float
        - emergency_reason: str
    """

    def __init__(
        self, name: str, robot_controller, blackboard: Blackboard
    ) -> None:
        """
        Initialize emergency stop behavior.

        Args:
            name: Behavior name
            robot_controller: Robot controller with stop capability
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.controller = robot_controller

    def update(self) -> Status:
        """Execute emergency stop."""
        reason = self.blackboard.get("emergency_reason", "Unspecified")

        try:
            # Stop robot immediately
            self.controller.emergency_stop()

            # Set emergency flags
            import time

            self.blackboard.set("emergency_stop_active", True)
            self.blackboard.set("emergency_timestamp", time.time())
            self.blackboard.set("emergency_reason", reason)

            self.feedback_message = f"EMERGENCY STOP: {reason}"
            return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"Emergency stop error: {str(e)}"
            return Status.FAILURE


class CheckCollision(BaseBehavior):
    """
    Check for potential collisions using collision detection.

    Blackboard Inputs:
        - current_joint_positions: np.ndarray
        - planned_trajectory: List[np.ndarray] (optional)

    Blackboard Outputs:
        - collision_detected: bool
        - collision_objects: List[str] (colliding objects)
        - collision_free: bool
    """

    def __init__(
        self, name: str, collision_checker, blackboard: Blackboard
    ) -> None:
        """
        Initialize collision check behavior.

        Args:
            name: Behavior name
            collision_checker: Collision detection instance
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.collision_checker = collision_checker

    def update(self) -> Status:
        """Check for collisions."""
        joint_positions = self.blackboard.get("current_joint_positions")

        if joint_positions is None:
            self.feedback_message = "No joint positions for collision check"
            return Status.FAILURE

        try:
            # Check current configuration
            is_collision, colliding_objects = (
                self.collision_checker.check_collision(joint_positions)
            )

            # Store results
            self.blackboard.set("collision_detected", is_collision)
            self.blackboard.set("collision_objects", colliding_objects)
            self.blackboard.set("collision_free", not is_collision)

            if is_collision:
                self.feedback_message = (
                    f"Collision detected with: {colliding_objects}"
                )
                return Status.FAILURE
            else:
                # Check trajectory if available
                trajectory = self.blackboard.get("planned_trajectory")
                if trajectory is not None:
                    for waypoint in trajectory:
                        is_collision, objects = (
                            self.collision_checker.check_collision(waypoint)
                        )
                        if is_collision:
                            self.feedback_message = (
                                f"Trajectory collision with: {objects}"
                            )
                            self.blackboard.set("collision_detected", True)
                            self.blackboard.set("collision_free", False)
                            return Status.FAILURE

                self.feedback_message = "No collisions detected"
                return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"Collision check error: {str(e)}"
            return Status.FAILURE


class ValidateTrajectory(BaseBehavior):
    """
    Validate a planned trajectory for safety.

    Checks joint limits, velocity limits, and collision-free path.

    Blackboard Inputs:
        - planned_trajectory: List[np.ndarray]
        - max_velocity: float (optional)
        - max_acceleration: float (optional)

    Blackboard Outputs:
        - trajectory_valid: bool
        - validation_errors: List[str]
    """

    def __init__(
        self,
        name: str,
        robot_model,
        blackboard: Blackboard,
        collision_checker=None,
    ) -> None:
        """
        Initialize trajectory validation behavior.

        Args:
            name: Behavior name
            robot_model: Robot model with limits
            blackboard: Shared state storage
            collision_checker: Optional collision checker
        """
        super().__init__(name, blackboard)
        self.robot = robot_model
        self.collision_checker = collision_checker

    def update(self) -> Status:
        """Validate trajectory."""
        trajectory = self.blackboard.get("planned_trajectory")

        if trajectory is None or len(trajectory) == 0:
            self.feedback_message = "No trajectory to validate"
            return Status.FAILURE

        try:
            errors = []

            # Check joint limits
            q_min = self.robot.get_joint_limits_lower()
            q_max = self.robot.get_joint_limits_upper()

            for i, waypoint in enumerate(trajectory):
                if np.any(waypoint < q_min) or np.any(waypoint > q_max):
                    errors.append(f"Joint limits at waypoint {i}")

            # Check collisions if checker available
            if self.collision_checker is not None:
                for i, waypoint in enumerate(trajectory):
                    is_collision, _ = self.collision_checker.check_collision(
                        waypoint
                    )
                    if is_collision:
                        errors.append(f"Collision at waypoint {i}")

            # Store results
            is_valid = len(errors) == 0
            self.blackboard.set("trajectory_valid", is_valid)
            self.blackboard.set("validation_errors", errors)

            if is_valid:
                self.feedback_message = "Trajectory validated"
                return Status.SUCCESS
            else:
                self.feedback_message = f"Validation failed: {errors[0]}"
                return Status.FAILURE

        except Exception as e:
            self.feedback_message = f"Validation error: {str(e)}"
            return Status.FAILURE
