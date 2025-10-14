"""
Control behaviors that wrap existing high-level controllers.

These behaviors provide BT leaf nodes for various control strategies
by wrapping the existing controller implementations.
"""

from typing import Optional
import numpy as np

from fullstack_manip.execution.behaviors import BaseBehavior, Status
from fullstack_manip.execution.blackboard import Blackboard


class ImpedanceControlBehavior(BaseBehavior):
    """
    Execute impedance control to a target pose.

    Wraps the ImpedanceController from control.high_level.

    Blackboard Inputs:
        - target_pose: Dict with 'position' and 'orientation'
        - stiffness: float or np.ndarray (optional)
        - damping: float or np.ndarray (optional)

    Blackboard Outputs:
        - control_status: str
        - position_error: float
        - at_target: bool
    """

    def __init__(
        self,
        name: str,
        impedance_controller,
        blackboard: Blackboard,
        position_tolerance: float = 0.01,
    ) -> None:
        """
        Initialize impedance control behavior.

        Args:
            name: Behavior name
            impedance_controller: ImpedanceController instance
            blackboard: Shared state storage
            position_tolerance: Position error tolerance (m)
        """
        super().__init__(name, blackboard)
        self.controller = impedance_controller
        self.position_tolerance = position_tolerance

    def update(self) -> Status:
        """Execute impedance control."""
        target_pose = self.blackboard.get("target_pose")

        if target_pose is None:
            self.feedback_message = "No target pose on blackboard"
            return Status.FAILURE

        try:
            # Get optional stiffness and damping
            stiffness = self.blackboard.get("stiffness", None)
            damping = self.blackboard.get("damping", None)

            # Set control parameters if provided
            if stiffness is not None:
                self.controller.set_stiffness(stiffness)
            if damping is not None:
                self.controller.set_damping(damping)

            # Set target
            self.controller.set_target(
                target_pose.get("position"), target_pose.get("orientation")
            )

            # Check if at target
            error = self.controller.get_position_error()
            at_target = error < self.position_tolerance

            self.blackboard.set("position_error", error)
            self.blackboard.set("at_target", at_target)

            if at_target:
                self.blackboard.set("control_status", "at_target")
                self.feedback_message = f"At target (error: {error:.4f}m)"
                return Status.SUCCESS
            else:
                self.blackboard.set("control_status", "approaching")
                return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Impedance control error: {str(e)}"
            self.blackboard.set("control_status", "error")
            return Status.FAILURE


class AdmittanceControlBehavior(BaseBehavior):
    """
    Execute admittance control for compliant interaction.

    Wraps the AdmittanceController from control.high_level.

    Blackboard Inputs:
        - target_force: np.ndarray or float
        - virtual_mass: float (optional)
        - virtual_damping: float (optional)

    Blackboard Outputs:
        - control_status: str
        - force_error: float
        - at_target_force: bool
    """

    def __init__(
        self,
        name: str,
        admittance_controller,
        blackboard: Blackboard,
        force_tolerance: float = 1.0,
    ) -> None:
        """
        Initialize admittance control behavior.

        Args:
            name: Behavior name
            admittance_controller: AdmittanceController instance
            blackboard: Shared state storage
            force_tolerance: Force error tolerance (N)
        """
        super().__init__(name, blackboard)
        self.controller = admittance_controller
        self.force_tolerance = force_tolerance

    def update(self) -> Status:
        """Execute admittance control."""
        target_force = self.blackboard.get("target_force")

        if target_force is None:
            self.feedback_message = "No target force on blackboard"
            return Status.FAILURE

        try:
            # Get optional parameters
            virtual_mass = self.blackboard.get("virtual_mass", None)
            virtual_damping = self.blackboard.get("virtual_damping", None)

            # Set control parameters if provided
            if virtual_mass is not None:
                self.controller.set_virtual_mass(virtual_mass)
            if virtual_damping is not None:
                self.controller.set_virtual_damping(virtual_damping)

            # Set target force
            self.controller.set_target_force(target_force)

            # Check if at target force
            error = self.controller.get_force_error()
            at_target = error < self.force_tolerance

            self.blackboard.set("force_error", error)
            self.blackboard.set("at_target_force", at_target)

            if at_target:
                self.blackboard.set("control_status", "at_target_force")
                self.feedback_message = (
                    f"At target force (error: {error:.2f}N)"
                )
                return Status.SUCCESS
            else:
                self.blackboard.set("control_status", "adjusting")
                return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Admittance control error: {str(e)}"
            self.blackboard.set("control_status", "error")
            return Status.FAILURE


class TrajectoryFollowingBehavior(BaseBehavior):
    """
    Follow a planned trajectory.

    Wraps the TrajectoryFollowingController from control.high_level.

    Blackboard Inputs:
        - planned_trajectory: List[np.ndarray]
        - trajectory_dt: float (optional, timestep)

    Blackboard Outputs:
        - trajectory_progress: float (0.0 to 1.0)
        - trajectory_complete: bool
        - control_status: str
    """

    def __init__(
        self, name: str, trajectory_controller, blackboard: Blackboard
    ) -> None:
        """
        Initialize trajectory following behavior.

        Args:
            name: Behavior name
            trajectory_controller: TrajectoryFollowingController instance
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.controller = trajectory_controller
        self.started = False

    def setup(self) -> None:
        """Setup before execution."""
        self.started = False

    def update(self) -> Status:
        """Execute trajectory following."""
        trajectory = self.blackboard.get("planned_trajectory")

        if trajectory is None:
            self.feedback_message = "No trajectory on blackboard"
            return Status.FAILURE

        try:
            if not self.started:
                # Start trajectory execution
                dt = self.blackboard.get("trajectory_dt", 0.01)
                self.controller.start_trajectory(trajectory, dt)
                self.started = True
                self.blackboard.set("control_status", "executing")

            # Update controller
            self.controller.update()

            # Check progress
            progress = self.controller.get_progress()
            complete = self.controller.is_complete()

            self.blackboard.set("trajectory_progress", progress)
            self.blackboard.set("trajectory_complete", complete)

            if complete:
                self.blackboard.set("control_status", "complete")
                self.feedback_message = "Trajectory complete"
                return Status.SUCCESS
            else:
                return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Trajectory following error: {str(e)}"
            self.blackboard.set("control_status", "error")
            return Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        """Cleanup on termination."""
        if self.started:
            self.controller.stop()
            self.started = False


class ForceControlBehavior(BaseBehavior):
    """
    Execute force control along specified directions.

    Wraps the ForceControlController from control.high_level.

    Blackboard Inputs:
        - target_force: np.ndarray [fx, fy, fz]
        - force_directions: np.ndarray (unit vectors)
        - duration: float (optional, max duration)

    Blackboard Outputs:
        - current_force: np.ndarray
        - force_control_time: float
        - control_status: str
    """

    def __init__(
        self,
        name: str,
        force_controller,
        blackboard: Blackboard,
        max_duration: Optional[float] = None,
    ) -> None:
        """
        Initialize force control behavior.

        Args:
            name: Behavior name
            force_controller: ForceControlController instance
            blackboard: Shared state storage
            max_duration: Maximum control duration (seconds)
        """
        super().__init__(name, blackboard)
        self.controller = force_controller
        self.max_duration = max_duration
        self.start_time = None

    def setup(self) -> None:
        """Setup before execution."""
        import time

        self.start_time = time.time()

    def update(self) -> Status:
        """Execute force control."""
        target_force = self.blackboard.get("target_force")

        if target_force is None:
            self.feedback_message = "No target force on blackboard"
            return Status.FAILURE

        try:
            # Get optional force directions
            directions = self.blackboard.get("force_directions", None)

            # Set force target
            if directions is not None:
                self.controller.set_target_force(target_force, directions)
            else:
                self.controller.set_target_force(target_force)

            # Get current force
            current_force = self.controller.get_measured_force()
            self.blackboard.set("current_force", current_force)

            # Check duration
            import time

            elapsed = time.time() - self.start_time
            self.blackboard.set("force_control_time", elapsed)

            duration = self.blackboard.get("duration", self.max_duration)
            if duration is not None and elapsed >= duration:
                self.blackboard.set("control_status", "duration_complete")
                self.feedback_message = (
                    f"Force control duration complete ({elapsed:.2f}s)"
                )
                return Status.SUCCESS

            # Continue force control
            self.blackboard.set("control_status", "controlling")
            return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Force control error: {str(e)}"
            self.blackboard.set("control_status", "error")
            return Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        """Cleanup on termination."""
        if self.controller is not None:
            self.controller.stop()


class GravityCompensationBehavior(BaseBehavior):
    """
    Enable gravity compensation mode.

    Wraps the GravityCompensationController from control.high_level.

    Blackboard Inputs:
        - enable_gravity_comp: bool

    Blackboard Outputs:
        - gravity_comp_active: bool
        - control_status: str
    """

    def __init__(
        self, name: str, gravity_comp_controller, blackboard: Blackboard
    ) -> None:
        """
        Initialize gravity compensation behavior.

        Args:
            name: Behavior name
            gravity_comp_controller: GravityCompensationController instance
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.controller = gravity_comp_controller

    def update(self) -> Status:
        """Enable/disable gravity compensation."""
        enable = self.blackboard.get("enable_gravity_comp", True)

        try:
            if enable:
                self.controller.enable()
                self.blackboard.set("gravity_comp_active", True)
                self.blackboard.set("control_status", "active")
                self.feedback_message = "Gravity compensation enabled"
            else:
                self.controller.disable()
                self.blackboard.set("gravity_comp_active", False)
                self.blackboard.set("control_status", "disabled")
                self.feedback_message = "Gravity compensation disabled"

            return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"Gravity compensation error: {str(e)}"
            return Status.FAILURE


class PickPlaceControlBehavior(BaseBehavior):
    """
    Execute pick-and-place using existing PickPlaceController.

    This wraps the high-level PickPlaceController but allows
    behavior tree orchestration of the overall task.

    Blackboard Inputs:
        - pick_position: np.ndarray [x, y, z]
        - place_position: np.ndarray [x, y, z]

    Blackboard Outputs:
        - pick_place_status: str
        - pick_complete: bool
        - place_complete: bool
    """

    def __init__(
        self, name: str, pick_place_controller, blackboard: Blackboard
    ) -> None:
        """
        Initialize pick-place control behavior.

        Args:
            name: Behavior name
            pick_place_controller: PickPlaceController instance
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.controller = pick_place_controller
        self.executing = False

    def setup(self) -> None:
        """Setup before execution."""
        self.executing = False

    def update(self) -> Status:
        """Execute pick and place."""
        pick_pos = self.blackboard.get("pick_position")
        place_pos = self.blackboard.get("place_position")

        if pick_pos is None or place_pos is None:
            self.feedback_message = "Pick or place position missing"
            return Status.FAILURE

        try:
            if not self.executing:
                # Start pick-and-place
                self.controller.execute(pick_pos, place_pos)
                self.executing = True
                self.blackboard.set("pick_place_status", "executing")
                return Status.RUNNING

            # Check if complete
            if self.controller.is_complete():
                self.blackboard.set("pick_place_status", "complete")
                self.blackboard.set("pick_complete", True)
                self.blackboard.set("place_complete", True)
                self.feedback_message = "Pick and place complete"
                return Status.SUCCESS
            else:
                # Still executing
                return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Pick-place error: {str(e)}"
            self.blackboard.set("pick_place_status", "error")
            return Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        """Cleanup on termination."""
        if self.executing:
            self.controller.stop()
            self.executing = False


class VisualServoControlBehavior(BaseBehavior):
    """
    Execute visual servoing control.

    Wraps the VisualServoController from control.high_level.

    Blackboard Inputs:
        - target_image_features: np.ndarray
        - servo_gain: float (optional)

    Blackboard Outputs:
        - feature_error: float
        - servo_converged: bool
        - control_status: str
    """

    def __init__(
        self,
        name: str,
        visual_servo_controller,
        blackboard: Blackboard,
        convergence_threshold: float = 0.01,
    ) -> None:
        """
        Initialize visual servoing behavior.

        Args:
            name: Behavior name
            visual_servo_controller: VisualServoController instance
            blackboard: Shared state storage
            convergence_threshold: Error threshold for convergence
        """
        super().__init__(name, blackboard)
        self.controller = visual_servo_controller
        self.convergence_threshold = convergence_threshold

    def update(self) -> Status:
        """Execute visual servoing."""
        target_features = self.blackboard.get("target_image_features")

        if target_features is None:
            self.feedback_message = "No target image features"
            return Status.FAILURE

        try:
            # Set servo gain if provided
            gain = self.blackboard.get("servo_gain", None)
            if gain is not None:
                self.controller.set_gain(gain)

            # Execute servo step
            self.controller.set_target_features(target_features)
            error = self.controller.get_feature_error()

            self.blackboard.set("feature_error", error)

            # Check convergence
            converged = error < self.convergence_threshold
            self.blackboard.set("servo_converged", converged)

            if converged:
                self.blackboard.set("control_status", "converged")
                self.feedback_message = (
                    f"Visual servo converged (error: {error:.4f})"
                )
                return Status.SUCCESS
            else:
                self.blackboard.set("control_status", "servoing")
                return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Visual servo error: {str(e)}"
            self.blackboard.set("control_status", "error")
            return Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        """Cleanup on termination."""
        if self.controller is not None:
            self.controller.stop()
