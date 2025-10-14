"""
Gripper behaviors for opening, closing, and verifying grasps.

These behaviors provide BT leaf nodes for gripper control operations.
"""

from typing import Optional
import numpy as np

from fullstack_manip.execution.behaviors import BaseBehavior, Status
from fullstack_manip.execution.blackboard import Blackboard


class OpenGripper(BaseBehavior):
    """
    Open the gripper to release an object.

    Blackboard Inputs:
        - open_position: float (optional, default from gripper)
        - gripper_speed: float (optional)

    Blackboard Outputs:
        - gripper_state: str ("open", "closed", "moving")
        - gripper_position: float
    """

    def __init__(
        self,
        name: str,
        gripper_controller,
        blackboard: Blackboard,
        default_open_position: float = 1.0,
    ) -> None:
        """
        Initialize open gripper behavior.

        Args:
            name: Behavior name
            gripper_controller: Gripper control instance
            blackboard: Shared state storage
            default_open_position: Default open position if not on blackboard
        """
        super().__init__(name, blackboard)
        self.gripper = gripper_controller
        self.default_open_position = default_open_position

    def update(self) -> Status:
        """Execute gripper opening."""
        open_pos = self.blackboard.get(
            "open_position", self.default_open_position
        )

        try:
            # Command gripper to open
            self.gripper.set_position(open_pos)

            # Check if movement is complete
            current_pos = self.gripper.get_position()
            tolerance = 0.01  # 1% tolerance

            if abs(current_pos - open_pos) < tolerance:
                self.blackboard.set("gripper_state", "open")
                self.blackboard.set("gripper_position", current_pos)
                self.feedback_message = "Gripper opened successfully"
                return Status.SUCCESS
            else:
                self.blackboard.set("gripper_state", "moving")
                self.blackboard.set("gripper_position", current_pos)
                return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Gripper open error: {str(e)}"
            return Status.FAILURE


class CloseGripper(BaseBehavior):
    """
    Close the gripper to grasp an object.

    Blackboard Inputs:
        - close_position: float (optional, default from gripper)
        - gripper_speed: float (optional)
        - max_force: float (optional, for force-controlled grasping)

    Blackboard Outputs:
        - gripper_state: str ("open", "closed", "moving")
        - gripper_position: float
        - gripper_force: float (if available)
    """

    def __init__(
        self,
        name: str,
        gripper_controller,
        blackboard: Blackboard,
        default_close_position: float = 0.0,
        force_controlled: bool = False,
    ) -> None:
        """
        Initialize close gripper behavior.

        Args:
            name: Behavior name
            gripper_controller: Gripper control instance
            blackboard: Shared state storage
            default_close_position: Default close position
            force_controlled: Whether to use force control
        """
        super().__init__(name, blackboard)
        self.gripper = gripper_controller
        self.default_close_position = default_close_position
        self.force_controlled = force_controlled

    def update(self) -> Status:
        """Execute gripper closing."""
        close_pos = self.blackboard.get(
            "close_position", self.default_close_position
        )

        try:
            if self.force_controlled:
                # Force-controlled grasping
                max_force = self.blackboard.get("max_force", 50.0)
                self.gripper.set_force(max_force)
                self.gripper.close_with_force_limit()

                # Check if object is grasped
                current_force = self.gripper.get_force()
                if current_force is not None:
                    self.blackboard.set("gripper_force", current_force)

                    # If force is detected, grasp is successful
                    if current_force > 0.1:  # Threshold for contact
                        self.blackboard.set("gripper_state", "closed")
                        self.feedback_message = "Object grasped"
                        return Status.SUCCESS
            else:
                # Position-controlled grasping
                self.gripper.set_position(close_pos)

                current_pos = self.gripper.get_position()
                tolerance = 0.01

                if abs(current_pos - close_pos) < tolerance:
                    self.blackboard.set("gripper_state", "closed")
                    self.blackboard.set("gripper_position", current_pos)
                    self.feedback_message = "Gripper closed"
                    return Status.SUCCESS
                else:
                    self.blackboard.set("gripper_state", "moving")
                    self.blackboard.set("gripper_position", current_pos)
                    return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Gripper close error: {str(e)}"
            return Status.FAILURE


class VerifyGrasp(BaseBehavior):
    """
    Verify that an object has been successfully grasped.

    Uses force/torque feedback to determine grasp success.

    Blackboard Inputs:
        - grasp_force_threshold: float (minimum force for success)
        - use_visual_verification: bool (optional)

    Blackboard Outputs:
        - grasp_success: bool
        - grasp_force: float
        - grasped_object_id: str (if visual verification used)
    """

    def __init__(
        self,
        name: str,
        force_sensor,
        blackboard: Blackboard,
        default_threshold: float = 1.0,
        vision_system=None,
    ) -> None:
        """
        Initialize grasp verification behavior.

        Args:
            name: Behavior name
            force_sensor: Force/torque sensor instance
            blackboard: Shared state storage
            default_threshold: Default force threshold (N)
            vision_system: Optional vision system for visual verification
        """
        super().__init__(name, blackboard)
        self.force_sensor = force_sensor
        self.vision_system = vision_system
        self.default_threshold = default_threshold

    def update(self) -> Status:
        """Execute grasp verification."""
        threshold = self.blackboard.get(
            "grasp_force_threshold", self.default_threshold
        )

        try:
            # Check force feedback
            force = self.force_sensor.get_force()

            if force is None:
                self.feedback_message = "Force sensor not available"
                return Status.FAILURE

            force_magnitude = (
                np.linalg.norm(force)
                if isinstance(force, np.ndarray)
                else abs(force)
            )

            self.blackboard.set("grasp_force", force_magnitude)

            # Visual verification if available
            use_visual = self.blackboard.get("use_visual_verification", False)
            if use_visual and self.vision_system is not None:
                # Check if object is still visible in gripper
                in_gripper = self.vision_system.check_object_in_gripper()
                if not in_gripper:
                    self.blackboard.set("grasp_success", False)
                    self.feedback_message = "Object not in gripper"
                    return Status.FAILURE

            # Verify force threshold
            if force_magnitude >= threshold:
                self.blackboard.set("grasp_success", True)
                self.feedback_message = (
                    f"Grasp verified (force: {force_magnitude:.2f}N)"
                )
                return Status.SUCCESS
            else:
                self.blackboard.set("grasp_success", False)
                self.feedback_message = (
                    f"Grasp failed (force: {force_magnitude:.2f}N "
                    f"< threshold: {threshold:.2f}N)"
                )
                return Status.FAILURE

        except Exception as e:
            self.feedback_message = f"Grasp verification error: {str(e)}"
            self.blackboard.set("grasp_success", False)
            return Status.FAILURE


class SetGripperPosition(BaseBehavior):
    """
    Set gripper to a specific position.

    More general than OpenGripper/CloseGripper for intermediate positions.

    Blackboard Inputs:
        - target_gripper_position: float (required)

    Blackboard Outputs:
        - gripper_position: float
        - gripper_state: str
    """

    def __init__(
        self, name: str, gripper_controller, blackboard: Blackboard
    ) -> None:
        """
        Initialize set gripper position behavior.

        Args:
            name: Behavior name
            gripper_controller: Gripper control instance
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.gripper = gripper_controller

    def update(self) -> Status:
        """Execute gripper position setting."""
        target = self.blackboard.get("target_gripper_position")

        if target is None:
            self.feedback_message = "No target gripper position on blackboard"
            return Status.FAILURE

        try:
            self.gripper.set_position(target)
            current_pos = self.gripper.get_position()

            tolerance = 0.01
            if abs(current_pos - target) < tolerance:
                self.blackboard.set("gripper_position", current_pos)
                self.blackboard.set("gripper_state", "at_target")
                self.feedback_message = (
                    f"Gripper at position {current_pos:.3f}"
                )
                return Status.SUCCESS
            else:
                self.blackboard.set("gripper_position", current_pos)
                self.blackboard.set("gripper_state", "moving")
                return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Gripper position error: {str(e)}"
            return Status.FAILURE
