"""
Motion behaviors that interface with the motion planning system.

These behaviors wrap the MotionPlanner to provide BT leaf nodes
for motion-related actions.
"""

from typing import Optional
import numpy as np

from fullstack_manip.planning.motion_planner import MotionPlanner
from fullstack_manip.execution.behaviors import BaseBehavior, Status
from fullstack_manip.execution.blackboard import Blackboard


class MoveToJointConfiguration(BaseBehavior):
    """
    Plan and execute motion to a target joint configuration.

    Blackboard Inputs:
        - target_joint_config: np.ndarray of target joint positions

    Blackboard Outputs:
        - planned_trajectory: List of joint configurations
        - motion_status: str describing result
    """

    def __init__(
        self, name: str, motion_planner: MotionPlanner, blackboard: Blackboard
    ) -> None:
        """
        Initialize move to joint configuration behavior.

        Args:
            name: Behavior name
            motion_planner: Motion planning instance
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.planner = motion_planner

    def update(self) -> Status:
        """Execute motion planning to joint configuration."""
        target = self.blackboard.get("target_joint_config")

        if target is None:
            self.feedback_message = "No target joint config on blackboard"
            return Status.FAILURE

        try:
            # Plan trajectory using motion planner
            trajectory = self.planner.plan_to_joint_configuration(
                target_q=target
            )

            if trajectory is not None and len(trajectory) > 0:
                self.blackboard.set("planned_trajectory", trajectory)
                self.blackboard.set("motion_status", "planning_success")
                self.feedback_message = "Successfully planned trajectory"
                return Status.SUCCESS
            else:
                self.feedback_message = "Planning failed"
                self.blackboard.set("motion_status", "planning_failed")
                return Status.FAILURE

        except Exception as e:
            self.feedback_message = f"Planning error: {str(e)}"
            self.blackboard.set("motion_status", "planning_error")
            return Status.FAILURE


class MoveToCartesianPose(BaseBehavior):
    """
    Plan and execute motion to a target Cartesian pose.

    Blackboard Inputs:
        - target_position: np.ndarray [x, y, z]
        - target_orientation: Optional np.ndarray (quaternion)

    Blackboard Outputs:
        - planned_trajectory: List of joint configurations
        - motion_status: str describing result
    """

    def __init__(
        self,
        name: str,
        motion_planner: MotionPlanner,
        blackboard: Blackboard,
        orientation_required: bool = False,
    ) -> None:
        """
        Initialize move to Cartesian pose behavior.

        Args:
            name: Behavior name
            motion_planner: Motion planning instance
            blackboard: Shared state storage
            orientation_required: Whether orientation must be specified
        """
        super().__init__(name, blackboard)
        self.planner = motion_planner
        self.orientation_required = orientation_required

    def update(self) -> Status:
        """Execute motion planning to Cartesian pose."""
        position = self.blackboard.get("target_position")
        orientation = self.blackboard.get("target_orientation")

        if position is None:
            self.feedback_message = "No target position on blackboard"
            return Status.FAILURE

        if self.orientation_required and orientation is None:
            self.feedback_message = "Orientation required but not provided"
            return Status.FAILURE

        try:
            # Plan trajectory using IK-based planning
            trajectory = self.planner.plan_to_pose(
                target_position=position, target_orientation=orientation
            )

            if trajectory is not None and len(trajectory) > 0:
                self.blackboard.set("planned_trajectory", trajectory)
                self.blackboard.set("motion_status", "planning_success")
                self.feedback_message = "Successfully planned trajectory"
                return Status.SUCCESS
            else:
                self.feedback_message = "Planning failed"
                self.blackboard.set("motion_status", "planning_failed")
                return Status.FAILURE

        except Exception as e:
            self.feedback_message = f"Planning error: {str(e)}"
            self.blackboard.set("motion_status", "planning_error")
            return Status.FAILURE


class ExecuteTrajectory(BaseBehavior):
    """
    Execute a planned trajectory using trajectory controller.

    Blackboard Inputs:
        - planned_trajectory: List of joint configurations

    Blackboard Outputs:
        - execution_status: str describing result
        - execution_progress: float (0.0 to 1.0)
    """

    def __init__(
        self, name: str, trajectory_controller, blackboard: Blackboard
    ) -> None:
        """
        Initialize trajectory execution behavior.

        Args:
            name: Behavior name
            trajectory_controller: Controller for trajectory following
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.controller = trajectory_controller
        self.executing = False
        self.trajectory_index = 0

    def setup(self) -> None:
        """Setup before first execution."""
        self.executing = False
        self.trajectory_index = 0

    def update(self) -> Status:
        """Execute trajectory following."""
        trajectory = self.blackboard.get("planned_trajectory")

        if trajectory is None:
            self.feedback_message = "No trajectory on blackboard"
            return Status.FAILURE

        try:
            if not self.executing:
                # Start trajectory execution
                self.controller.start_trajectory(trajectory)
                self.executing = True
                self.blackboard.set("execution_status", "executing")
                return Status.RUNNING

            # Check execution progress
            if self.controller.is_trajectory_complete():
                self.executing = False
                self.blackboard.set("execution_status", "complete")
                self.blackboard.set("execution_progress", 1.0)
                self.feedback_message = "Trajectory execution complete"
                return Status.SUCCESS

            # Update progress
            progress = self.controller.get_progress()
            self.blackboard.set("execution_progress", progress)
            return Status.RUNNING

        except Exception as e:
            self.executing = False
            self.feedback_message = f"Execution error: {str(e)}"
            self.blackboard.set("execution_status", "error")
            return Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        """Cleanup on termination."""
        if self.executing:
            self.controller.stop()
            self.executing = False
