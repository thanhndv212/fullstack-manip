"""
Pick skill: Composite behavior tree for object grasping.

This skill orchestrates the complete pick sequence including approach,
visual alignment, grasping, and verification.
"""

from typing import Any, Dict
import numpy as np

try:
    from py_trees.composites import Sequence
    from py_trees.decorators import Retry, Timeout
    from py_trees import Behaviour

    HAS_PY_TREES = True
except ImportError:
    HAS_PY_TREES = False
    # Fallback to custom implementations
    Sequence = None
    Retry = None
    Timeout = None
    Behaviour = None

from fullstack_manip.execution.behaviors.motion_behaviors import (
    MoveToCartesianPose,
    ExecuteTrajectory,
)
from fullstack_manip.execution.blackboard import Blackboard


class PickSkill:
    """
    Reusable pick skill as a behavior tree subtree.

    This skill implements the standard pick sequence:
    1. Move above object (with safety offset)
    2. Visual alignment (with retries)
    3. Move to pre-grasp pose
    4. Close gripper
    5. Verify grasp (force feedback)
    6. Lift object

    Blackboard Inputs:
        - object_position: np.ndarray [x, y, z]
        - approach_offset: float (default 0.1m above object)
        - pre_grasp_offset: float (default 0.05m above object)
        - lift_height: float (default 0.15m)

    Blackboard Outputs:
        - grasp_success: bool
        - picked_object_id: str (if available)
    """

    @staticmethod
    def create(
        motion_planner,
        gripper_controller,
        force_sensor,
        blackboard: Blackboard,
        max_retries: int = 3,
        timeout_sec: float = 30.0,
    ):
        """
        Create pick skill behavior tree.

        Args:
            motion_planner: Motion planning instance
            gripper_controller: Gripper control instance
            force_sensor: Force/torque sensor for grasp verification
            blackboard: Shared state storage
            max_retries: Max attempts for visual alignment
            timeout_sec: Overall timeout for pick sequence

        Returns:
            Behavior tree root node for pick skill
        """
        if not HAS_PY_TREES:
            raise ImportError(
                "py_trees not installed. " "Install with: pip install py-trees"
            )

        # Create behavior sequence
        pick_sequence = Sequence(
            name="Pick Object Skill",
            children=[
                # 1. Move above object with safety margin
                MoveAboveObject(
                    name="Move Above Object",
                    motion_planner=motion_planner,
                    blackboard=blackboard,
                ),
                # 2. Visual alignment with retries
                Retry(
                    name="Visual Alignment with Retry",
                    child=VisuallyAlignToObject(
                        name="Visual Alignment",
                        vision_system=None,  # TODO: Add vision system
                        blackboard=blackboard,
                    ),
                    num_failures=max_retries,
                ),
                # 3. Move to pre-grasp pose
                MoveToPreGrasp(
                    name="Move to Pre-Grasp",
                    motion_planner=motion_planner,
                    blackboard=blackboard,
                ),
                # 4. Close gripper
                CloseGripper(
                    name="Close Gripper",
                    gripper=gripper_controller,
                    blackboard=blackboard,
                ),
                # 5. Verify grasp succeeded
                VerifyGrasp(
                    name="Verify Grasp",
                    force_sensor=force_sensor,
                    blackboard=blackboard,
                ),
                # 6. Lift object
                LiftObject(
                    name="Lift Object",
                    motion_planner=motion_planner,
                    blackboard=blackboard,
                ),
            ],
        )

        # Wrap with timeout
        pick_with_timeout = Timeout(
            name=f"Pick Skill (timeout={timeout_sec}s)",
            child=pick_sequence,
            duration=timeout_sec,
        )

        return pick_with_timeout


# Example atomic behaviors for pick skill
# (These would typically be in separate files)


class MoveAboveObject(Behaviour):
    """Move end-effector above object with safety offset."""

    def __init__(self, name, motion_planner, blackboard):
        super().__init__(name)
        self.planner = motion_planner
        self.blackboard = blackboard

    def update(self):
        from py_trees.common import Status

        obj_pos = self.blackboard.get("object_position")
        offset = self.blackboard.get("approach_offset", 0.1)

        if obj_pos is None:
            return Status.FAILURE

        # Target position above object
        target = obj_pos + np.array([0, 0, offset])
        self.blackboard.set("target_position", target)

        # Plan and execute
        trajectory = self.planner.plan_to_pose(target_position=target)
        if trajectory is not None:
            return Status.SUCCESS
        return Status.FAILURE


class MoveToPreGrasp(Behaviour):
    """Move to pre-grasp pose close to object."""

    def __init__(self, name, motion_planner, blackboard):
        super().__init__(name)
        self.planner = motion_planner
        self.blackboard = blackboard

    def update(self):
        from py_trees.common import Status

        obj_pos = self.blackboard.get("object_position")
        offset = self.blackboard.get("pre_grasp_offset", 0.05)

        if obj_pos is None:
            return Status.FAILURE

        target = obj_pos + np.array([0, 0, offset])
        trajectory = self.planner.plan_to_pose(target_position=target)

        if trajectory is not None:
            self.blackboard.set("target_position", target)
            return Status.SUCCESS
        return Status.FAILURE


class CloseGripper(Behaviour):
    """Close gripper to grasp object."""

    def __init__(self, name, gripper, blackboard):
        super().__init__(name)
        self.gripper = gripper
        self.blackboard = blackboard

    def update(self):
        from py_trees.common import Status

        try:
            self.gripper.close()
            # TODO: Check if gripper actually closed
            return Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Gripper error: {str(e)}"
            return Status.FAILURE


class VerifyGrasp(Behaviour):
    """Verify object was successfully grasped using force feedback."""

    def __init__(self, name, force_sensor, blackboard):
        super().__init__(name)
        self.force_sensor = force_sensor
        self.blackboard = blackboard

    def update(self):
        from py_trees.common import Status

        # Check force/torque readings
        force = self.force_sensor.get_force()
        threshold = self.blackboard.get("grasp_force_threshold", 1.0)

        if force is not None and np.linalg.norm(force) > threshold:
            self.blackboard.set("grasp_success", True)
            return Status.SUCCESS

        self.blackboard.set("grasp_success", False)
        return Status.FAILURE


class LiftObject(Behaviour):
    """Lift grasped object to safe height."""

    def __init__(self, name, motion_planner, blackboard):
        super().__init__(name)
        self.planner = motion_planner
        self.blackboard = blackboard

    def update(self):
        from py_trees.common import Status

        current_pos = self.blackboard.get("current_ee_position")
        lift_height = self.blackboard.get("lift_height", 0.15)

        if current_pos is None:
            return Status.FAILURE

        target = current_pos + np.array([0, 0, lift_height])
        trajectory = self.planner.plan_to_pose(target_position=target)

        if trajectory is not None:
            return Status.SUCCESS
        return Status.FAILURE


class VisuallyAlignToObject(Behaviour):
    """Use visual servoing to align with object."""

    def __init__(self, name, vision_system, blackboard):
        super().__init__(name)
        self.vision = vision_system
        self.blackboard = blackboard

    def update(self):
        from py_trees.common import Status

        # TODO: Implement visual servoing
        # This would use camera feedback to fine-tune approach
        return Status.SUCCESS
