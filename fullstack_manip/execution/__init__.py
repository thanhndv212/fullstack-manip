"""
Behavior Tree Execution Layer for Fullstack Manipulation

This module provides the task orchestration layer using behavior trees
(BTs) to coordinate perception, planning, and control subsystems for
complex manipulation tasks.

Key Components:
- behaviors: Atomic behavior implementations (leaf nodes)
- skills: Composite behaviors (reusable subtrees)
- trees: Complete task definitions
- blackboard: Shared state management
- decorators: BT modifiers (retry, timeout, preconditions)
- executors: BT execution engines
- visualization: Tree debugging and visualization tools
"""

from .blackboard import Blackboard
from .behaviors import BaseBehavior, Status

# Import all behavior types for convenience
from .behaviors.motion_behaviors import (
    MoveToJointConfiguration,
    MoveToCartesianPose,
    ExecuteTrajectory,
)

from .behaviors.gripper_behaviors import (
    OpenGripper,
    CloseGripper,
    VerifyGrasp,
    SetGripperPosition,
)

from .behaviors.perception_behaviors import (
    DetectObject,
    EstimateObjectPose,
    CheckDetectionStatus,
    VisuallyAlign,
    UpdateSceneState,
)

from .behaviors.safety_behaviors import (
    MonitorJointLimits,
    MonitorWorkspaceBounds,
    CheckForceLimit,
    EmergencyStop,
    CheckCollision,
    ValidateTrajectory,
)

from .behaviors.control_behaviors import (
    ImpedanceControlBehavior,
    AdmittanceControlBehavior,
    TrajectoryFollowingBehavior,
    ForceControlBehavior,
    GravityCompensationBehavior,
    PickPlaceControlBehavior,
    VisualServoControlBehavior,
)

__all__ = [
    # Core classes
    "Blackboard",
    "BaseBehavior",
    "Status",
    # Motion behaviors
    "MoveToJointConfiguration",
    "MoveToCartesianPose",
    "ExecuteTrajectory",
    # Gripper behaviors
    "OpenGripper",
    "CloseGripper",
    "VerifyGrasp",
    "SetGripperPosition",
    # Perception behaviors
    "DetectObject",
    "EstimateObjectPose",
    "CheckDetectionStatus",
    "VisuallyAlign",
    "UpdateSceneState",
    # Safety behaviors
    "MonitorJointLimits",
    "MonitorWorkspaceBounds",
    "CheckForceLimit",
    "EmergencyStop",
    "CheckCollision",
    "ValidateTrajectory",
    # Control behaviors
    "ImpedanceControlBehavior",
    "AdmittanceControlBehavior",
    "TrajectoryFollowingBehavior",
    "ForceControlBehavior",
    "GravityCompensationBehavior",
    "PickPlaceControlBehavior",
    "VisualServoControlBehavior",
]
