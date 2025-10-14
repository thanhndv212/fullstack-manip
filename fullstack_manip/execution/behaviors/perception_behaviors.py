"""
Perception behaviors for object detection and pose estimation.

These behaviors interface with vision systems to provide
BT leaf nodes for perception tasks.
"""

from typing import Optional, List, Dict, Any
import numpy as np

from fullstack_manip.execution.behaviors import BaseBehavior, Status
from fullstack_manip.execution.blackboard import Blackboard


class DetectObject(BaseBehavior):
    """
    Detect objects in the scene using vision system.

    Blackboard Inputs:
        - target_object_class: str (optional, specific object to detect)
        - detection_confidence_threshold: float (default 0.7)

    Blackboard Outputs:
        - detected_objects: List[Dict] (object detections)
        - detection_count: int
        - detection_timestamp: float
    """

    def __init__(
        self,
        name: str,
        detector,
        blackboard: Blackboard,
        min_confidence: float = 0.7,
    ) -> None:
        """
        Initialize object detection behavior.

        Args:
            name: Behavior name
            detector: Object detector instance
            blackboard: Shared state storage
            min_confidence: Minimum detection confidence
        """
        super().__init__(name, blackboard)
        self.detector = detector
        self.min_confidence = min_confidence

    def update(self) -> Status:
        """Execute object detection."""
        confidence_threshold = self.blackboard.get(
            "detection_confidence_threshold", self.min_confidence
        )
        target_class = self.blackboard.get("target_object_class", None)

        try:
            # Run detection
            detections = self.detector.detect()

            if detections is None:
                self.feedback_message = "Detection failed"
                return Status.FAILURE

            # Filter by confidence
            filtered = [
                d
                for d in detections
                if d.get("confidence", 0.0) >= confidence_threshold
            ]

            # Filter by class if specified
            if target_class is not None:
                filtered = [
                    d for d in filtered if d.get("class", "") == target_class
                ]

            # Store results
            self.blackboard.set("detected_objects", filtered)
            self.blackboard.set("detection_count", len(filtered))
            self.blackboard.set(
                "detection_timestamp", self.detector.get_timestamp()
            )

            if len(filtered) > 0:
                self.feedback_message = f"Detected {len(filtered)} object(s)"
                return Status.SUCCESS
            else:
                self.feedback_message = "No objects detected"
                return Status.FAILURE

        except Exception as e:
            self.feedback_message = f"Detection error: {str(e)}"
            return Status.FAILURE


class EstimateObjectPose(BaseBehavior):
    """
    Estimate 6D pose of detected objects.

    Blackboard Inputs:
        - detected_objects: List[Dict] (from DetectObject)
        - target_object_id: str (optional, specific object to estimate)

    Blackboard Outputs:
        - object_poses: Dict[str, Dict] (object_id -> pose)
        - target_object_pose: Dict (position, orientation)
        - object_position: np.ndarray [x, y, z]
        - object_orientation: np.ndarray (quaternion [x, y, z, w])
    """

    def __init__(
        self, name: str, pose_estimator, blackboard: Blackboard
    ) -> None:
        """
        Initialize pose estimation behavior.

        Args:
            name: Behavior name
            pose_estimator: Pose estimation instance
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.pose_estimator = pose_estimator

    def update(self) -> Status:
        """Execute pose estimation."""
        detections = self.blackboard.get("detected_objects", [])

        if not detections:
            self.feedback_message = "No detections available for pose est."
            return Status.FAILURE

        target_id = self.blackboard.get("target_object_id", None)

        try:
            # Estimate poses for all detected objects
            poses = {}
            for detection in detections:
                obj_id = detection.get("id", detection.get("class", "unknown"))
                pose = self.pose_estimator.estimate_pose(detection)

                if pose is not None:
                    poses[obj_id] = pose

            self.blackboard.set("object_poses", poses)

            # If target specified, extract its pose
            if target_id is not None:
                if target_id in poses:
                    target_pose = poses[target_id]
                    self.blackboard.set("target_object_pose", target_pose)
                    self.blackboard.set(
                        "object_position", np.array(target_pose["position"])
                    )
                    if "orientation" in target_pose:
                        self.blackboard.set(
                            "object_orientation",
                            np.array(target_pose["orientation"]),
                        )
                    self.feedback_message = f"Estimated pose for {target_id}"
                    return Status.SUCCESS
                else:
                    self.feedback_message = f"Target {target_id} not found"
                    return Status.FAILURE
            else:
                # No specific target, return first object
                if poses:
                    first_obj_id = list(poses.keys())[0]
                    first_pose = poses[first_obj_id]
                    self.blackboard.set(
                        "object_position", np.array(first_pose["position"])
                    )
                    if "orientation" in first_pose:
                        self.blackboard.set(
                            "object_orientation",
                            np.array(first_pose["orientation"]),
                        )
                    self.feedback_message = f"Estimated {len(poses)} poses"
                    return Status.SUCCESS
                else:
                    self.feedback_message = "No poses estimated"
                    return Status.FAILURE

        except Exception as e:
            self.feedback_message = f"Pose estimation error: {str(e)}"
            return Status.FAILURE


class CheckDetectionStatus(BaseBehavior):
    """
    Check if specific object(s) are currently detected.

    Useful as a condition check in behavior trees.

    Blackboard Inputs:
        - detected_objects: List[Dict]
        - required_object_class: str (optional)
        - min_detections: int (default 1)

    Blackboard Outputs:
        - detection_status: bool
    """

    def __init__(
        self,
        name: str,
        blackboard: Blackboard,
        required_class: Optional[str] = None,
        min_count: int = 1,
    ) -> None:
        """
        Initialize detection status check.

        Args:
            name: Behavior name
            blackboard: Shared state storage
            required_class: Required object class (optional)
            min_count: Minimum number of detections required
        """
        super().__init__(name, blackboard)
        self.required_class = required_class
        self.min_count = min_count

    def update(self) -> Status:
        """Check detection status."""
        detections = self.blackboard.get("detected_objects", [])
        required = self.blackboard.get(
            "required_object_class", self.required_class
        )
        min_dets = self.blackboard.get("min_detections", self.min_count)

        # Filter by class if specified
        if required is not None:
            detections = [
                d for d in detections if d.get("class", "") == required
            ]

        # Check count
        status = len(detections) >= min_dets
        self.blackboard.set("detection_status", status)

        if status:
            self.feedback_message = (
                f"Found {len(detections)} detections (>= {min_dets})"
            )
            return Status.SUCCESS
        else:
            self.feedback_message = (
                f"Only {len(detections)} detections (< {min_dets})"
            )
            return Status.FAILURE


class VisuallyAlign(BaseBehavior):
    """
    Use visual servoing to align end-effector with target.

    This behavior performs iterative visual feedback control
    to fine-tune the robot's approach to an object.

    Blackboard Inputs:
        - target_object_pose: Dict (from EstimateObjectPose)
        - alignment_tolerance: float (default 0.005m)
        - max_iterations: int (default 10)

    Blackboard Outputs:
        - alignment_error: float (current error magnitude)
        - alignment_success: bool
        - visual_servo_iterations: int
    """

    def __init__(
        self,
        name: str,
        visual_servo_controller,
        blackboard: Blackboard,
        tolerance: float = 0.005,
        max_iters: int = 10,
    ) -> None:
        """
        Initialize visual servoing behavior.

        Args:
            name: Behavior name
            visual_servo_controller: Visual servo controller instance
            blackboard: Shared state storage
            tolerance: Alignment tolerance (meters)
            max_iters: Maximum servo iterations
        """
        super().__init__(name, blackboard)
        self.servo_controller = visual_servo_controller
        self.tolerance = tolerance
        self.max_iters = max_iters
        self.iteration_count = 0

    def setup(self) -> None:
        """Setup before execution."""
        self.iteration_count = 0

    def update(self) -> Status:
        """Execute visual servoing step."""
        target_pose = self.blackboard.get("target_object_pose")

        if target_pose is None:
            self.feedback_message = "No target pose for visual servoing"
            return Status.FAILURE

        tolerance = self.blackboard.get("alignment_tolerance", self.tolerance)
        max_iters = self.blackboard.get("max_iterations", self.max_iters)

        try:
            # Execute one servo step
            error = self.servo_controller.servo_step(target_pose)

            self.iteration_count += 1
            self.blackboard.set(
                "visual_servo_iterations", self.iteration_count
            )
            self.blackboard.set("alignment_error", error)

            # Check convergence
            if error < tolerance:
                self.blackboard.set("alignment_success", True)
                self.feedback_message = (
                    f"Aligned (error: {error:.4f}m, "
                    f"iters: {self.iteration_count})"
                )
                return Status.SUCCESS

            # Check max iterations
            if self.iteration_count >= max_iters:
                self.blackboard.set("alignment_success", False)
                self.feedback_message = (
                    f"Max iterations reached (error: {error:.4f}m)"
                )
                return Status.FAILURE

            # Continue servoing
            return Status.RUNNING

        except Exception as e:
            self.feedback_message = f"Visual servo error: {str(e)}"
            self.blackboard.set("alignment_success", False)
            return Status.FAILURE

    def terminate(self, new_status: Status) -> None:
        """Cleanup on termination."""
        if self.servo_controller is not None:
            self.servo_controller.stop()


class UpdateSceneState(BaseBehavior):
    """
    Update the scene state representation.

    Consolidates perception information into a scene graph
    or world model representation.

    Blackboard Inputs:
        - detected_objects: List[Dict]
        - object_poses: Dict[str, Dict]

    Blackboard Outputs:
        - scene_state: Dict (updated scene representation)
        - scene_timestamp: float
    """

    def __init__(
        self, name: str, scene_manager, blackboard: Blackboard
    ) -> None:
        """
        Initialize scene state update behavior.

        Args:
            name: Behavior name
            scene_manager: Scene representation manager
            blackboard: Shared state storage
        """
        super().__init__(name, blackboard)
        self.scene_manager = scene_manager

    def update(self) -> Status:
        """Update scene state."""
        detections = self.blackboard.get("detected_objects", [])
        poses = self.blackboard.get("object_poses", {})

        try:
            # Update scene with new information
            self.scene_manager.update_objects(detections, poses)

            # Get updated scene state
            scene_state = self.scene_manager.get_scene_state()

            self.blackboard.set("scene_state", scene_state)
            self.blackboard.set(
                "scene_timestamp", self.scene_manager.get_timestamp()
            )

            self.feedback_message = (
                f"Scene updated with {len(detections)} objects"
            )
            return Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"Scene update error: {str(e)}"
            return Status.FAILURE
