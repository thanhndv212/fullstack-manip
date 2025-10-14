"""Visual servoing controller for image-based manipulation."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Tuple

import numpy as np

from .base_controller import BaseController

if TYPE_CHECKING:  # pragma: no cover - circular import guard
    from ...core.robot import Robot


class VisualServoController(BaseController):
    """Image-based and pose-based visual servoing for alignment tasks.

    This controller integrates with the camera calibration system to
    perform visual servoing. Supports both position-based (PBVS) and
    image-based (IBVS) visual servoing strategies.

    Attributes:
        camera_matrix: Camera intrinsic matrix (3x3).
        servo_gain: Control gain for visual servoing (0.0 to 1.0).
        feature_tolerance: Pixel error tolerance for convergence.
    """

    def __init__(
        self,
        robot: "Robot",
        camera_matrix: Optional[np.ndarray] = None,
        servo_gain: float = 0.1,
        feature_tolerance: float = 5.0,
    ) -> None:
        """Initialize the visual servoing controller.

        Args:
            robot: Robot instance to control.
            camera_matrix: Camera intrinsic parameters (3x3). If None,
                uses default parameters.
            servo_gain: Visual servoing control gain (0 < gain <= 1).
            feature_tolerance: Pixel error threshold for convergence.
        """
        super().__init__(robot)

        if servo_gain <= 0 or servo_gain > 1:
            raise ValueError("Servo gain must be in (0, 1]")
        if feature_tolerance <= 0:
            raise ValueError("Feature tolerance must be positive")

        self.camera_matrix = (
            camera_matrix
            if camera_matrix is not None
            else self._default_camera_matrix()
        )
        self.servo_gain = servo_gain
        self.feature_tolerance = feature_tolerance

    def execute(
        self,
        target_features: np.ndarray,
        max_iterations: int = 100,
    ) -> bool:
        """Execute visual servoing to align with target features.

        Args:
            target_features: Target image features (e.g., [u, v] pixels).
            max_iterations: Maximum servoing iterations.

        Returns:
            True if successfully converged, False otherwise.
        """
        if not isinstance(target_features, np.ndarray):
            raise ValueError("Target features must be numpy array")

        for iteration in range(max_iterations):
            current_features = self._extract_current_features()

            if current_features is None:
                print("Failed to extract current features")
                return False

            feature_error = target_features - current_features
            error_norm = np.linalg.norm(feature_error)

            if error_norm < self.feature_tolerance:
                print(f"Visual servoing converged in {iteration} iterations")
                return True

            # Compute velocity from image error
            velocity = self._compute_control_velocity(feature_error)
            self._apply_velocity_command(velocity)

        print(f"Visual servoing did not converge in {max_iterations} iters")
        return False

    def move_to_visual_target(
        self,
        target_marker_id: int,
        offset: Optional[np.ndarray] = None,
    ) -> bool:
        """Move end-effector to align with detected visual marker.

        Args:
            target_marker_id: ID of ArUco marker to track.
            offset: Optional offset from marker pose [x, y, z].

        Returns:
            True if successfully reached target, False otherwise.
        """
        marker_pose = self._detect_marker_pose(target_marker_id)

        if marker_pose is None:
            print(f"Could not detect marker {target_marker_id}")
            return False

        target_pos, target_orient = marker_pose

        if offset is not None:
            if not isinstance(offset, np.ndarray) or offset.shape != (3,):
                raise ValueError("Offset must be 3D numpy array")
            target_pos = target_pos + offset

        try:
            self.robot.move_to_position(target_pos, target_orient)
            return True
        except RuntimeError as e:
            print(f"Failed to move to visual target: {e}")
            return False

    def track_feature(
        self,
        feature_extractor,
        duration: float = 10.0,
    ) -> None:
        """Continuously track a moving feature.

        Args:
            feature_extractor: Callable that extracts current features
                from camera image.
            duration: Duration to track (seconds).
        """
        steps = int(duration / self.robot.dt)

        for _ in range(steps):
            current_features = feature_extractor()

            if current_features is None:
                continue

            # Compute desired features (e.g., image center)
            target_features = self._get_centered_features()
            feature_error = target_features - current_features

            velocity = self._compute_control_velocity(feature_error)
            self._apply_velocity_command(velocity)

    def align_with_marker(
        self,
        marker_id: int,
        alignment_axis: str = "z",
        distance: float = 0.15,
    ) -> bool:
        """Align end-effector with marker at specified distance.

        Args:
            marker_id: ArUco marker ID to align with.
            alignment_axis: Axis to align ('x', 'y', or 'z').
            distance: Desired distance from marker (meters).

        Returns:
            True if alignment successful, False otherwise.
        """
        if alignment_axis not in ["x", "y", "z"]:
            raise ValueError("Alignment axis must be 'x', 'y', or 'z'")
        if distance <= 0:
            raise ValueError("Distance must be positive")

        marker_pose = self._detect_marker_pose(marker_id)

        if marker_pose is None:
            return False

        marker_pos, marker_orient = marker_pose

        # Compute target position along alignment axis
        axis_map = {"x": 0, "y": 1, "z": 2}
        axis_idx = axis_map[alignment_axis]

        offset = np.zeros(3)
        offset[axis_idx] = distance

        # Rotate offset by marker orientation (simplified)
        target_pos = marker_pos + offset

        try:
            self.robot.move_to_position(target_pos, marker_orient)
            return True
        except RuntimeError:
            return False

    def _default_camera_matrix(self) -> np.ndarray:
        """Return default camera intrinsic matrix."""
        # Default camera parameters (focal length, principal point)
        fx, fy = 800.0, 800.0
        cx, cy = 320.0, 240.0
        return np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32
        )

    def _extract_current_features(self) -> Optional[np.ndarray]:
        """Extract current image features from camera.

        Returns:
            Current feature vector or None if extraction fails.
        """
        # Placeholder - would integrate with actual camera module
        # from fullstack_manip.state_estimation.camera
        return np.array([320.0, 240.0])  # Image center as placeholder

    def _detect_marker_pose(
        self,
        marker_id: int,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Detect ArUco marker and return its pose.

        Args:
            marker_id: ID of marker to detect.

        Returns:
            Tuple of (position, orientation) or None if not detected.
        """
        # Placeholder - would use camera calibration module
        # from fullstack_manip.state_estimation.camera.calibration
        return None

    def _compute_control_velocity(
        self,
        feature_error: np.ndarray,
    ) -> np.ndarray:
        """Compute control velocity from feature error.

        Args:
            feature_error: Error in image features.

        Returns:
            Control velocity in Cartesian space.
        """
        # Simplified IBVS: velocity = -lambda * L^+ * error
        # where L is the interaction matrix (image Jacobian)
        interaction_matrix = self._compute_interaction_matrix()

        try:
            velocity = (
                -self.servo_gain
                * np.linalg.pinv(interaction_matrix)
                @ feature_error
            )
        except np.linalg.LinAlgError:
            velocity = np.zeros(6)

        return velocity

    def _compute_interaction_matrix(self) -> np.ndarray:
        """Compute image Jacobian (interaction matrix).

        Returns:
            Interaction matrix L relating feature velocities to
            camera velocities.
        """
        # Simplified 2D point feature interaction matrix
        # For a point at pixel (u, v) with depth Z:
        # L = [[-1/Z, 0, u/Z, uv, -(1+u^2), v],
        #      [0, -1/Z, v/Z, 1+v^2, -uv, -u]]
        return np.eye(6) * 0.01  # Placeholder

    def _apply_velocity_command(self, velocity: np.ndarray) -> None:
        """Apply velocity command to robot.

        Args:
            velocity: Cartesian velocity [vx, vy, vz, wx, wy, wz].
        """
        # Get current pose
        ee_pos, ee_orient = self.robot.get_body_pose(
            self.robot.end_effector_name
        )

        # Integrate velocity for small displacement
        dt = self.robot.dt
        new_pos = ee_pos + velocity[:3] * dt

        # Apply motion
        self.robot.move_to_position(new_pos, ee_orient, duration=dt)

    def _get_centered_features(self) -> np.ndarray:
        """Get features for centered alignment (image center)."""
        # Assuming 640x480 camera
        return np.array([320.0, 240.0])

    def reset(self) -> None:
        """Reset visual servoing state."""
        pass


__all__ = ["VisualServoController"]
