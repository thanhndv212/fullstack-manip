"""Smoke tests for camera window utilities.

This module mirrors the legacy FIGAROH camera window script while
providing guard rails so automated environments can skip the test when
camera hardware or GUI support is unavailable.
"""

from __future__ import annotations

import time
import unittest

try:  # pragma: no cover - optional dependency
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - handled via skip
    cv2 = None  # type: ignore


@unittest.skipIf(cv2 is None, "OpenCV is not installed")
class CameraWindowSmokeTest(unittest.TestCase):
    """Ensure the OpenCV camera feed can be opened briefly."""

    def test_camera_window_smoke(self) -> None:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.skipTest("No accessible camera to validate")

        frame_count = 0
        success = False

        try:
            deadline = time.time() + 5.0
            while time.time() < deadline and frame_count < 30:
                ret, frame = cap.read()
                if not ret or frame is None:
                    continue
                frame_count += 1
                success = True
                break
        finally:
            cap.release()
            cv2.destroyAllWindows()

        self.assertTrue(success, "Camera opened but failed to supply frames")
