"""Unit tests for the calibration scripts shared helpers."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from fullstack_manip.state_estimation.camera.calibration.scripts import (
    load_calibration_parameters,
    parse_resolution,
    resolve_camera_output_prefix,
)


class ParseResolutionTests(unittest.TestCase):
    def test_parse_resolution_standard_formats(self) -> None:
        self.assertEqual(parse_resolution("1280x720"), (1280, 720))
        self.assertEqual(parse_resolution("1920,1080"), (1920, 1080))
        self.assertEqual(parse_resolution("3840X2160"), (3840, 2160))

    def test_parse_resolution_invalid(self) -> None:
        with self.assertRaises(ValueError):
            parse_resolution("foo")


class LoadCalibrationParametersTests(unittest.TestCase):
    def test_load_from_file(self) -> None:
        payload = {
            "camera_matrix": np.eye(3, dtype=np.float32).tolist(),
            "dist_coeffs": np.zeros(5, dtype=np.float32).tolist(),
            "image_size": [640, 480],
            "fov": 60.0,
        }
        with tempfile.TemporaryDirectory() as tmp:
            json_path = Path(tmp) / "sample.json"
            json_path.write_text(json.dumps(payload), encoding="utf-8")
            result = load_calibration_parameters(
                "sample",
                calibration_file=json_path,
                allow_fallback=False,
            )
        self.assertEqual(result["resolution"], (640, 480))
        self.assertIn("camera_matrix", result)

    def test_load_fallback(self) -> None:
        result = load_calibration_parameters(
            "missing_camera",
            allow_fallback=True,
            fallback_resolution=(800, 600),
            fallback_fov=75.0,
        )
        self.assertEqual(result["resolution"], (800, 600))
        self.assertAlmostEqual(result["fov"], 75.0)
        self.assertEqual(result["metadata"]["source"], "fallback")


class ResolveOutputPrefixTests(unittest.TestCase):
    def test_prefix_creation(self) -> None:
        prefix = resolve_camera_output_prefix(
            "Test Camera",
            suffix="session",
            timestamp=False,
        )
        self.assertTrue(prefix.parent.exists())
        self.assertTrue(prefix.name.startswith("test_camera_session"))


if __name__ == "__main__":
    unittest.main()
