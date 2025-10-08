#!/usr/bin/env python3
"""Estimate camera intrinsics when a full calibration isn't available."""

from __future__ import annotations

import argparse
import os
import re
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

DEFAULT_CAMERA_SCRIPT = Path(__file__).resolve().parents[1].joinpath(
    "scripts", "camera_data_collection.py"
)

MATRIX_PATTERN = re.compile(
    r"camera_matrix = np\.array\([\s\S]*?dtype=np\.float32\)",
    re.MULTILINE,
)
DIST_PATTERN = re.compile(
    r"dist_coeffs = np\.[\s\S]*?dtype=np\.float32\)",
    re.MULTILINE,
)
DEFAULT_DIST_SNIPPET = "dist_coeffs = np.zeros((4, 1))"


class WebcamParameterEstimator:
    """Helpers for inferring camera intrinsics for common webcams."""

    @staticmethod
    def get_typical_parameters(
        resolution: Tuple[int, int] = (1280, 720),
        fov_degrees: float = 60.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return a heuristic camera matrix and zero distortion terms."""

        width, height = resolution
        fov_rad = np.radians(fov_degrees)
        fx = width / (2 * np.tan(fov_rad / 2))
        fy = fx
        cx = width / 2.0
        cy = height / 2.0

        camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        dist_coeffs = np.zeros((5,), dtype=np.float32)
        return camera_matrix, dist_coeffs

    @staticmethod
    def get_webcam_presets() -> Dict[str, Dict[str, object]]:
        """Return predefined presets for popular webcam configurations."""

        presets = {
            "basic_720p": {
                "resolution": (1280, 720),
                "fov": 60.0,
                "description": "Basic 720p webcam",
            },
            "wide_720p": {
                "resolution": (1280, 720),
                "fov": 78.0,
                "description": "Wide-angle 720p webcam",
            },
            "basic_1080p": {
                "resolution": (1920, 1080),
                "fov": 65.0,
                "description": "Basic 1080p webcam",
            },
            "wide_1080p": {
                "resolution": (1920, 1080),
                "fov": 78.0,
                "description": "Wide-angle 1080p webcam",
            },
            "ultrawide": {
                "resolution": (1280, 720),
                "fov": 90.0,
                "description": "Ultra-wide webcam",
            },
        }

        results: Dict[str, Dict[str, object]] = {}
        for name, params in presets.items():
            camera_matrix, dist_coeffs = (
                WebcamParameterEstimator.get_typical_parameters(
                    params["resolution"],
                    params["fov"],
                )
            )
            results[name] = {
                "camera_matrix": camera_matrix,
                "dist_coeffs": dist_coeffs,
                "resolution": params["resolution"],
                "fov": params["fov"],
                "description": params["description"],
            }
        return results

    @staticmethod
    def detect_camera_resolution(
        camera_id: int = 0,
    ) -> Optional[Tuple[int, int]]:
        """Probe the camera for its active resolution."""

        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        ret, frame = cap.read()
        cap.release()

        if not ret:
            return None

        height, width = frame.shape[:2]
        return (width, height)

    @staticmethod
    def estimate_fov_from_detection(
        camera_id: int = 0,
        known_object_size: float = 0.1,
        known_distance: float = 0.5,
    ) -> Optional[float]:
        """Estimate field of view interactively using a known object."""

        print("📐 FOV Estimation Setup:")
        print(f"   1. Place an object of size {known_object_size*100:.1f}cm")
        print(f"   2. Position it {known_distance*100:.1f}cm from camera")
        print("   3. Measure width in pixels")
        print("   4. Press 'q' to quit measurement")

        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            return None

        frame = None
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            height, width = frame.shape[:2]
            cv2.line(
                frame,
                (width // 2, 0),
                (width // 2, height),
                (0, 255, 0),
                1,
            )
            cv2.line(
                frame,
                (0, height // 2),
                (width, height // 2),
                (0, 255, 0),
                1,
            )
            cv2.putText(
                frame,
                "Measure object width in pixels",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            obj_text = (
                f"Object: {known_object_size*100:.1f}cm at "
                f"{known_distance*100:.1f}cm"
            )
            cv2.putText(
                frame,
                obj_text,
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                frame,
                "Press 'q' to quit",
                (10, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            cv2.imshow("FOV Estimation", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()

        if frame is None:
            return None

        try:
            prompt = "Measured object width in pixels: "
            pixel_width = float(input(prompt))
        except ValueError:
            return None

        angular_size = 2 * np.arctan(
            known_object_size / (2 * known_distance)
        )
        pixels_per_rad = pixel_width / angular_size
        fov_rad = frame.shape[1] / pixels_per_rad
        return float(np.degrees(fov_rad))


def auto_detect_camera_parameters(
    camera_id: int = 0,
) -> Optional[Dict[str, object]]:
    """Detect camera resolution and derive plausible intrinsics."""

    print("🔍 Auto-detecting camera parameters...")
    resolution = WebcamParameterEstimator.detect_camera_resolution(camera_id)
    if resolution is None:
        print("❌ Could not detect camera resolution")
        return None

    width, height = resolution
    print(f"📊 Detected resolution: {width} x {height}")

    presets = WebcamParameterEstimator.get_webcam_presets()
    best_match: Optional[Tuple[str, Dict[str, object]]] = None
    for name, preset in presets.items():
        if preset["resolution"] == resolution:
            if best_match is None or preset["fov"] == 60.0:
                best_match = (name, preset)

    if best_match is None:
        camera_matrix, dist_coeffs = (
            WebcamParameterEstimator.get_typical_parameters(resolution)
        )
        description = f"Generic parameters for {width}x{height}"
        return {
            "camera_matrix": camera_matrix,
            "dist_coeffs": dist_coeffs,
            "resolution": resolution,
            "fov": 60.0,
            "description": description,
        }

    name, preset = best_match
    print(f"✅ Best match: {name} - {preset['description']}")
    return preset


def print_parameters_summary(params: Dict[str, object]) -> None:
    """Pretty-print derived parameters for quick copying."""

    camera_matrix = params["camera_matrix"]
    dist_coeffs = params["dist_coeffs"]
    resolution = params["resolution"]

    print("\n" + "=" * 60)
    print("📷 ESTIMATED CAMERA PARAMETERS")
    print("=" * 60)
    print(f"Description : {params['description']}")
    print(f"Resolution  : {resolution[0]} x {resolution[1]}")
    print(f"Field of View: {params['fov']:.1f}°")

    print("\n🎯 Camera Matrix:")
    print(f"   fx = {camera_matrix[0, 0]:.2f}")
    print(f"   fy = {camera_matrix[1, 1]:.2f}")
    print(f"   cx = {camera_matrix[0, 2]:.2f}")
    print(f"   cy = {camera_matrix[1, 2]:.2f}")

    print("\n🔧 Distortion Coefficients:")
    print(f"   {dist_coeffs.flatten()}")

    matrix_values = camera_matrix.tolist()
    dist_values = dist_coeffs.flatten().tolist()
    print("\n📝 For camera_data_collection.py:")
    print(f"camera_matrix = np.array({matrix_values}, dtype=np.float32)")
    print(f"dist_coeffs = np.array({dist_values}, dtype=np.float32)")
    print("=" * 60)


def update_camera_collection_file(
    params: Dict[str, object],
    target_file: Optional[Path] = None,
) -> bool:
    """Patch camera_data_collection script with the estimated intrinsics."""

    if target_file is None:
        override = os.environ.get("FULLSTACK_CAMERA_COLLECTION_FILE")
        target_file = Path(override) if override else DEFAULT_CAMERA_SCRIPT
    else:
        target_file = Path(target_file)

    if not target_file.exists():
        print(f"❌ Target file not found: {target_file}")
        return False

    try:
        content = target_file.read_text(encoding="utf-8")
    except OSError as exc:  # pragma: no cover - I/O safeguard
        print(f"❌ Error reading {target_file}: {exc}")
        return False

    matrix_values = params["camera_matrix"].tolist()
    dist_values = params["dist_coeffs"].flatten().tolist()
    matrix_line = (
        "camera_matrix = np.array("
        f"{matrix_values}, dtype=np.float32)"
    )
    dist_line = (
        "dist_coeffs = np.array("
        f"{dist_values}, dtype=np.float32)"
    )

    new_content, matrix_subs = MATRIX_PATTERN.subn(
        matrix_line,
        content,
        count=1,
    )
    if matrix_subs == 0:
        new_content = content.replace(
            "camera_matrix = np.array([[800, 0, 320], [0, 800, 240], "
            "[0, 0, 1]], dtype=np.float32)",
            matrix_line,
            1,
        )

    final_content, dist_subs = DIST_PATTERN.subn(
        dist_line,
        new_content,
        count=1,
    )
    if dist_subs == 0:
        final_content = final_content.replace(
            DEFAULT_DIST_SNIPPET,
            dist_line,
            1,
        )

    try:
        target_file.write_text(final_content, encoding="utf-8")
    except OSError as exc:  # pragma: no cover
        print(f"❌ Error writing {target_file}: {exc}")
        return False

    print(f"✅ Updated camera parameters in {target_file}")
    return True


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""

    parser = argparse.ArgumentParser(description="Webcam Parameter Estimator")
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera device ID",
    )
    parser.add_argument(
        "--preset",
        choices=[
            "basic_720p",
            "wide_720p",
            "basic_1080p",
            "wide_1080p",
            "ultrawide",
        ],
        help="Select a preset instead of detection",
    )
    parser.add_argument(
        "--fov",
        type=float,
        help="Manual FOV in degrees",
    )
    parser.add_argument(
        "--resolution",
        type=str,
        help='Manual resolution formatted as "width,height"',
    )
    parser.add_argument(
        "--update-target",
        type=Path,
        default=None,
        help="File to update with the inferred parameters",
    )
    return parser.parse_args()


def main() -> None:  # pragma: no cover - CLI entry point
    args = parse_args()

    if args.preset:
        presets = WebcamParameterEstimator.get_webcam_presets()
        params = presets.get(args.preset)
        if params is None:
            print(f"❌ Unknown preset: {args.preset}")
            return
        print_parameters_summary(params)
        return

    if args.resolution and args.fov is not None:
        try:
            width_str, height_str = args.resolution.split(",")
            resolution = (int(width_str), int(height_str))
        except ValueError as exc:
            raise SystemExit(
                "❌ Invalid resolution format. Use 'width,height'"
            ) from exc

        camera_matrix, dist_coeffs = (
            WebcamParameterEstimator.get_typical_parameters(
                resolution,
                args.fov,
            )
        )
        description = (
            f"Manual parameters: {resolution[0]}x{resolution[1]}"
            f" @ {args.fov:.1f}°"
        )
        params = {
            "camera_matrix": camera_matrix,
            "dist_coeffs": dist_coeffs,
            "resolution": resolution,
            "fov": args.fov,
            "description": description,
        }
        print_parameters_summary(params)
        return

    params = auto_detect_camera_parameters(args.camera)
    if params is None:
        return

    print_parameters_summary(params)

    save = input(
        "\n💾 Update camera_data_collection.py with these values? (y/n): "
    )
    if save.lower() == "y":
        target = args.update_target or DEFAULT_CAMERA_SCRIPT
        update_camera_collection_file(params, target)


if __name__ == "__main__":  # pragma: no cover
    main()
