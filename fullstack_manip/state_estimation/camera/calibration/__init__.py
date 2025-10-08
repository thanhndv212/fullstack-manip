"""Camera calibration utilities for the fullstack manipulation project.

This package consolidates calibration scripts, marker generation helpers, and
camera test tools so that they can be shared across the project stack.  The
layout mirrors the original FIGAROH examples while adopting project-friendly
paths and configuration helpers.
"""

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
TOOLS_DIR = PACKAGE_ROOT / "tools"
SCRIPTS_DIR = PACKAGE_ROOT / "scripts"
PATTERNS_DIR = PACKAGE_ROOT / "patterns"
DATA_DIR = PACKAGE_ROOT / "data"

__all__ = [
    "CameraCalibrator",
    "WebcamParameterEstimator",
    "generate_aruco_marker",
    "generate_marker_set",
    "generate_marker_sheet",
    "PACKAGE_ROOT",
    "TOOLS_DIR",
    "SCRIPTS_DIR",
    "PATTERNS_DIR",
    "DATA_DIR",
    "run_detection_monitor",
]


def run_detection_monitor() -> None:
    """Launch the ArUco detection monitor tool."""

    from .tools.detection_monitor import main

    main()


def ensure_data_dir(exists: bool = True) -> Path:
    """Ensure the calibration data directory exists and return its path."""

    if exists:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
    return DATA_DIR


def ensure_patterns_dir(exists: bool = True) -> Path:
    """Ensure the calibration patterns directory exists and return its path."""

    if exists:
        PATTERNS_DIR.mkdir(parents=True, exist_ok=True)
    return PATTERNS_DIR


def __getattr__(name: str):  # pragma: no cover - lightweight proxies
    if name == "CameraCalibrator":
        from .tools.camera_calibration_tool import CameraCalibrator

        return CameraCalibrator
    if name == "WebcamParameterEstimator":
        from .tools.webcam_parameter_estimator import (
            WebcamParameterEstimator,
        )

        return WebcamParameterEstimator
    if name in {
        "generate_aruco_marker",
        "generate_marker_set",
        "generate_marker_sheet",
    }:
        from .tools import generate_aruco_markers as marker_module

        return getattr(marker_module, name)
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
