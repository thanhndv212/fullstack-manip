"""Camera utilities for state estimation."""

__all__ = ["calibration"]


def __getattr__(name: str):  # pragma: no cover - thin module wrapper
    if name == "calibration":
        from . import calibration as calibration_module

        return calibration_module
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
