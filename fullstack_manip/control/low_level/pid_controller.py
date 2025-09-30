from __future__ import annotations

from typing import Optional, Union

import numpy as np


class PIDController:
    """Basic PID controller with optional integral and output clamping."""

    def __init__(
        self,
        kp: Union[float, np.ndarray],
        ki: Union[float, np.ndarray],
        kd: Union[float, np.ndarray],
        dt: float = 0.01,
        integral_limit: Optional[float] = None,
        output_limit: Optional[float] = None,
        signal_dim: int = 6,
    ) -> None:
        self.kp = np.array(kp)
        self.ki = np.array(ki)
        self.kd = np.array(kd)
        self.dt = float(dt)
        self.integral_limit = integral_limit
        self.output_limit = output_limit

        self.integral = np.zeros(signal_dim)
        self.previous_error = np.zeros(signal_dim)
        self.previous_time = 0.0

    def compute(
        self,
        target: Union[float, np.ndarray],
        current: Union[float, np.ndarray],
        timestamp: Optional[float] = None,
    ) -> np.ndarray:
        """Compute the control effort for the given error."""
        error = np.array(target) - np.array(current)

        proportional = self.kp * error

        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = np.clip(
                self.integral,
                -self.integral_limit,
                self.integral_limit,
            )
        integral = self.ki * self.integral

        if timestamp is not None and timestamp != self.previous_time:
            dt = timestamp - self.previous_time
            derivative = self.kd * (error - self.previous_error) / dt
        else:
            derivative = self.kd * (error - self.previous_error) / self.dt

        damping = 0.1 * (error - self.previous_error) / self.dt
        self.previous_error = error
        if timestamp is not None:
            self.previous_time = timestamp

        output = proportional + integral + derivative - damping
        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)

        return output

    def reset(self) -> None:
        """Reset the integrator and previous error state."""
        self.integral = np.zeros_like(self.integral)
        self.previous_error = np.zeros_like(self.previous_error)
        self.previous_time = 0.0

    def set_gains(
        self,
        kp: Optional[Union[float, np.ndarray]] = None,
        ki: Optional[Union[float, np.ndarray]] = None,
        kd: Optional[Union[float, np.ndarray]] = None,
    ) -> None:
        """Update controller gains."""
        if kp is not None:
            self.kp = np.array(kp)
        if ki is not None:
            self.ki = np.array(ki)
        if kd is not None:
            self.kd = np.array(kd)


__all__ = ["PIDController"]
