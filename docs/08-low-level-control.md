# Low-Level Controller

- Real-time communication with robot (e.g., ROS2, RTDE).
- Joint-level PID/impedance control.
- Safety checks and watchdogs.

## Timing configuration

- Loop rates are sourced from a shared `RateConfig` utility that loads
	defaults from `configs/rates.yaml` (or environment overrides).
- Core components (`Robot`, `MotionExecutor`, planners, and high-level
	controllers) receive the same configuration to avoid mismatched
	assumptions about `dt`.
- Use `load_rate_config()` to customise simulation, control, planner,
	estimator, or sensor frequencies for specific deployments.
