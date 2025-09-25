
# Fullstack Manipulation Project

>A modular, extensible framework for research and development in robotic manipulation, supporting both model-based and learning-based approaches. Initially focused on the UR10 manipulator, but designed for generalization to other fixed-base robots.

---

## Project Goals
- Provide a unified stack for manipulation research (hardware, simulation, control, planning, learning)
- Support both model-based and learning-based pipelines
- Enable sim-to-real transfer and benchmarking
- Modular design for easy extension to new robots and tasks

## Architecture Overview

```
fullstack-manip/
├── hardware/           # Robot models, CAD, system ID
├── state_estimation/   # Multi-sensor fusion, calibration
├── simulation/         # MuJoCo and other simulators
├── control/            # Low- and high-level controllers
├── planning/           # Motion planning, trajectory gen
├── perception/         # Visual servoing, vision modules
├── learning/           # RL, VLA, datasets
├── evaluation/         # Benchmarking, metrics
├── scripts/            # Utilities, launchers
├── tests/              # Unit/integration tests
├── docs/               # Documentation, diagrams
```

## Key Features
- UR10 support (URDF, sysID, comms)
- Multi-sensor state estimation (camera, mocap, IMU)
- MuJoCo-based simulation with sim2real tools
- Model-based stack: planning, visual servoing, MPC
- Learning-based stack: RL, VLA (Open PI, LeRobot)
- Modular, extensible, research-friendly

## Getting Started
1. Clone the repository
2. Install dependencies: `pip install -r requirements.txt` or use `environment.yml`
3. Explore example scripts in `scripts/`
4. See `docs/` for architecture and usage guides

## Folder Guide
- `hardware/` — URDF, CAD, sysID data
- `state_estimation/` — Sensor fusion, calibration
- `simulation/` — MuJoCo envs, assets, sim2real
- `control/` — Low/high-level controllers
- `planning/` — MoveIt, planners
- `perception/` — Vision, visual servoing
- `learning/` — RL, VLA, datasets
- `evaluation/` — Metrics, benchmarking
- `scripts/` — Launchers, utilities
- `tests/` — Testing
- `docs/` — Documentation

## Contributing
Contributions are welcome! Please open issues or pull requests for new features, bug fixes, or documentation improvements.

## License
MIT License (see LICENSE file)
