# Fullstack Manipulation Codebase Summary

This document provides a quick reference for automated code-generation agents interacting with the repository. It captures the project architecture, key modules, and extension tips so agents can locate the right entry points without re-discovering the structure.

## High-level overview

- **Purpose:** Research sandbox for robot manipulation combining MuJoCo simulation, motion planning, and learning experiments.
- **Primary language:** Python 3.10+.
- **Simulation core:** MuJoCo runtime plus DeepMind’s `mujoco_menagerie` assets.
- **Motion generation:** Uses the `mink` library for inverse kinematics (IK) and constrained trajectory generation.
- **Visualization:** Thin wrapper around `mujoco.viewer` for interactive and scripted rendering.
- **Documentation:** Rich design docs live in `docs/` and parallel the planned stack (state estimation, planning, learning, evaluation, etc.).

## Repository layout

| Path | Purpose |
| --- | --- |
| `docs/` | Project documentation (requirements, architecture, subsystem plans). |
| `fullstack_manip/control/` | Placeholder for high-level and low-level controllers (currently minimal). |
| `fullstack_manip/evaluation/` | Stub for metrics/benchmarking utilities. |
| `fullstack_manip/hardware/` | CAD, system identification results, URDFs, and real-world calibration assets. |
| `fullstack_manip/learning/` | Datasets, reinforcement learning, and VLA experiment scaffolding. |
| `fullstack_manip/perception/` | Future perception stack modules (empty scaffolding today). |
| `fullstack_manip/planning/` | Planning integrations (e.g., MoveIt bridge, planner prototypes). |
| `fullstack_manip/scripts/` | Main runnable modules for MuJoCo simulation, robot abstraction, asset loading, teleop, etc. |
| `fullstack_manip/simulation/` | Scenario definitions, sim-to-real experiments, environment assets. |
| `fullstack_manip/state_estimation/` | Camera, IMU, mocap fusion placeholders. |
| `tests/` | Reserved for automated tests (currently empty). |

> Many subdirectories are scaffolds; when adding functionality, prefer extending the existing structure rather than creating new top-level folders.

## Core runtime flow (simulation & control)

1. **Asset staging** – `scripts/assets.py` ensures the `mujoco_menagerie` repo is available, retrieves robot/hand/environment XMLs, and exposes helper utilities for bundling assets.
2. **Scene loading** – `scripts/loader.py` (`MuJoCoSceneLoader`) can load models from `robot_descriptions`, raw XML strings, or file paths. It is designed to keep asset resolution transparent to callers.
3. **Scene management** – `scripts/scene.py` (`MuJoCoSceneManager`) wraps MuJoCo model/data creation, state resets, joint getters/setters, and simple kinematic queries. It normalizes scene initialization by resetting to the `home` keyframe.
4. **Robot abstraction** – `scripts/robot.py` (`Robot` class) composes:
   - `MotionPlanner` (`scripts/motion_planner.py`) for IK and spline-based trajectory planning through `mink`.
   - `PIDController` (`scripts/pid_controller.py`) for tracking planned joint trajectories.
   - `MuJoCoViewer` (`scripts/viewer.py`) for visualization/recording.
   - Utility methods for contact checks, grasp heuristics, and workspace estimation (`compute_workspace`).
5. **Entry points** – `scripts/sim_mujoco.py` demonstrates the full pipeline: load scene assets, instantiate `Robot`, sample workspace, and visualize results.

## Key modules and responsibilities

- **`Robot` (`robot.py`)**
  - High-level API for moving the simulated manipulator, computing reachable workspaces, checking grasps, and interfacing with control/planning utilities.
  - Maintains MuJoCo data synchronization (`mj_forward`, `mj_step`) and viewer updates.
  - Recent refinements include an adaptive workspace sampler that reuses successful IK seeds and filters duplicates.

- **`MotionPlanner` (`motion_planner.py`)**
  - Configures `mink.Configuration` with collision avoidance limits, velocity bounds, and IK convergence settings.
  - Provides `plan_trajectory()` (CubicSpline interpolation between IK solutions) and `solve_ik_for_pose()` (iterative solver under constraints).

- **`MuJoCoViewer` (`viewer.py`)**
  - Launches passive viewers, synchronizes frames, and supports video recording (set via `start_recording`/`stop_recording`).

- **`PIDController` (`pid_controller.py`)**
  - Simple PID implementation for joint-space control; tuned defaults assume 0.01 s timestep.

- **`teleop_soarm100.py`, `pick_place_controller.py`, etc.**
  - Scenario-specific scripts that orchestrate robot actions; most still reference future hardware integrations.

## External dependencies

- **MuJoCo** (`mujoco` Python bindings) – core physics engine.
- **mink** – Differentiable kinematics/trajectory optimizer used for IK.
- **etils.epath** – Filesystem helper compatible with Bazel-like resource paths.
- **numpy`, `scipy`** – Numerical computation and spline interpolation.
- **tqdm** – Progress feedback during repository cloning.

The repository expects MuJoCo, `mink`, and the DeepMind `mujoco_menagerie` assets to be installed or cloneable. `scripts/assets.py` manages cloning when absent.

## Data & asset conventions

- Robot/gripper/environment XMLs live in `fullstack_manip/scripts/object`, `realworld`, and `trs_so_arm100`, which mirror menagerie layouts.
- Scene loader defaults to the `home` keyframe to ensure deterministic starting configurations.
- Workspace points and additional geoms added during visualization should keep sizes small (≤2 mm radius) to avoid cluttering MuJoCo viewers.

## Testing & validation

- Continuous integration scaffolding is not yet defined. Use ad-hoc scripts (`sim_mujoco.py`) or `python -m compileall` for quick sanity checks.
- Empty `tests/` directory signals an opportunity to add unit/regression tests; prefer `pytest` for new coverage.

## Extension guidelines for agents

- **Respect module boundaries:** Extend planners inside `fullstack_manip/planning`, controllers inside `fullstack_manip/control`, etc.
- **Re-use helpers:** Stick to `MuJoCoSceneLoader`, `MuJoCoSceneManager`, and `Robot` abstractions for new scripts rather than re-instantiating MuJoCo manually.
- **Keep viewers optional:** Scripts should run headless when viewers are unavailable; gate visualization behind configuration flags.
- **Document new workflows:** For substantial features, add short design notes under `docs/` and update this summary if the architecture changes.
- **Asset management:** When adding new robots/environments, update `scripts/assets.py` or reference menagerie directories consistently so automated fetches continue to work.

## Current gaps & potential follow-ups

- Many subsystem folders are placeholders. Populate them with clear interfaces before deeply coupling new features.
- Collision checking in `MotionPlanner` is limited; consider integrating richer environment awareness when extending planning capabilities.
- Workspace visualization currently relies on side effects in MuJoCo viewer; consider exporting point clouds for offline analysis if needed.

This summary should be refreshed as modules mature so downstream automation stays aligned with the project’s structure.
