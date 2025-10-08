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
| `fullstack_manip/control/` | High-level controllers (base class + 8 specialized controllers) and low-level (`pid_controller.py`) control layers. |
| `fullstack_manip/evaluation/` | Stub for metrics/benchmarking utilities. |
| `fullstack_manip/hardware/` | CAD, system identification results, URDFs, and real-world calibration assets. |
| `fullstack_manip/learning/` | Datasets, reinforcement learning, and VLA experiment scaffolding. |
| `fullstack_manip/perception/` | Future perception stack modules (empty scaffolding today). |
| `fullstack_manip/planning/` | Motion planning utilities (e.g., Mink-based `motion_planner.py`). |
| `fullstack_manip/scripts/` | Thin entry-point scripts (`sim_mujoco.py`, `picknplace_soarm100.py`, `teleop_soarm100.py`). |
| `fullstack_manip/simulation/` | Asset manager, scene loader, viewer wrappers, and sim-to-real resources. |
| `fullstack_manip/utils/` | Shared helpers such as the `RateLimiter` loop utilities. |
| `fullstack_manip/state_estimation/` | Camera, IMU, mocap fusion placeholders. |
| `tests/` | Reserved for automated tests (currently empty). |

> Many subdirectories are scaffolds; when adding functionality, prefer extending the existing structure rather than creating new top-level folders.

## Core runtime flow (simulation & control)

1. **Asset staging** – `fullstack_manip/simulation/asset_manager.py` ensures the `mujoco_menagerie` repo is available, retrieves robot/hand/environment XMLs, and exposes helper utilities for bundling assets.
2. **Scene loading** – `fullstack_manip/simulation/loader.py` (`MuJoCoSceneLoader`) can load models from menagerie directories, raw XML strings, or file paths. It keeps asset resolution transparent to callers.
3. **Scene management** – `fullstack_manip/simulation/scene.py` (`MuJoCoSceneManager`) wraps MuJoCo model/data creation, state resets, joint getters/setters, and kinematic queries, resetting to the `home` keyframe.
4. **Robot abstraction** – `fullstack_manip/control/high_level/robot.py` (`Robot` class) composes:
  - `MotionPlanner` (`fullstack_manip/planning/motion_planner.py`) for Mink-powered IK and spline-based trajectory planning.
  - `PIDController` (`fullstack_manip/control/low_level/pid_controller.py`) for tracking planned joint trajectories.
  - `MuJoCoViewer` (`fullstack_manip/simulation/viewer.py`) for visualization/recording.
  - Utility methods for contact checks, grasp heuristics, workspace estimation (`compute_workspace`), and loop pacing via `RateLimiter`.
5. **High-level controllers** – `fullstack_manip/control/high_level/` provides a family of specialized controllers inheriting from `BaseController`:
  - `PickPlaceController` – Autonomous pick-and-place routines with grasp retry logic.
  - `TrajectoryFollowingController` – Execute pre-planned paths with configurable interpolation.
  - `ForceControlController` – Contact-rich manipulation leveraging MuJoCo contact dynamics.
  - `VisualServoController` – Image-based servoing with camera calibration integration.
  - `ImpedanceController` – Variable stiffness for safe human-robot interaction.
  - `AdmittanceController` – Force-to-motion mapping for manual guidance and teaching.
  - `GravityCompensationController` – Nullify gravity for kinesthetic teaching and manual positioning.
6. **Entry points** – `fullstack_manip/scripts/sim_mujoco.py` (and related teleop/pick-place scripts) demonstrate the full pipeline by importing the packaged modules.

## High-level controller architecture

The `control/high_level/` module implements an extensible controller framework built around object-oriented polymorphism:

**Design Pattern:**
- All controllers inherit from `BaseController` (abstract base class using Python's ABC).
- Common interface: `execute(*args, **kwargs)` method must be implemented by each controller.
- Shared utilities provided by base class: reach checking, pose queries, state reporting.
- Each controller encapsulates domain-specific parameters and behavior.

**Controller Categories:**
1. **Task Controllers:** `PickPlaceController` – High-level task execution with primitive sequencing.
2. **Motion Controllers:** `TrajectoryFollowingController` – Path execution and trajectory tracking.
3. **Force Controllers:** `ForceControlController` – Contact-rich manipulation with force feedback.
4. **Visual Controllers:** `VisualServoController` – Image-based alignment and visual feature tracking.
5. **Compliance Controllers:** `ImpedanceController`, `AdmittanceController` – Safe interaction with variable mechanical properties.
6. **Teaching Controllers:** `GravityCompensationController` – Manual guidance and kinesthetic demonstration.

**Integration Points:**
- All controllers operate on the same `Robot` instance, accessing its `MotionPlanner`, `PIDController`, and MuJoCo `model`/`data`.
- Controllers can be composed: e.g., use `GravityCompensationController` to record a trajectory, then replay with `TrajectoryFollowingController`.
- Force-based controllers (`ForceControlController`, `AdmittanceController`, `GravityCompensationController`) leverage MuJoCo's contact dynamics and `qfrc_bias` for physics-accurate interaction.

## Key modules and responsibilities

- **`BaseController` (`control/high_level/base_controller.py`)**
  - Abstract base class defining the common interface for all high-level controllers.
  - Provides shared utilities: `is_within_reach()`, `get_current_end_effector_pose()`, `move_to_pose()`, `get_controller_state()`.
  - Enforces implementation of abstract `execute()` method in subclasses.
  - Configurable reach threshold for task-specific tolerance.

- **`Robot` (`control/high_level/robot.py`)**
  - High-level API for moving the simulated manipulator, computing reachable workspaces, checking grasps, and interfacing with control/planning utilities.
  - Maintains MuJoCo data synchronization (`mj_forward`, `mj_step`) and viewer updates.
  - Recent refinements include an adaptive workspace sampler that reuses successful IK seeds and filters duplicates.

- **Specialized Controllers (`control/high_level/*_controller.py`)**
  - **`PickPlaceController`** – Autonomous pick-and-place with grasp success detection and retry logic; integrates with gripper joint control.
  - **`TrajectoryFollowingController`** – Executes waypoint sequences with linear/cubic interpolation; supports both joint and Cartesian space trajectories.
  - **`ForceControlController`** – Performs contact-rich tasks with force monitoring; includes compliant motion and constant force application.
  - **`VisualServoController`** – Image-based (IBVS) and position-based (PBVS) visual servoing; ready for ArUco marker detection integration with camera calibration module.
  - **`ImpedanceController`** – Implements mechanical impedance (mass-spring-damper) for compliant interaction; adjustable stiffness/damping/inertia matrices.
  - **`AdmittanceController`** – Maps external forces to desired motion; supports kinesthetic teaching, backdriving, and human guidance.
  - **`GravityCompensationController`** – Applies joint torques to cancel gravity using MuJoCo's `qfrc_bias`; enables manual positioning and teaching.

- **`MotionPlanner` (`planning/motion_planner.py`)**
  - Configures `mink.Configuration` with collision avoidance limits, velocity bounds, and IK convergence settings.
  - Provides `plan_trajectory()` (CubicSpline interpolation between IK solutions) and `solve_ik_for_qpos()` (iterative solver under constraints).

- **`MuJoCoViewer` (`simulation/viewer.py`)**
  - Launches passive viewers, synchronizes frames, and supports video recording (set via `start_recording`/`stop_recording`).

- **`PIDController` (`control/low_level/pid_controller.py`)**
  - Simple PID implementation for joint-space control; tuned defaults assume 0.01 s timestep.

- **`RateLimiter` (`utils/loop_rate_limiters.py`)**
  - Lightweight loop timing utility that maintains fixed control frequencies and warns on overruns when desired.

- **Scripts (`scripts/sim_mujoco.py`, `scripts/teleop_soarm100.py`, etc.)**
  - Scenario-specific entry points that orchestrate robot actions by importing the packaged control/planning/simulation modules.

## External dependencies

- **MuJoCo** (`mujoco` Python bindings) – core physics engine; contact dynamics used for force control.
- **mink** – Differentiable kinematics/trajectory optimizer used for IK.
- **etils.epath** – Filesystem helper compatible with Bazel-like resource paths.
- **numpy**, **scipy** – Numerical computation and spline interpolation.
- **tqdm** – Progress feedback during repository cloning.
- **Python standard library** – `abc` for controller base class, `typing` for type hints.

The repository expects MuJoCo, `mink`, and the DeepMind `mujoco_menagerie` assets to be installed or cloneable. `fullstack_manip/simulation/asset_manager.py` manages cloning when absent.

## Data & asset conventions

- Robot/gripper/environment XMLs live in `fullstack_manip/scripts/object`, `realworld`, and `trs_so_arm100`, which mirror menagerie layouts.
- Scene loader defaults to the `home` keyframe to ensure deterministic starting configurations.
- Workspace points and additional geoms added during visualization should keep sizes small (≤2 mm radius) to avoid cluttering MuJoCo viewers.

## Testing & validation

- Continuous integration scaffolding is not yet defined. Use ad-hoc scripts (`sim_mujoco.py`) or `python -m compileall` for quick sanity checks.
- Empty `tests/` directory signals an opportunity to add unit/regression tests; prefer `pytest` for new coverage.

## Extension guidelines for agents

- **Respect module boundaries:** Extend planners inside `fullstack_manip/planning`, controllers inside `fullstack_manip/control`, utilities under `fullstack_manip/utils`, etc.
- **Controller extensions:** New high-level controllers should inherit from `BaseController` and implement the abstract `execute()` method. Follow the established pattern of configurable parameters in `__init__` and specialized behavior methods.
- **Re-use helpers:** Stick to `MuJoCoSceneLoader`, `MuJoCoSceneManager`, `Robot`, and `RateLimiter` abstractions for new scripts rather than re-instantiating MuJoCo manually.
- **Keep viewers optional:** Scripts should run headless when viewers are unavailable; gate visualization behind configuration flags.
- **Document new workflows:** For substantial features, add short design notes under `docs/` and update this summary if the architecture changes.
- **Asset management:** When adding new robots/environments, update `fullstack_manip/simulation/asset_manager.py` or reference menagerie directories consistently so automated fetches continue to work.
- **Controller composition:** High-level controllers can be composed or switched dynamically based on task requirements; prefer clean handoff protocols between control modes.

## Current gaps & potential follow-ups

- **State estimation integration:** The `VisualServoController` has placeholders for camera calibration integration; connect with `fullstack_manip/state_estimation/camera` module when perception stack matures.
- **Force/torque sensing:** `ForceControlController`, `AdmittanceController`, and `GravityCompensationController` currently use simplified force estimation; integrate with hardware F/T sensors or MuJoCo sensor elements for real deployments.
- **Controller validation:** High-level controllers need unit tests and scenario-based integration tests; populate `tests/` with coverage for each controller's core behaviors.
- **Hybrid control:** Consider implementing mode-switching logic or hierarchical control architectures to seamlessly transition between controllers (e.g., force control → impedance control → pick-place).
- Many subsystem folders remain placeholders. Populate them with clear interfaces before deeply coupling new features.
- Collision checking in `MotionPlanner` is limited; consider integrating richer environment awareness when extending planning capabilities.
- Workspace visualization currently relies on side effects in MuJoCo viewer; consider exporting point clouds for offline analysis if needed.
- **Learning integration:** Controllers like `AdmittanceController` (kinesthetic teaching) and `TrajectoryFollowingController` (demonstration replay) provide natural interfaces for learning from demonstration (LfD) pipelines.

This summary should be refreshed as modules mature so downstream automation stays aligned with the project’s structure.
