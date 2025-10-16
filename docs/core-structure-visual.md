# Core Package Structure - Visual Overview

**Last Updated**: October 16, 2025  
**Status**: ✅ Current Production Structure

---

## 📦 Complete Package Structure

```
fullstack_manip/
├── core/                              ✨ MODULAR ARCHITECTURE
│   ├── __init__.py                    # Exports all core components
│   ├── robot.py                       # Robot instance & control
│   ├── gripper.py                     # Gripper control with state machine
│   ├── collision.py                   # Collision detection & contact forces
│   ├── ik.py                          # Inverse kinematics solver
│   ├── limit.py                       # Joint/workspace limit management
│   ├── state.py                       # State management (StateManager)
│   ├── objects.py                     # Object tracking (ObjectManager)
│   ├── manip_plant.py                 # Main orchestrator (ManipulationPlant)
│   ├── config.py                      # Configuration system (YAML/JSON)
│   ├── visualization.py               # Architecture diagram generation
│   └── interfaces.py                  # Protocol interfaces for extensibility
│
├── control/
│   ├── motion_executor.py            # Motion execution helper
│   ├── high_level/
│   │   ├── gravity_compensation_controller.py
│   │   ├── impedance_controller.py
│   │   └── trajectory_following_controller.py
│   └── low_level/
│       └── pid_controller.py
│
├── planning/
│   ├── __init__.py
│   ├── motion_planner.py              # Trajectory planning
│   ├── moveit/                        # MoveIt! integration
│   └── planners/                      # Custom planners
│
├── execution/
│   ├── behaviors/                     # Behavior tree nodes
│   │   ├── control_behaviors.py
│   │   ├── gripper_behaviors.py
│   │   ├── motion_behaviors.py
│   │   ├── perception_behaviors.py
│   │   └── safety_behaviors.py
│   ├── blackboard/                    # Shared data storage
│   └── skills/                        # High-level skills
│       └── pick_skill.py
│
├── simulation/
│   ├── __init__.py
│   ├── loader.py                      # URDF/MJCF loading
│   ├── scene.py                       # Scene management
│   ├── viewer.py                      # Visualization
│   ├── asset_manager.py               # Asset handling
│   └── assets/                        # 3D models, URDFs
│
├── state_estimation/
│   ├── camera/
│   │   └── calibration/               # Camera calibration tools
│   ├── fusion/                        # Sensor fusion
│   ├── imu/                          # IMU processing
│   └── mocap/                        # Motion capture
│
└── utils/
    └── loop_rate_limiters.py          # Timing utilities
```

## 🏗️ Modular Architecture Overview

### Core Design Philosophy

```
┌─────────────────────────────────────────────────────────────────┐
│                    ManipulationPlant                             │
│                  (Main Orchestrator)                             │
│                                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │  Robot       │  │  Gripper     │  │ StateManager │          │
│  │              │  │              │  │              │          │
│  │ • Control    │  │ • Grasp      │  │ • Robot      │          │
│  │ • Kinematics │  │ • Release    │  │ • Objects    │          │
│  │ • Motion     │  │ • Force      │  │ • Task       │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
│                                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ObjectManager │  │ IKSolver     │  │ Collision    │          │
│  │              │  │              │  │ Checker      │          │
│  │ • Track      │  │ • Solve      │  │ • Detect     │          │
│  │ • Update     │  │ • Limits     │  │ • Forces     │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└─────────────────────────────────────────────────────────────────┘
                              ↑
                              │
              ┌───────────────┼───────────────┐
              │               │               │
    ┌─────────▼────────┐  ┌──▼────────┐  ┌──▼─────────┐
    │ Configuration    │  │ Planning  │  │ Execution  │
    │ • YAML/JSON      │  │ • Motion  │  │ • Skills   │
    │ • Factory        │  │ • IK      │  │ • Behaviors│
    └──────────────────┘  └───────────┘  └────────────┘
```

### Component Interaction Flow

```
Application Layer
      │
      ├─→ Load Config (config.py)
      │        │
      │        ├─→ Create Components (ComponentFactory)
      │        │
      │        └─→ Build Plant (ManipulationPlant)
      │
      ├─→ Plan Motion (planning/)
      │        │
      │        └─→ Use IK/Collision (core/)
      │
      ├─→ Execute Skills (execution/)
      │        │
      │        └─→ Control Plant (core/)
      │
      └─→ Visualize (visualization.py)
               │
               └─→ Generate Diagrams
```

## 📊 Core Module Responsibilities

| Module | Responsibility | Key Classes/Methods |
|--------|---------------|---------------------|
| **manip_plant.py** | Main orchestrator | `ManipulationPlant`, `ManipulationPlantBuilder` |
| **robot.py** | Robot control & kinematics | `Robot`, `get_body_pose()`, `move_to_position()` |
| **gripper.py** | Gripper control & state | `Gripper`, `grasp()`, `release()`, `get_state()` |
| **state.py** | State management | `StateManager`, `RobotState`, `ObjectState`, `TaskState` |
| **objects.py** | Object tracking | `ObjectManager`, `ManipulationObject` |
| **collision.py** | Collision detection | `CollisionChecker`, `detect_contact()`, `compute_contact_force()` |
| **ik.py** | Inverse kinematics | `IKSolver`, `solve_ik_for_qpos()` |
| **limit.py** | Limit management | `LimitManager`, `set_limits()` |
| **config.py** | Configuration system | `PlantConfig`, `ComponentFactory`, `create_plant_from_config()` |
| **visualization.py** | Diagram generation | `PlantVisualizer`, `ConfigVisualizer`, `generate_diagrams()` |
| **interfaces.py** | Protocol definitions | `RobotProtocol`, `GripperProtocol`, etc. |

### Control Utilities

- **control/motion_executor.py** – Bridges motion planning with low-level PID
    execution for all high-level controllers, replacing the older
    `Robot.move_to_position` helper.

## 🔗 Module Dependencies

```
┌──────────────────────────────────────────────────────────┐
│                  Application Layer                        │
│  • examples/pickplace_with_new_architecture.py           │
│  • scripts/picknplace_soarm100.py                        │
│  • scripts/sim_mujoco.py                                 │
└───────────────────────┬──────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ▼               ▼               ▼
┌───────────────┐  ┌──────────┐  ┌─────────────┐
│ execution/    │  │planning/ │  │ control/    │
│ • skills      │  │• motion  │  │• high_level │
│ • behaviors   │  │  planner │  │• low_level  │
└───────┬───────┘  └────┬─────┘  └──────┬──────┘
        │               │                │
        └───────────────┼────────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │     fullstack_manip.core      │
        │                               │
        │  • ManipulationPlant          │
        │  • Robot, Gripper             │
        │  • StateManager               │
        │  • ObjectManager              │
        │  • IKSolver, CollisionChecker │
        │  • Configuration              │
        │  • Visualization              │
        └───────────────┬───────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │    simulation/ (optional)     │
        │  • loader, scene, viewer      │
        └───────────────────────────────┘

Legend:
  ─────►  Direct dependency
  ◄─────  Provides interface/data
```

## ✅ Package Features

### Core Components
- [x] **ManipulationPlant** - Main orchestrator with builder pattern
- [x] **Robot** - Robot instance, control, kinematics
- [x] **Gripper** - State machine-based gripper control
- [x] **StateManager** - Centralized state management
- [x] **ObjectManager** - Object tracking and properties
- [x] **CollisionChecker** - Collision detection & contact forces
- [x] **IKSolver** - Inverse kinematics solver
- [x] **LimitManager** - Joint/workspace limits

### Configuration System
- [x] **PlantConfig** - YAML/JSON configuration support
- [x] **ComponentFactory** - Dynamic component creation
- [x] **Schema validation** - Automatic config validation
- [x] **create_plant_from_config()** - One-line plant creation

### Visualization
- [x] **PlantVisualizer** - Architecture diagrams
- [x] **ConfigVisualizer** - Configuration diagrams
- [x] **Component diagrams** - Module structure
- [x] **Dataflow diagrams** - Information flow
- [x] **State diagrams** - State machines

### Integration
- [x] **Protocol interfaces** - Extensibility via protocols
- [x] **Behavior tree support** - execution/behaviors/
- [x] **Skills framework** - execution/skills/
- [x] **Motion planning** - planning/motion_planner.py
- [x] **Motion execution** - control/motion_executor.py
- [x] **Controllers** - control/high_level/
- [x] **Simulation** - simulation/ package

## 📝 Quick Usage Reference

### 1. Using ManipulationPlant (Recommended)

```python
from fullstack_manip.core import ManipulationPlant

# Create plant with builder pattern
plant = (
    ManipulationPlant.builder()
    .with_robot(robot)
    .with_gripper(gripper)
    .with_state_manager()
    .with_object_manager()
    .build()
)

# Or load from config
from fullstack_manip.core import create_plant_from_config
plant = create_plant_from_config("configs/pickplace.yaml")

# Use the plant
plant.initialize()
plant.plan_grasp(obj_name="cube")
plant.execute_grasp()
```

### 2. Using Individual Components

```python
from fullstack_manip.core import (
    Robot,
    Gripper,
    StateManager,
    ObjectManager,
    CollisionChecker,
    IKSolver,
    LimitManager,
)

# Robot control
robot = Robot(model, data)
pose = robot.get_body_pose("end_effector")
robot.move_to_position(target_pos)

# Gripper control
gripper = Gripper(model, data, config)
gripper.open()
gripper.grasp("cube", force=10.0)
state = gripper.get_state()

# State management
state_mgr = StateManager()
state_mgr.update_robot_state(position, velocity)
robot_state = state_mgr.get_robot_state()

# Object tracking
obj_mgr = ObjectManager()
obj_mgr.add_object("cube", position, "cube")
obj = obj_mgr.get_object("cube")

# Collision detection
collision = CollisionChecker(model, data)
in_contact = collision.detect_contact("gripper", "cube")
force = collision.compute_contact_force("gripper", "cube")

# IK solving
ik_solver = IKSolver(model, data, limits)
joint_pos = ik_solver.solve_ik_for_qpos("ee_site", target_pos)

# Limit management
limit_mgr = LimitManager(model, gripper_bodies, obstacles)
limits = limit_mgr.set_limits()
```

### 3. Configuration System

```python
from fullstack_manip.core import PlantConfig

# Load from YAML
config = PlantConfig.from_yaml("configs/pickplace.yaml")

# Load from JSON
config = PlantConfig.from_json("configs/assembly.json")

# Access config data
print(config.robot_config)
print(config.gripper_config)
print(config.components)

# Validate config
config.validate()
```

### 4. Visualization

```python
from fullstack_manip.core import visualize_plant, visualize_config

# Visualize plant architecture
visualize_plant(plant, output_dir="diagrams/")

# Visualize configuration
visualize_config("configs/pickplace.yaml", output_dir="diagrams/")

# Generate custom diagrams
from fullstack_manip.core import PlantVisualizer
viz = PlantVisualizer(plant)
viz.generate_component_diagram("diagrams/components.png")
viz.generate_dataflow_diagram("diagrams/dataflow.png")
```

### 5. Complete Example

```python
# Complete pick-and-place workflow
from fullstack_manip.core import create_plant_from_config

# 1. Load configuration
plant = create_plant_from_config("configs/pickplace.yaml")

# 2. Initialize
plant.initialize()

# 3. Plan grasp
plant.plan_grasp(obj_name="cube", approach_height=0.1)

# 4. Execute grasp
success = plant.execute_grasp()

# 5. Plan place
plant.plan_place(target_pos=[0.5, 0.0, 0.2])

# 6. Execute place
plant.execute_place()

# 7. Get state
state = plant.get_state()
print(f"Task complete: {state.task_complete}")
```

---

## 📚 Additional Resources

- **Full Documentation**: See `docs/modular-architecture-readme.md`
- **Configuration Guide**: See `docs/configuration-system.md`
- **Visualization Guide**: See `docs/visualization-system.md`
- **Migration Guide**: See `docs/migration-guide.md`
- **Gripper API**: See `docs/gripper.md`
- **Examples**: See `examples/` directory
- **Quick Start**: See `docs/QUICK_REFERENCE.md`

---

**Status**: ✅ Production Ready  
**Last Updated**: October 16, 2025  
**Package Version**: See `pyproject.toml`
