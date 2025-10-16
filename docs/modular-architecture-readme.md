# Modular Manipulation Architecture

A composable, plugin-based architecture for building manipulation systems.

## 🎯 Core Philosophy

Build any manipulation system by composing modular components:
- **Robot**: Core kinematics and control
- **Gripper**: End-effector control
- **StateManager**: Centralized state tracking with observers
- **ObjectManager**: Scene understanding and object tracking
- **ManipulationPlant**: Orchestrator that manages all components

## 🏗️ Architecture Overview

```
ManipulationPlant (Orchestrator)
├── Robot (Required)
│   └── Gripper
├── MotionPlanner (Optional)
├── Controllers (Multiple)
├── StateManager (Auto-created)
├── ObjectManager (Auto-created)
├── TaskPlanner (Optional)
├── SceneUnderstanding (Optional)
└── Sensors (Multiple)
```

## 🚀 Quick Start

### Basic Plant

```python
from fullstack_manip.core import ManipulationPlant, Robot, StateManager

# Create plant with builder pattern
plant = (
    ManipulationPlant.builder()
    .with_name("my_plant")
    .with_robot(robot)
    .with_state_manager(StateManager())
    .build()
)
```

### Complete Pick-and-Place System

```python
from fullstack_manip.core import (
    ManipulationPlant,
    Robot,
    Gripper,
    ObjectManager,
    StateManager,
)

# 1. Create components
robot = Robot(model, data, model_path, base_link="base", ee_link="ee")

gripper = Gripper(model, data)
gripper.set_joint_names(["gripper_left", "gripper_right"])
gripper.set_positions(open_pos=0.04, closed_pos=0.0)
robot.gripper = gripper

# 2. Build plant
plant = (
    ManipulationPlant.builder()
    .with_name("pickplace_plant")
    .with_robot(robot)
    .with_state_manager(StateManager())
    .with_object_manager(ObjectManager(model, data))
    .build()
)

# 3. Register scene objects
cube = plant.object_manager.register_object(
    name="red_cube",
    object_type=ObjectType.BOX,
)
cube.set_pose(position=[0.5, 0.0, 0.05])
cube.properties.graspable = True

# 4. Execute high-level commands
plant.move_to_pose(target_position, target_orientation)
plant.execute_skill("pick", object_name="red_cube")
```

## 📦 Core Components

### 1. Robot

Core robot functionality: kinematics, collision detection, joint control.

```python
from fullstack_manip.core import Robot

robot = Robot(
    model=mujoco_model,
    data=mujoco_data,
    model_path="robot.xml",
    base_link="base_link",
    ee_link="end_effector",
)

# Joint control
robot.set_robot_joint_positions([0.1, 0.2, 0.3, 0, 0, 0])
positions = robot.get_robot_joint_positions()

# Forward kinematics
ee_pos, ee_quat = robot.get_body_pose("end_effector")

# Collision checking
in_collision = robot.check_collision()
```

### 2. Gripper

Dedicated gripper control with grasp verification.

```python
from fullstack_manip.core import Gripper

gripper = Gripper(model, data)
gripper.set_joint_names(["gripper_left", "gripper_right"])
gripper.set_positions(open_pos=0.04, closed_pos=0.0)
gripper.set_force_thresholds(min_force=0.1, max_force=10.0)

# Control
gripper.open()
gripper.close()

# Verification
success = gripper.check_grasp_success()
force = gripper.get_current_force()
```

### 3. StateManager

Centralized state tracking with observer pattern.

```python
from fullstack_manip.core import StateManager, StateType

state_mgr = StateManager()

# Register observers
class MyObserver:
    def on_state_update(self, state_type, state_data):
        print(f"State {state_type} updated")

state_mgr.register_observer(StateType.ROBOT, MyObserver())

# Update states
state_mgr.update_robot_state(
    joint_positions=[0.1, 0.2, 0.3],
    joint_velocities=[0, 0, 0],
)

# Query states
robot_state = state_mgr.get_robot_state()
object_state = state_mgr.get_object_state("cube")
```

### 4. ObjectManager

Scene understanding and object tracking.

```python
from fullstack_manip.core import ObjectManager, ObjectType

obj_mgr = ObjectManager(model, data)

# Register objects
cube = obj_mgr.register_object("cube", ObjectType.BOX)
cube.set_pose(position=[0.5, 0, 0.1])
cube.properties.graspable = True
cube.properties.size = [0.05, 0.05, 0.05]

# Query objects
graspable = obj_mgr.get_graspable_objects()
nearest = obj_mgr.find_nearest_object([0.4, 0.1, 0.15])
by_type = obj_mgr.get_objects_by_type(ObjectType.BOX)

# Grasp planning
grasp_points = cube.get_grasp_points()
```

### 5. ManipulationPlant

Main orchestrator managing all components.

```python
from fullstack_manip.core import ManipulationPlant

# Build with fluent API
plant = (
    ManipulationPlant.builder()
    .with_name("my_plant")
    .with_robot(robot)
    .with_motion_planner(planner)
    .with_controller("pid", pid_controller)
    .with_state_manager(state_mgr)
    .with_object_manager(obj_mgr)
    .build()
)

# Lifecycle
plant.initialize()
plant.reset()
plant.step()

# High-level operations
plant.move_to_pose(position, orientation)
plant.execute_skill("pick", object_name="cube")

# Component access
robot = plant.robot
gripper = robot.gripper
controller = plant.get_controller("pid")

# Visualization
print(plant.visualize())
summary = plant.get_state_summary()
```

## 🔌 Plugin Architecture

All components use Protocol interfaces for duck typing:

```python
from fullstack_manip.core.interfaces import (
    RobotInterface,
    MotionPlannerInterface,
    ControllerInterface,
    GripperInterface,
)

# Create custom components
class MyCustomPlanner:
    """Custom planner implementing the interface."""
    
    def plan_to_pose(self, target_pos, target_quat):
        # Your planning logic
        return trajectory

# Plug it in
plant = (
    ManipulationPlant.builder()
    .with_robot(robot)
    .with_motion_planner(MyCustomPlanner())
    .build()
)
```

Available interfaces:
- `RobotInterface`: Core robot operations
- `MotionPlannerInterface`: Path planning
- `ControllerInterface`: Control execution
- `StateManagerInterface`: State management
- `GripperInterface`: Gripper control
- `SensorInterface`: Sensor data
- `ObjectInterface`: Object representation
- `TaskPlannerInterface`: High-level planning
- `SceneUnderstandingInterface`: Scene analysis
- `CollisionCheckerInterface`: Collision detection

## 🧪 Testing

Mock components for easy testing:

```python
# Mock robot
class MockRobot:
    def get_robot_joint_positions(self):
        return [0, 0, 0, 0, 0, 0]
    
    def set_robot_joint_positions(self, positions):
        pass

# Use in tests
plant = (
    ManipulationPlant.builder()
    .with_robot(MockRobot())
    .build()
)
```

## 📋 Examples

### Example 1: Basic Plant
```bash
python examples/manipulation_plant_demo.py
```

Shows:
- Creating a basic plant
- Object management
- State tracking with observers
- Plant visualization
- Mock components

### Example 2: Pick-and-Place
```bash
python examples/pickplace_with_new_architecture.py
```

Shows:
- Complete pick-and-place system
- Integration with MuJoCo
- High-level task execution
- Architecture benefits

## 🏛️ Design Patterns

### 1. Builder Pattern
Fluent API for constructing complex plants:
```python
plant = (
    ManipulationPlant.builder()
    .with_name("plant")
    .with_robot(robot)
    .with_motion_planner(planner)
    .build()
)
```

### 2. Observer Pattern
Decoupled state notifications:
```python
state_mgr.register_observer(StateType.ROBOT, observer)
state_mgr.update_robot_state(...)  # Notifies all observers
```

### 3. Dependency Injection
Components don't create dependencies:
```python
# Good: Inject dependencies
plant = (
    ManipulationPlant.builder()
    .with_robot(robot)
    .with_state_manager(state_mgr)
    .build()
)

# Bad: Components create their own dependencies
# robot = Robot()
# robot.state_manager = StateManager()  # Tight coupling!
```

### 4. Protocol-based Interfaces
Duck typing with type hints:
```python
from typing import Protocol

class RobotInterface(Protocol):
    def get_robot_joint_positions(self): ...
    def set_robot_joint_positions(self, positions): ...
```

## 🔄 Migration from Old Architecture

### Before (Monolithic)
```python
from fullstack_manip.core import Robot

robot = Robot(model, data, model_path)
# Robot creates motion planner internally
# Tight coupling, hard to test
robot.move_to_position(target)
```

### After (Modular)
```python
from fullstack_manip.core import (
    Robot,
    ManipulationPlant,
)

robot = Robot(model, data, model_path)
motion_planner = MyMotionPlanner(robot)

plant = (
    ManipulationPlant.builder()
    .with_robot(robot)
    .with_motion_planner(motion_planner)
    .build()
)

# Separated concerns, easy to test
plant.move_to_pose(target_pos, target_quat)
```

## 📚 Documentation

- [Architecture Vision](../docs/modular-architecture-vision.md)
- [Implementation Progress](../docs/implementation-progress.md)
- [Gripper Quick Reference](../docs/gripper-quick-reference.md)
- [Gripper Architecture](../docs/gripper-architecture.md)

## 🎯 Benefits

1. **Modularity**: Each component has single responsibility
2. **Testability**: Mock any component for isolated testing
3. **Flexibility**: Swap implementations without changing code
4. **Extensibility**: Add new components via Protocol interfaces
5. **Clarity**: Clear separation of concerns
6. **Reusability**: Components work across different systems

## 🚧 Future Enhancements

- [ ] Configuration system (YAML/JSON loading)
- [ ] Graphviz visualization tool
- [ ] Additional motion planners (RRT*, PRM)
- [ ] Learning-based components (IL, RL policies)
- [ ] Multi-robot coordination
- [ ] Async/parallel execution

## 🤝 Contributing

To add a new component:

1. Define Protocol interface in `core/interfaces.py`
2. Implement component class
3. Add to ManipulationPlant via builder
4. Write tests with mock components
5. Update documentation

## 📄 License

[Your License Here]
