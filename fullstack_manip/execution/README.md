# Behavior Tree Execution Layer

This package provides task orchestration for the fullstack manipulation system using behavior trees (BTs).

## Overview

The execution layer serves as the high-level task coordination system that bridges perception, planning, and control subsystems. It provides:

- **Modularity**: Reusable behaviors and skills
- **Reactivity**: Built-in sensor feedback and error handling
- **Flexibility**: Easy to switch between model-based and learning-based approaches
- **Debuggability**: Visual tree representation and execution logging

## Architecture

```
execution/
├── behaviors/          # Atomic behaviors (leaf nodes)
├── skills/             # Composite behaviors (subtrees)
├── trees/              # Complete task definitions
├── blackboard/         # Shared state management
├── decorators/         # BT modifiers
├── executors/          # BT execution engines
├── visualization/      # Debugging tools
└── ros2/              # ROS2 integration (optional)
```

## Quick Start

### 1. Install Dependencies

```bash
pip install py-trees>=2.2.0  # Behavior tree library
pip install graphviz pydot   # For visualization (optional)
```

### 2. Basic Usage

```python
from fullstack_manip.execution.blackboard import Blackboard
from fullstack_manip.execution.behaviors.motion_behaviors import (
    MoveToCartesianPose
)
from fullstack_manip.planning.motion_planner import MotionPlanner

# Initialize components
blackboard = Blackboard()
motion_planner = MotionPlanner(model, data)

# Create behavior
move_behavior = MoveToCartesianPose(
    name="Move to Target",
    motion_planner=motion_planner,
    blackboard=blackboard
)

# Set target on blackboard
blackboard.set("target_position", np.array([0.3, 0.0, 0.5]))

# Execute behavior
status = move_behavior.update()
```

### 3. Using Skills

```python
from fullstack_manip.execution.skills.pick_skill import PickSkill

# Create pick skill
pick_tree = PickSkill.create(
    motion_planner=motion_planner,
    gripper_controller=gripper,
    force_sensor=force_sensor,
    blackboard=blackboard,
    max_retries=3
)

# Set object position
blackboard.set("object_position", np.array([0.4, 0.2, 0.1]))

# Execute skill
import py_trees
for node in py_trees.visitors.tick_once(pick_tree):
    if node.status != py_trees.common.Status.RUNNING:
        break
```

## Integration with Existing Code

### Model-based Stack

BT behaviors wrap existing controllers:

```python
from fullstack_manip.control.high_level import ImpedanceController

class ImpedanceControlBehavior(BaseBehavior):
    def __init__(self, controller: ImpedanceController, blackboard):
        super().__init__("Impedance Control", blackboard)
        self.controller = controller
    
    def update(self):
        # Use existing controller
        target = self.blackboard.get("target_pose")
        self.controller.set_target(target)
        return Status.SUCCESS if self.controller.is_done() else Status.RUNNING
```

### Learning-based Stack

BT behaviors can invoke RL policies:

```python
class ExecuteRLPolicy(BaseBehavior):
    def __init__(self, policy_model, blackboard):
        super().__init__("RL Policy", blackboard)
        self.policy = policy_model
    
    def update(self):
        obs = self.blackboard.get("observation")
        action = self.policy.predict(obs)
        self.blackboard.set("policy_action", action)
        return Status.SUCCESS
```

### Hybrid Approach

Combine learned and model-based:

```python
from py_trees.composites import Selector

hybrid_tree = Selector(
    name="Hybrid Grasp",
    children=[
        # Try learned approach first
        ExecuteRLPolicy(learned_policy, blackboard),
        # Fallback to model-based
        ModelBasedGrasp(planner, blackboard)
    ]
)
```

## Key Concepts

### Blackboard

Shared state storage for inter-behavior communication:

```python
# Write
blackboard.set("target_position", position)

# Read
position = blackboard.get("target_position")

# Check existence
if blackboard.exists("grasp_success"):
    # ...
```

### Behavior Status

All behaviors return one of:
- `Status.SUCCESS`: Behavior completed successfully
- `Status.FAILURE`: Behavior failed
- `Status.RUNNING`: Behavior still executing
- `Status.INVALID`: Initial state

### Composites

- **Sequence**: Execute children in order (AND logic)
- **Selector**: Try children until one succeeds (OR logic)
- **Parallel**: Execute children concurrently

### Decorators

- **Retry**: Retry child on failure
- **Timeout**: Limit execution time
- **Precondition**: Check condition before execution

## Creating Custom Behaviors

```python
from fullstack_manip.execution.behaviors import BaseBehavior, Status

class MyCustomBehavior(BaseBehavior):
    def __init__(self, name, blackboard):
        super().__init__(name, blackboard)
        # Your initialization
    
    def setup(self):
        # Called once before first execution
        pass
    
    def update(self) -> Status:
        # Main execution logic
        try:
            # Do something
            return Status.SUCCESS
        except Exception as e:
            self.feedback_message = str(e)
            return Status.FAILURE
    
    def terminate(self, new_status):
        # Cleanup when behavior terminates
        pass
```

## Visualization

Visualize behavior trees using Groot or graphviz:

```python
from fullstack_manip.execution.visualization import TreeVisualizer

visualizer = TreeVisualizer()
visualizer.render_tree(pick_tree, filename="pick_tree.pdf")
```

## Testing

Test behaviors in isolation:

```python
import unittest
from fullstack_manip.execution.behaviors.motion_behaviors import (
    MoveToJointConfiguration
)

class TestMotionBehaviors(unittest.TestCase):
    def test_move_to_joint_config(self):
        blackboard = Blackboard()
        behavior = MoveToJointConfiguration(
            "Test Move",
            motion_planner,
            blackboard
        )
        
        blackboard.set("target_joint_config", target)
        status = behavior.update()
        
        self.assertEqual(status, Status.SUCCESS)
        self.assertTrue(blackboard.exists("planned_trajectory"))
```

## Roadmap

- [x] Basic blackboard implementation
- [x] Base behavior class
- [x] Motion behaviors
- [ ] Gripper behaviors
- [ ] Perception behaviors
- [ ] Safety behaviors
- [ ] Complete skills (pick, place, visual alignment)
- [ ] Complete task trees
- [ ] ROS2 integration
- [ ] Visualization tools
- [ ] Comprehensive testing

## References

- [py_trees documentation](https://py-trees.readthedocs.io/)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [Groot visualization tool](https://github.com/BehaviorTree/Groot)
