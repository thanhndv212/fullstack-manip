# Gripper Module Documentation

Complete guide for the Gripper class and gripper control in fullstack-manip.

---

## Overview

The `Gripper` class provides dedicated control for robotic end-effectors (grippers). It was extracted from the `Robot` class to improve modularity and separation of concerns.

**Location**: `fullstack_manip/core/gripper.py`

---

## Quick Start

```python
from fullstack_manip.core import Gripper
import mujoco

# Create gripper
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

gripper = Gripper(
    model=model,
    data=data,
    joint_names=["left_jaw", "right_jaw"],
    actuator_names=["jaw_actuator"],
)

# Configure
gripper.set_positions(close_pos=0.0, open_pos=1.0)
gripper.set_force_thresholds(grasp_threshold=1.0, release_threshold=0.01)
gripper.set_object_geom("cube")

# Use
gripper.open()
gripper.close()
success = gripper.grasp()  # With verification
```

---

## API Reference

### Constructor

```python
Gripper(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint_names: List[str] = None,
    actuator_names: List[str] = None,
    body_names: List[str] = None,
    collision_checker = None,
    viewer = None,
    end_effector_name: str = None,
    gripper_bodies: List[str] = None
)
```

**Parameters:**
- `model`: MuJoCo model
- `data`: MuJoCo data
- `joint_names`: Gripper joint names (e.g., `["left_jaw", "right_jaw"]`)
- `actuator_names`: Actuator names for control
- `body_names`: Body names for collision detection
- `collision_checker`: Optional collision checker instance
- `viewer`: Optional viewer for visualization
- `end_effector_name`: Name of the end-effector site
- `gripper_bodies`: Legacy parameter, use `body_names`

### Configuration Methods

#### `set_joint_names(joint_names: List[str])`
Set the gripper joint names.

#### `set_positions(close_pos: float, open_pos: float)`
Configure closed and open positions.

```python
gripper.set_positions(close_pos=0.0, open_pos=1.0)
```

#### `set_force_thresholds(grasp_threshold: float, release_threshold: float)`
Set force thresholds for grasp/release verification.

```python
gripper.set_force_thresholds(
    grasp_threshold=1.0,    # Minimum force to confirm grasp
    release_threshold=0.01  # Maximum force to confirm release
)
```

#### `set_object_geom(geom_name: str)`
Set the object geometry name for grasp detection.

```python
gripper.set_object_geom("target_cube")
```

### Control Methods

#### `open(check_release: bool = True) -> bool`
Open the gripper.

```python
success = gripper.open(check_release=True)
```

**Parameters:**
- `check_release`: If True, verifies object was released

**Returns:**
- `True` if successful (or if verification disabled)
- `False` if release verification failed

#### `close(check_grasp: bool = True) -> bool`
Close the gripper.

```python
success = gripper.close(check_grasp=True)
```

**Parameters:**
- `check_grasp`: If True, verifies object was grasped

**Returns:**
- `True` if successful (or if verification disabled)
- `False` if grasp verification failed

#### `grasp() -> bool`
Close gripper and verify grasp.

Equivalent to `close(check_grasp=True)`.

```python
if gripper.grasp():
    print("Object grasped successfully!")
```

#### `release() -> bool`
Open gripper and verify release.

Equivalent to `open(check_release=True)`.

```python
if gripper.release():
    print("Object released successfully!")
```

### State Methods

#### `get_position() -> float`
Get current gripper position.

```python
pos = gripper.get_position()
```

#### `is_closed() -> bool`
Check if gripper is closed.

```python
if gripper.is_closed():
    print("Gripper is closed")
```

#### `is_open() -> bool`
Check if gripper is open.

```python
if gripper.is_open():
    print("Gripper is open")
```

#### `check_grasp_success() -> bool`
Verify if object is grasped (contact forces + in-hand detection).

```python
if gripper.check_grasp_success():
    print("Object is in grasp")
```

### Properties

- `grasp_success` (bool): True if object is currently grasped
- `joint_names` (List[str]): Gripper joint names
- `object_geom` (str): Name of target object geometry
- `close_position` (float): Position value for closed state
- `open_position` (float): Position value for open state

---

## Usage Patterns

### Basic Open/Close

```python
# Simple control without verification
gripper.open(check_release=False)
time.sleep(1.0)
gripper.close(check_grasp=False)
```

### Grasp with Verification

```python
# Move to pre-grasp pose
robot.move_to_position(pre_grasp_pos)

# Open gripper
gripper.open()

# Move to grasp pose
robot.move_to_position(grasp_pos)

# Grasp with verification
if gripper.grasp():
    print("✓ Object grasped")
    # Move to place position
    robot.move_to_position(place_pos)
    
    # Release
    if gripper.release():
        print("✓ Object released")
else:
    print("✗ Grasp failed")
```

### Force Monitoring

```python
# Configure thresholds
gripper.set_force_thresholds(
    grasp_threshold=2.0,     # Higher threshold for heavy objects
    release_threshold=0.05
)

# The grasp() method now requires >= 2.0N to confirm success
success = gripper.grasp()
```

### Integration with ManipulationPlant

```python
from fullstack_manip.core import ManipulationPlant

# Create plant with gripper
plant = (
    ManipulationPlant.builder()
    .with_robot(robot)
    .with_gripper(gripper)
    .build()
)

# Access gripper through plant
plant.gripper.grasp()
plant.gripper.release()
```

---

## Architecture

### Design Principles

1. **Separation of Concerns**: Gripper logic independent from Robot
2. **State Management**: Clear state tracking (open/closed/grasping)
3. **Verification**: Optional force-based success verification
4. **Configurability**: Flexible configuration for different grippers
5. **Integration**: Works standalone or within ManipulationPlant

### State Machine

```
┌─────────┐
│  OPEN   │◄────┐
└────┬────┘     │
     │          │
     │ close()  │ open()
     │          │
     ▼          │
┌─────────┐    │
│ CLOSED  │────┘
└────┬────┘
     │
     │ grasp() + verify
     │
     ▼
┌─────────┐
│GRASPING │
└─────────┘
```

### Grasp Verification

Two-stage verification:
1. **Contact Forces**: Check contact forces exceed threshold
2. **In-Hand Detection**: Verify object is between gripper jaws

```python
def check_grasp_success(self) -> bool:
    # Stage 1: Contact force check
    if total_force < self.grasp_threshold:
        return False
    
    # Stage 2: In-hand detection
    if not self._is_object_in_hand():
        return False
    
    return True
```

---

## Configuration

### YAML Configuration

```yaml
gripper:
  joint_names: [left_jaw, right_jaw]
  actuator_names: [jaw_actuator]
  body_names: [jaw_left_body, jaw_right_body]
  close_position: 0.0
  open_position: 1.0
  grasp_threshold: 1.0
  release_threshold: 0.01
  object_geom: target_object
```

### Programmatic Configuration

```python
gripper = Gripper(model, data)
gripper.set_joint_names(["finger_left", "finger_right"])
gripper.set_positions(close_pos=0.0, open_pos=0.8)
gripper.set_force_thresholds(grasp_threshold=1.5, release_threshold=0.02)
gripper.set_object_geom("cube")
```

---

## Examples

### Example 1: Simple Pick and Place

```python
import mujoco
from fullstack_manip.core import Robot, Gripper

# Setup
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

robot = Robot(model, data, end_effector_name="ee_site")
gripper = Gripper(model, data, joint_names=["jaw"])

gripper.set_positions(close_pos=0.0, open_pos=1.0)
gripper.set_object_geom("cube")

# Pick
robot.move_to_position(pick_pos)
if gripper.grasp():
    # Place
    robot.move_to_position(place_pos)
    gripper.release()
```

### Example 2: With ManipulationPlant

```python
from fullstack_manip.core import ManipulationPlant, PlantConfig

# Load configuration
config = PlantConfig.from_yaml("pickplace.yaml")
plant = create_plant_from_config(config, model, data)

# Use gripper through plant
plant.gripper.grasp()
plant.gripper.release()
```

### Example 3: Custom Verification Logic

```python
class CustomGripper(Gripper):
    def check_grasp_success(self) -> bool:
        # Custom verification logic
        force = self._get_contact_forces()
        camera_detects = self._check_camera_detection()
        
        return force > self.grasp_threshold and camera_detects
```

---

## Troubleshooting

### Grasp Always Fails

**Problem**: `gripper.grasp()` always returns False

**Solutions**:
1. Check force threshold: `gripper.set_force_thresholds(grasp_threshold=0.5)`
2. Verify object_geom is set: `gripper.set_object_geom("cube")`
3. Ensure object is between jaws before closing
4. Check collision detection is enabled

### Release Verification Issues

**Problem**: `gripper.release()` returns False even though object dropped

**Solutions**:
1. Lower release threshold: `gripper.set_force_thresholds(release_threshold=0.0)`
2. Disable verification: `gripper.open(check_release=False)`
3. Check object_geom name matches model

### Position Control Not Working

**Problem**: Gripper doesn't move to commanded position

**Solutions**:
1. Verify joint_names are correct
2. Check actuator_names match model
3. Ensure control mode is position control
4. Verify model has the specified joints

---

## Best Practices

1. **Always Configure Before Use**
   ```python
   gripper.set_positions(close_pos=0.0, open_pos=1.0)
   gripper.set_force_thresholds(grasp_threshold=1.0, release_threshold=0.01)
   ```

2. **Use Verification for Critical Tasks**
   ```python
   if not gripper.grasp():
       handle_grasp_failure()
   ```

3. **Disable Verification for Speed**
   ```python
   # When speed matters more than reliability
   gripper.close(check_grasp=False)
   ```

4. **Configure Thresholds Based on Object**
   ```python
   if object_weight > 1.0:
       gripper.set_force_thresholds(grasp_threshold=2.0)
   ```

5. **Use with ManipulationPlant for Complex Systems**
   ```python
   plant = ManipulationPlant.builder().with_gripper(gripper).build()
   ```

---

## See Also

- [Quick Reference](QUICK_REFERENCE.md) - Fast overview
- [Modular Architecture](modular-architecture-readme.md) - Full architecture guide
- [Configuration System](configuration-system.md) - YAML/JSON configuration
- [Migration Guide](migration-guide.md) - Updating existing code
- [PROJECT_HANDOFF](PROJECT_HANDOFF.md) - Complete project documentation

---

*Last Updated: October 16, 2025*
