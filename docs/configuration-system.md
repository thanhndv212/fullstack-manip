# Configuration System Guide

The ManipulationPlant configuration system allows you to define manipulation systems declaratively using YAML or JSON files.

## 🎯 Benefits

- **Version Control**: Track plant configurations in git
- **Reproducibility**: Share exact system setups
- **Iteration**: Quickly try different configurations
- **Documentation**: Self-documenting system structure
- **No Code Changes**: Modify systems without changing code

## 📝 Configuration Format

### Minimal Configuration

```yaml
name: my_plant

robot:
  type: generic
  model_path: "path/to/robot.xml"
  base_link: "base_link"
  ee_link: "end_effector"
```

### Complete Configuration

```yaml
name: pickplace_plant

# Robot configuration (required)
robot:
  type: generic
  model_path: "hardware/urdf/robot.xml"
  base_link: "base_link"
  ee_link: "end_effector"

# Gripper configuration (optional)
gripper:
  joint_names:
    - "gripper_left"
    - "gripper_right"
  open_position: 0.04
  closed_position: 0.0
  min_force: 0.1
  max_force: 10.0

# Motion planner configuration (optional)
motion_planner:
  type: rrt
  max_planning_time: 5.0
  step_size: 0.1

# Controllers (optional)
controllers:
  pid_controller:
    type: pid
    kp: 100.0
    ki: 0.0
    kd: 10.0

# Sensors (optional)
sensors:
  camera:
    type: camera
    resolution: [640, 480]

# Objects in the scene (optional)
objects:
  - name: red_cube
    type: box
    position: [0.5, 0.0, 0.05]
    orientation: [1, 0, 0, 0]  # w, x, y, z
    properties:
      size: [0.05, 0.05, 0.05]
      mass: 0.1
      graspable: true
      color: [1, 0, 0, 1]
```

## 🚀 Usage

### Load from YAML

```python
from fullstack_manip.core import PlantConfig, create_plant_from_config

# Load configuration
config = PlantConfig.from_yaml("configs/pickplace.yaml")

# Create plant (requires MuJoCo model/data)
import mujoco
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

plant = create_plant_from_config(config, model, data)
```

### Load from JSON

```python
config = PlantConfig.from_json("configs/assembly.json")
plant = create_plant_from_config(config, model, data)
```

### Create Programmatically

```python
from fullstack_manip.core import PlantConfig

config_dict = {
    "name": "my_plant",
    "robot": {
        "type": "generic",
        "model_path": "robot.xml",
        "base_link": "base",
        "ee_link": "ee",
    },
    "objects": [
        {
            "name": "cube",
            "type": "box",
            "position": [0.5, 0.0, 0.05],
            "properties": {
                "size": [0.05, 0.05, 0.05],
                "graspable": True,
            }
        }
    ]
}

config = PlantConfig(config_dict)

# Save for later use
config.to_yaml("my_plant.yaml")
config.to_json("my_plant.json")
```

## 📋 Configuration Fields

### Robot (Required)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | string | Yes | Robot type (`generic`, etc.) |
| `model_path` | string | Yes | Path to URDF/XML file |
| `base_link` | string | Yes | Base link name |
| `ee_link` | string | Yes | End-effector link name |

### Gripper (Optional)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `joint_names` | list | Yes | Gripper joint names |
| `open_position` | float | Yes | Open position (meters/radians) |
| `closed_position` | float | Yes | Closed position |
| `min_force` | float | No | Minimum grasp force |
| `max_force` | float | No | Maximum grasp force |

### Motion Planner (Optional)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | string | Yes | Planner type (`rrt`, `rrt_star`, `prm`) |
| `max_planning_time` | float | No | Max planning time (seconds) |
| `step_size` | float | No | Planning step size |

### Controllers (Optional)

Dictionary of controller name to controller config:

```yaml
controllers:
  my_controller:
    type: pid  # Controller type
    kp: 100.0  # Type-specific parameters
    ki: 0.0
    kd: 10.0
```

### Sensors (Optional)

Dictionary of sensor name to sensor config:

```yaml
sensors:
  my_camera:
    type: camera
    resolution: [640, 480]
    fov: 60
```

### Objects (Optional)

List of object configurations:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `name` | string | Yes | Object name |
| `type` | string | Yes | Object type (`box`, `sphere`, `cylinder`, etc.) |
| `position` | list[float] | No | [x, y, z] position |
| `orientation` | list[float] | No | [w, x, y, z] quaternion |
| `properties` | dict | No | Object properties (see below) |

#### Object Properties

| Field | Type | Description |
|-------|------|-------------|
| `size` | list[float] | Dimensions (type-specific) |
| `mass` | float | Mass in kg |
| `graspable` | bool | Can be grasped |
| `color` | list[float] | RGBA color [r, g, b, a] |

## 📂 Example Configurations

### Pick-and-Place

See: `configs/pickplace.yaml`

```bash
python examples/config_system_demo.py
```

### Assembly

See: `configs/assembly.json`

### Minimal

See: `configs/minimal.yaml`

## ✅ Validation

Configurations are automatically validated:

```python
from fullstack_manip.core import PlantConfig, ConfigurationError

try:
    config = PlantConfig.from_yaml("invalid.yaml")
except ConfigurationError as e:
    print(f"Invalid configuration: {e}")
```

**Validated:**
- Required fields present
- Correct data types
- Valid enum values (object types, etc.)

## 🔄 Workflow

1. **Create configuration file**:
   ```bash
   cp configs/minimal.yaml configs/my_task.yaml
   # Edit my_task.yaml
   ```

2. **Load and test**:
   ```python
   config = PlantConfig.from_yaml("configs/my_task.yaml")
   plant = create_plant_from_config(config, model, data)
   ```

3. **Iterate**:
   - Modify YAML file
   - Reload
   - Test

4. **Version control**:
   ```bash
   git add configs/my_task.yaml
   git commit -m "Add my_task configuration"
   ```

## 🎨 Configuration Patterns

### Development vs Production

```yaml
# dev.yaml - Fast iteration
name: dev_plant
robot:
  type: generic
  model_path: "robot_simple.xml"
motion_planner:
  max_planning_time: 1.0  # Fast planning

# production.yaml - Robust
name: prod_plant
robot:
  type: generic
  model_path: "robot_full.xml"
motion_planner:
  max_planning_time: 10.0  # Better plans
```

### Task-Specific Configs

```
configs/
├── pickplace.yaml
├── assembly.yaml
├── sorting.yaml
└── inspection.yaml
```

### Environment-Specific Objects

```yaml
# lab_environment.yaml
objects:
  - name: table
    type: box
    position: [0, 0, 0]
    properties:
      size: [1.0, 1.0, 0.02]
      graspable: false
```

## 🔧 Advanced Usage

### Query Configuration

```python
config = PlantConfig.from_yaml("config.yaml")

# Access specific parts
print(f"Plant name: {config.name}")
print(f"Robot type: {config.robot_config['type']}")
print(f"Has gripper: {config.gripper_config is not None}")
print(f"Number of objects: {len(config.objects_config)}")
```

### Modify and Save

```python
config = PlantConfig.from_yaml("config.yaml")

# Modify
config_dict = config.to_dict()
config_dict['objects'].append({
    "name": "new_object",
    "type": "sphere",
    "position": [0.6, 0.0, 0.05],
})

# Save modified
new_config = PlantConfig(config_dict)
new_config.to_yaml("config_modified.yaml")
```

### Merge Configurations

```python
base_config = PlantConfig.from_yaml("base.yaml")
task_config = PlantConfig.from_yaml("task_objects.yaml")

# Merge objects
merged_dict = base_config.to_dict()
merged_dict['objects'].extend(task_config.objects_config)

merged_config = PlantConfig(merged_dict)
```

## 📚 See Also

- [Main Architecture README](modular-architecture-readme.md)
- [Example: Config System Demo](../examples/config_system_demo.py)
- [Implementation Summary](implementation-summary.md)

## 🎯 Next Steps

1. **Try the examples**:
   ```bash
   python examples/config_system_demo.py
   ```

2. **Create your own config**:
   ```bash
   cp configs/minimal.yaml configs/my_plant.yaml
   ```

3. **Load and use**:
   ```python
   config = PlantConfig.from_yaml("configs/my_plant.yaml")
   plant = create_plant_from_config(config, model, data)
   ```

4. **Version control your configs**!
