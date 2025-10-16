# Visualization System Guide

The ManipulationPlant visualization system generates architecture diagrams showing component structure, relationships, and data flow.

## 🎨 Overview

The visualization system provides:
- **Component Diagrams**: Show plant structure and connections
- **Data Flow Diagrams**: Illustrate information flow through system
- **State Diagrams**: Visualize state management architecture
- **Config Diagrams**: Preview configuration files before deployment

## 🛠️ Installation

### 1. Install Python Package

```bash
pip install graphviz
```

### 2. Install System Graphviz

**macOS:**
```bash
brew install graphviz
```

**Ubuntu/Debian:**
```bash
sudo apt-get install graphviz
```

**Windows:**
Download from [graphviz.org/download](https://graphviz.org/download/)

## 🚀 Quick Start

### Visualize Configuration

```python
from fullstack_manip.core import PlantConfig, visualize_config

config = PlantConfig.from_yaml("configs/pickplace.yaml")
output = visualize_config(config, "my_diagram", format="png")
print(f"Diagram saved: {output}")
```

### Visualize Plant

```python
from fullstack_manip.core import ManipulationPlant, visualize_plant

plant = ManipulationPlant.builder().with_robot(robot).build()
diagrams = visualize_plant(plant, output_dir="diagrams")

# Returns:
# {
#   'components': 'diagrams/plant_components.png',
#   'dataflow': 'diagrams/plant_dataflow.png',
#   'state': 'diagrams/plant_state.png'
# }
```

## 📊 Diagram Types

### 1. Component Diagram

Shows plant structure and component relationships.

```python
from fullstack_manip.core import PlantVisualizer

visualizer = PlantVisualizer(plant)
visualizer.generate_component_diagram(
    "diagrams/components",
    format="png"
)
```

**Shows:**
- ManipulationPlant (orchestrator)
- Robot and Gripper
- StateManager and ObjectManager
- Optional components (planner, controllers, sensors)

### 2. Data Flow Diagram

Illustrates how data flows through the system.

```python
visualizer.generate_dataflow_diagram(
    "diagrams/dataflow",
    format="png"
)
```

**Shows:**
- Command flow (task → plant → planner → controller → robot)
- Feedback flow (robot → state → plant)
- Data transformations at each layer

### 3. State Management Diagram

Visualizes state tracking architecture.

```python
visualizer.generate_state_diagram(
    "diagrams/state",
    format="png"
)
```

**Shows:**
- StateManager (central hub)
- State types (robot, object, gripper, task, sensor)
- Observer pattern for notifications

### 4. Configuration Diagram

Preview configuration before creating plant.

```python
from fullstack_manip.core import ConfigVisualizer

config = PlantConfig.from_yaml("config.yaml")
visualizer = ConfigVisualizer(config)
visualizer.generate_config_diagram("diagrams/config", format="png")
```

**Shows:**
- Configuration structure
- Defined components
- Number of objects, controllers, sensors

## 🎯 Usage Patterns

### Generate All Diagrams

```python
from fullstack_manip.core import PlantVisualizer

visualizer = PlantVisualizer(plant)
diagrams = visualizer.generate_all_diagrams(
    output_dir="diagrams",
    format="png"
)

for diagram_type, path in diagrams.items():
    print(f"{diagram_type}: {path}")
```

### Batch Process Configurations

```python
from pathlib import Path
from fullstack_manip.core import PlantConfig, visualize_config

# Find all configs
config_files = Path("configs").glob("*.yaml")

# Generate diagram for each
for config_file in config_files:
    config = PlantConfig.from_yaml(config_file)
    output = visualize_config(
        config,
        f"diagrams/{config.name}",
        format="png"
    )
    print(f"Generated: {output}")
```

### Different Formats

```python
# PNG (default, good for viewing)
visualizer.generate_component_diagram("diagram", format="png")

# PDF (good for documents)
visualizer.generate_component_diagram("diagram", format="pdf")

# SVG (good for web, scalable)
visualizer.generate_component_diagram("diagram", format="svg")

# DOT source (for manual editing)
dot_source = visualizer.generate_component_diagram()
print(dot_source)
```

### Custom Diagrams

```python
import graphviz

# Create custom diagram
dot = graphviz.Digraph(comment='My Custom Architecture')
dot.attr(rankdir='LR')
dot.attr('node', shape='box', style='filled')

# Add nodes
dot.node('sensor', 'Camera', fillcolor='lightblue')
dot.node('plant', 'Plant', fillcolor='lightgreen')
dot.node('robot', 'Robot', fillcolor='lightyellow')

# Add edges
dot.edge('sensor', 'plant', label='perception')
dot.edge('plant', 'robot', label='control')

# Save
dot.render('my_diagram', format='png', cleanup=True)
```

## 📋 API Reference

### PlantVisualizer

```python
class PlantVisualizer:
    """Visualizer for ManipulationPlant."""
    
    def __init__(self, plant: ManipulationPlant):
        """Initialize with plant to visualize."""
    
    def generate_component_diagram(
        self,
        output_path: Optional[str] = None,
        format: str = 'png'
    ) -> Optional[str]:
        """Generate component architecture diagram.
        
        Args:
            output_path: Path without extension (or None for source)
            format: Output format (png, pdf, svg)
            
        Returns:
            Path to generated file or diagram source
        """
    
    def generate_dataflow_diagram(...):
        """Generate data flow diagram."""
    
    def generate_state_diagram(...):
        """Generate state management diagram."""
    
    def generate_all_diagrams(
        self,
        output_dir: str = 'diagrams',
        format: str = 'png'
    ) -> dict[str, Optional[str]]:
        """Generate all diagrams.
        
        Returns:
            Dictionary mapping diagram type to file path
        """
```

### ConfigVisualizer

```python
class ConfigVisualizer:
    """Visualizer for PlantConfig."""
    
    def __init__(self, config: PlantConfig):
        """Initialize with config to visualize."""
    
    def generate_config_diagram(
        self,
        output_path: Optional[str] = None,
        format: str = 'png'
    ) -> Optional[str]:
        """Generate configuration diagram."""
```

### Convenience Functions

```python
def visualize_plant(
    plant: ManipulationPlant,
    output_dir: str = 'diagrams',
    format: str = 'png'
) -> dict[str, Optional[str]]:
    """Quick function to visualize a plant."""

def visualize_config(
    config: PlantConfig,
    output_path: str = 'config_diagram',
    format: str = 'png'
) -> Optional[str]:
    """Quick function to visualize a configuration."""
```

## 🎨 Customization

### Node Colors

Default colors:
- **Light Blue**: Core plant/orchestrator
- **Light Green**: Robot and gripper  
- **Light Yellow**: State and object management
- **Light Coral**: Optional components (planners, controllers, sensors)
- **Pink**: Observers

### Diagram Layout

```python
dot = graphviz.Digraph()

# Top-to-bottom (default)
dot.attr(rankdir='TB')

# Left-to-right
dot.attr(rankdir='LR')

# Bottom-to-top
dot.attr(rankdir='BT')

# Right-to-left
dot.attr(rankdir='RL')
```

### Node Styles

```python
dot.node('id', 'Label', 
    shape='box',         # box, circle, ellipse, etc.
    style='filled',      # filled, rounded, dashed, etc.
    fillcolor='blue',    # color name or hex
    fontname='Arial',    # font
    fontsize='12'        # size
)
```

### Edge Styles

```python
dot.edge('from', 'to',
    label='connection',  # edge label
    style='dashed',      # solid, dashed, dotted, bold
    color='red',         # color
    dir='both'           # none, forward, back, both
)
```

## 📸 Example Output

### Component Diagram
Shows plant structure with all components and their relationships.

### Data Flow Diagram
Shows how commands flow forward (task → robot) and feedback flows backward (sensors → state).

### State Diagram
Shows StateManager connected to all state types with observers.

### Config Diagram
Shows configuration structure before plant creation.

## 🔧 Troubleshooting

### "graphviz not installed"

**Solution**: Install both Python package AND system graphviz:
```bash
pip install graphviz
brew install graphviz  # macOS
```

### "dot command not found"

**Solution**: System graphviz not installed or not in PATH.
```bash
# macOS
brew install graphviz

# Ubuntu
sudo apt-get install graphviz

# Verify
which dot
```

### Diagrams not generating

**Check**:
1. Import successful: `from fullstack_manip.core import visualize_plant`
2. Output directory exists: `Path("diagrams").mkdir(exist_ok=True)`
3. Permissions: Can write to output directory

### Customizing appearance

**Modify** the `PlantVisualizer` class or create custom diagrams with graphviz directly.

## 📚 Examples

Run the complete demo:
```bash
python examples/visualization_demo.py
```

This demonstrates:
1. Visualizing configurations
2. Visualizing plants
3. Generating all diagrams
4. Creating custom diagrams
5. Batch processing configs

## 💡 Best Practices

1. **Generate Early**: Visualize configs before deployment
2. **Version Control**: Include diagrams in documentation
3. **Batch Generate**: Update all diagrams when architecture changes
4. **Use PDF for Docs**: PNG for viewing, PDF for documents
5. **Custom Diagrams**: Create task-specific diagrams as needed

## 🔗 Integration

### With CI/CD

```yaml
# .github/workflows/docs.yml
- name: Generate Diagrams
  run: |
    python -c "
    from fullstack_manip.core import PlantConfig, visualize_config
    from pathlib import Path
    
    for cfg in Path('configs').glob('*.yaml'):
        config = PlantConfig.from_yaml(cfg)
        visualize_config(config, f'docs/diagrams/{config.name}')
    "
```

### With Documentation

```markdown
# System Architecture

![Component Diagram](diagrams/plant_components.png)

The plant consists of...

![Data Flow](diagrams/plant_dataflow.png)

Data flows through...
```

## 📖 See Also

- [Main Architecture Guide](modular-architecture-readme.md)
- [Configuration System](configuration-system.md)
- [Quick Reference](QUICK_REFERENCE.md)
- [Graphviz Documentation](https://graphviz.org/documentation/)

---

**Status**: ✅ Visualization system complete and ready to use!
