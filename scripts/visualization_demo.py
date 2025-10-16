"""Example: Visualizing ManipulationPlant architecture.

This example demonstrates how to generate diagrams of plant
structure, configuration, and data flow.
"""

import sys
from pathlib import Path

from fullstack_manip.core import (
    ManipulationPlant,
    ObjectManager,
    PlantConfig,
    StateManager,
)

# Check if graphviz is available
try:
    import graphviz  # noqa: F401

    HAS_GRAPHVIZ = True
except ImportError:
    HAS_GRAPHVIZ = False
    print("=" * 70)
    print("⚠️  Graphviz not installed!")
    print("=" * 70)
    print("\nTo install graphviz:")
    print("  1. Install Python package: pip install graphviz")
    print("  2. Install system graphviz:")
    print("     - macOS: brew install graphviz")
    print("     - Ubuntu: sudo apt-get install graphviz")
    print("     - Windows: Download from https://graphviz.org/download/")
    print("\nContinuing with text-based visualization only...\n")


def example_1_visualize_config():
    """Example 1: Visualize configuration file."""
    print("=" * 70)
    print("Example 1: Visualize Configuration")
    print("=" * 70)

    # Load configuration
    config = PlantConfig.from_yaml("configs/pickplace.yaml")
    print(f"Loaded config: {config.name}")

    if HAS_GRAPHVIZ:
        from fullstack_manip.core import visualize_config

        # Generate diagram
        output_path = visualize_config(
            config, output_path="diagrams/pickplace_config", format="png"
        )
        if output_path:
            print(f"✓ Generated diagram: {output_path}")
    else:
        print("⚠ Graphviz not available, skipping diagram generation")

    # Text visualization
    print("\nConfiguration Structure:")
    print(
        f"""
{config.name}
├── Robot: {config.robot_config['type']}
├── Gripper: {'✓' if config.gripper_config else '✗'}
├── Motion Planner: {'✓' if config.motion_planner_config else '✗'}
├── Controllers: {len(config.controllers_config)}
├── Sensors: {len(config.sensors_config)}
└── Objects: {len(config.objects_config)}
    """
    )


def example_2_visualize_plant():
    """Example 2: Visualize ManipulationPlant."""
    print("\n" + "=" * 70)
    print("Example 2: Visualize ManipulationPlant")
    print("=" * 70)

    # Create mock robot for example
    class MockRobot:
        def __init__(self):
            self.gripper = None

        def get_robot_joint_positions(self):
            return [0, 0, 0, 0, 0, 0]

        def set_robot_joint_positions(self, positions):
            pass

    # Build plant
    robot = MockRobot()
    plant = (
        ManipulationPlant.builder()
        .with_name("example_plant")
        .with_robot(robot)
        .with_state_manager(StateManager())
        .with_object_manager(ObjectManager())
        .build()
    )

    print(f"Created plant: {plant.name}")

    if HAS_GRAPHVIZ:
        from fullstack_manip.core import PlantVisualizer

        visualizer = PlantVisualizer(plant)

        # Generate component diagram
        output = visualizer.generate_component_diagram(
            "diagrams/example_plant_components", format="png"
        )
        if output:
            print(f"✓ Component diagram: {output}")

        # Generate data flow diagram
        output = visualizer.generate_dataflow_diagram(
            "diagrams/example_plant_dataflow", format="png"
        )
        if output:
            print(f"✓ Data flow diagram: {output}")

        # Generate state diagram
        output = visualizer.generate_state_diagram(
            "diagrams/example_plant_state", format="png"
        )
        if output:
            print(f"✓ State diagram: {output}")
    else:
        print("⚠ Graphviz not available, skipping diagram generation")

    # Text visualization
    print("\nPlant Structure:")
    print(plant.visualize())


def example_3_generate_all_diagrams():
    """Example 3: Generate all diagrams for a config."""
    print("\n" + "=" * 70)
    print("Example 3: Generate All Diagrams")
    print("=" * 70)

    config = PlantConfig.from_yaml("configs/pickplace.yaml")
    print(f"Loaded config: {config.name}")

    if not HAS_GRAPHVIZ:
        print("⚠ Graphviz not available, skipping")
        return

    from fullstack_manip.core import visualize_config

    # Generate config diagram
    output = visualize_config(
        config, output_path=f"diagrams/{config.name}_config", format="png"
    )
    if output:
        print(f"✓ Config diagram: {output}")

    # For plant diagrams, we would need actual MuJoCo model/data
    print("\nNote: Plant diagrams require MuJoCo model/data")
    print("Example usage:")
    print(
        """
    import mujoco
    from fullstack_manip.core import create_plant_from_config
    
    model = mujoco.MjModel.from_xml_path("robot.xml")
    data = mujoco.MjData(model)
    plant = create_plant_from_config(config, model, data)
    
    diagrams = visualize_plant(plant, output_dir="diagrams")
    print(f"Generated: {diagrams}")
    """
    )


def example_4_custom_diagram():
    """Example 4: Create custom diagram."""
    print("\n" + "=" * 70)
    print("Example 4: Custom Diagram")
    print("=" * 70)

    if not HAS_GRAPHVIZ:
        print("⚠ Graphviz not available, skipping")
        return

    import graphviz  # type: ignore

    # Create custom diagram
    dot = graphviz.Digraph(comment="Custom Plant Architecture")
    dot.attr(rankdir="LR")
    dot.attr("node", shape="box", style="rounded,filled")

    # Add nodes
    dot.node("task", "Pick and Place\nTask", fillcolor="lightblue")
    dot.node("plant", "ManipulationPlant", fillcolor="lightblue")
    dot.node("robot", "Robot Arm", fillcolor="lightgreen")
    dot.node("gripper", "Parallel Gripper", fillcolor="lightgreen")
    dot.node("camera", "RGB-D Camera", fillcolor="lightcoral")
    dot.node("planner", "RRT Planner", fillcolor="lightyellow")

    # Add edges
    dot.edge("task", "plant", label="goal")
    dot.edge("plant", "robot", label="commands")
    dot.edge("robot", "gripper", label="controls")
    dot.edge("camera", "plant", label="perception", style="dashed")
    dot.edge("planner", "plant", label="trajectory", style="dashed")

    # Save with error handling
    try:
        dot.render("diagrams/custom_architecture", format="png", cleanup=True)
        print("✓ Custom diagram: diagrams/custom_architecture.png")
    except Exception as e:
        print(f"⚠ Could not render diagram: {e}")
        print("\nMake sure graphviz executables are installed:")
        print("  - macOS:   brew install graphviz")
        print("  - Ubuntu:  sudo apt-get install graphviz")
        print("  - Windows: Download from https://graphviz.org/download/")

    # Show source
    print("\nDiagram source:")
    print(dot.source)


def example_5_batch_visualization():
    """Example 5: Batch visualize all configs."""
    print("\n" + "=" * 70)
    print("Example 5: Batch Visualization")
    print("=" * 70)

    if not HAS_GRAPHVIZ:
        print("⚠ Graphviz not available, skipping")
        return

    from fullstack_manip.core import visualize_config

    # Find all config files
    config_dir = Path("configs")
    yaml_files = list(config_dir.glob("*.yaml"))
    json_files = list(config_dir.glob("*.json"))
    all_configs = yaml_files + json_files

    print(f"Found {len(all_configs)} configuration files\n")

    # Generate diagrams for each
    for config_path in all_configs:
        try:
            # Load config
            if config_path.suffix == ".yaml":
                config = PlantConfig.from_yaml(str(config_path))
            else:
                config = PlantConfig.from_json(str(config_path))

            # Generate diagram
            output_name = f"diagrams/{config.name}_config"
            output = visualize_config(config, output_name, format="png")

            if output:
                print(f"✓ {config_path.name} -> {output}")
        except Exception as e:
            print(f"✗ {config_path.name}: {e}")


def main():
    """Run all visualization examples."""
    print("\n📊 ManipulationPlant Visualization Examples\n")

    # Create diagrams directory
    Path("diagrams").mkdir(exist_ok=True)

    try:
        example_1_visualize_config()
        example_2_visualize_plant()
        example_3_generate_all_diagrams()
        example_4_custom_diagram()
        example_5_batch_visualization()

        print("\n" + "=" * 70)
        print("✅ Visualization examples completed!")
        print("=" * 70)

        if HAS_GRAPHVIZ:
            print("\nGenerated diagrams saved in: diagrams/")
            print("View them with your image viewer!")
        else:
            print("\nInstall graphviz to generate diagram images")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
