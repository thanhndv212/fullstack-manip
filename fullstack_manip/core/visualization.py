"""Visualization tools for ManipulationPlant architecture.

This module provides tools to visualize plant structure, component
relationships, and generate architecture diagrams.
"""

from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from fullstack_manip.core import ManipulationPlant, PlantConfig

try:
    import graphviz  # type: ignore

    HAS_GRAPHVIZ = True
except ImportError:
    HAS_GRAPHVIZ = False


def _render_diagram(dot, output_path: str, format: str) -> Optional[str]:
    """Helper to safely render a graphviz diagram.

    Args:
        dot: Graphviz Digraph object
        output_path: Path to save diagram
        format: Output format

    Returns:
        Path to generated file or None if rendering failed
    """
    try:
        dot.render(output_path, format=format, cleanup=True)
        return f"{output_path}.{format}"
    except Exception as e:
        error_msg = str(e)
        if "failed to execute" in error_msg or "ExecutableNotFound" in str(
            type(e)
        ):
            print("\n" + "=" * 70)
            print("⚠️  Graphviz system executables not found!")
            print("=" * 70)
            print("\nThe graphviz Python package is installed, but the")
            print("system executables (dot, neato, etc.) are missing.\n")
            print("To install graphviz system executables:")
            print("  - macOS:   brew install graphviz")
            print("  - Ubuntu:  sudo apt-get install graphviz")
            print("  - Windows: Download from https://graphviz.org/download/")
            print("\nFalling back to text visualization...\n")
        else:
            print(f"Warning: Failed to render diagram: {e}")
        return None


class PlantVisualizer:
    """Visualizer for ManipulationPlant architecture.

    Generates diagrams showing component structure, connections,
    and data flow.
    """

    def __init__(self, plant: "ManipulationPlant"):
        """Initialize visualizer.

        Args:
            plant: ManipulationPlant to visualize
        """
        self.plant = plant

    def generate_component_diagram(
        self,
        output_path: Optional[str] = None,
        format: str = "png",
    ) -> Optional[str]:
        """Generate component architecture diagram.

        Args:
            output_path: Path to save diagram (without extension)
            format: Output format (png, pdf, svg)

        Returns:
            Path to generated file, or None if graphviz not available
        """
        if not HAS_GRAPHVIZ:
            print("Warning: graphviz not installed")
            return None

        dot = graphviz.Digraph(comment=f"{self.plant.name} Architecture")
        dot.attr(rankdir="TB")
        dot.attr("node", shape="box", style="rounded,filled")

        # Central plant node
        dot.node(
            "plant",
            f"{self.plant.name}\n(ManipulationPlant)",
            fillcolor="lightblue",
        )

        # Robot (required)
        if self.plant.robot:
            dot.node("robot", "Robot", fillcolor="lightgreen")
            dot.edge("plant", "robot", label="manages")

            # Gripper (if present)
            if hasattr(self.plant.robot, "gripper"):
                if self.plant.robot.gripper:
                    dot.node("gripper", "Gripper", fillcolor="lightgreen")
                    dot.edge("robot", "gripper", label="has")

        # State manager
        if self.plant.state_manager:
            dot.node("state", "StateManager", fillcolor="lightyellow")
            dot.edge("plant", "state", label="uses")

        # Object manager
        if self.plant.object_manager:
            obj_count = len(self.plant.object_manager.get_all_objects())
            dot.node(
                "objects",
                f"ObjectManager\n({obj_count} objects)",
                fillcolor="lightyellow",
            )
            dot.edge("plant", "objects", label="uses")

        # Motion planner
        if self.plant.motion_planner:
            dot.node("planner", "MotionPlanner", fillcolor="lightcoral")
            dot.edge("plant", "planner", label="uses", style="dashed")

        # Controllers
        if self.plant.controllers:
            for name in self.plant.controllers.keys():
                node_id = f"ctrl_{name}"
                dot.node(
                    node_id, f"Controller\n({name})", fillcolor="lightcoral"
                )
                dot.edge("plant", node_id, label="uses", style="dashed")

        # Sensors
        if self.plant._sensors:
            for name in self.plant._sensors.keys():
                node_id = f"sensor_{name}"
                dot.node(node_id, f"Sensor\n({name})", fillcolor="lightcoral")
                dot.edge("plant", node_id, label="uses", style="dashed")

        if output_path:
            return _render_diagram(dot, output_path, format)

        return dot.source

    def generate_dataflow_diagram(
        self,
        output_path: Optional[str] = None,
        format: str = "png",
    ) -> Optional[str]:
        """Generate data flow diagram.

        Args:
            output_path: Path to save diagram
            format: Output format

        Returns:
            Path to generated file or diagram source
        """
        if not HAS_GRAPHVIZ:
            print("Warning: graphviz not installed")
            return None

        dot = graphviz.Digraph(comment="Data Flow")
        dot.attr(rankdir="TB")
        dot.attr("node", shape="box", style="rounded,filled")

        # Layers
        dot.node("task", "High-Level Task", fillcolor="lightblue")
        dot.node("plant", "ManipulationPlant", fillcolor="lightblue")
        dot.node("planner", "Motion Planner", fillcolor="lightyellow")
        dot.node("controller", "Controller", fillcolor="lightyellow")
        dot.node("robot", "Robot", fillcolor="lightgreen")
        dot.node("state", "StateManager", fillcolor="lightcoral")

        # Forward flow (commands)
        dot.edge("task", "plant", label="goal")
        dot.edge("plant", "planner", label="target")
        dot.edge("planner", "controller", label="trajectory")
        dot.edge("controller", "robot", label="commands")

        # Backward flow (feedback)
        dot.edge("robot", "state", label="sensor data", style="dashed")
        dot.edge("state", "plant", label="current state", style="dashed")

        if output_path:
            return _render_diagram(dot, output_path, format)

        return dot.source

    def generate_state_diagram(
        self,
        output_path: Optional[str] = None,
        format: str = "png",
    ) -> Optional[str]:
        """Generate state management diagram.

        Args:
            output_path: Path to save diagram
            format: Output format

        Returns:
            Path to generated file or diagram source
        """
        if not HAS_GRAPHVIZ:
            print("Warning: graphviz not installed")
            return None

        dot = graphviz.Digraph(comment="State Management")
        dot.attr(rankdir="LR")
        dot.attr("node", shape="box", style="filled")

        # State manager (center)
        dot.node("state_mgr", "StateManager", fillcolor="lightyellow")

        # State types
        states = [
            ("robot", "RobotState", "lightgreen"),
            ("object", "ObjectState", "lightgreen"),
            ("gripper", "GripperState", "lightgreen"),
            ("task", "TaskState", "lightblue"),
            ("sensor", "SensorState", "lightcoral"),
        ]

        for state_id, state_name, color in states:
            dot.node(state_id, state_name, fillcolor=color)
            dot.edge("state_mgr", state_id, label="tracks", dir="both")

        # Observers
        dot.node("observers", "Observers", fillcolor="pink")
        dot.edge("state_mgr", "observers", label="notifies")

        if output_path:
            return _render_diagram(dot, output_path, format)

        return dot.source

    def generate_all_diagrams(
        self, output_dir: str = "diagrams", format: str = "png"
    ) -> dict[str, Optional[str]]:
        """Generate all diagrams.

        Args:
            output_dir: Directory to save diagrams
            format: Output format

        Returns:
            Dictionary mapping diagram type to file path
        """
        from pathlib import Path

        Path(output_dir).mkdir(exist_ok=True)

        results = {}

        # Component diagram
        path = f"{output_dir}/{self.plant.name}_components"
        results["components"] = self.generate_component_diagram(path, format)

        # Data flow diagram
        path = f"{output_dir}/{self.plant.name}_dataflow"
        results["dataflow"] = self.generate_dataflow_diagram(path, format)

        # State diagram
        path = f"{output_dir}/{self.plant.name}_state"
        results["state"] = self.generate_state_diagram(path, format)

        return results


class ConfigVisualizer:
    """Visualizer for PlantConfig.

    Generates diagrams from configuration files before
    creating the actual plant.
    """

    def __init__(self, config: "PlantConfig"):
        """Initialize visualizer.

        Args:
            config: PlantConfig to visualize
        """
        self.config = config

    def generate_config_diagram(
        self,
        output_path: Optional[str] = None,
        format: str = "png",
    ) -> Optional[str]:
        """Generate diagram from configuration.

        Args:
            output_path: Path to save diagram
            format: Output format

        Returns:
            Path to generated file or diagram source
        """
        if not HAS_GRAPHVIZ:
            print("Warning: graphviz not installed")
            return None

        dot = graphviz.Digraph(comment=f"{self.config.name} Config")
        dot.attr(rankdir="TB")
        dot.attr("node", shape="box", style="rounded,filled")

        # Plant node
        dot.node(
            "plant",
            f"{self.config.name}\n(Configuration)",
            fillcolor="lightblue",
        )

        # Robot
        robot_cfg = self.config.robot_config
        dot.node(
            "robot", f"Robot\n{robot_cfg['type']}", fillcolor="lightgreen"
        )
        dot.edge("plant", "robot")

        # Gripper
        if self.config.gripper_config:
            dot.node("gripper", "Gripper", fillcolor="lightgreen")
            dot.edge("robot", "gripper")

        # Motion planner
        if self.config.motion_planner_config:
            planner_type = self.config.motion_planner_config["type"]
            dot.node(
                "planner",
                f"MotionPlanner\n{planner_type}",
                fillcolor="lightcoral",
            )
            dot.edge("plant", "planner", style="dashed")

        # Controllers
        for name, ctrl_cfg in self.config.controllers_config.items():
            node_id = f"ctrl_{name}"
            ctrl_type = ctrl_cfg.get("type", "unknown")
            dot.node(node_id, f"{name}\n({ctrl_type})", fillcolor="lightcoral")
            dot.edge("plant", node_id, style="dashed")

        # Objects
        obj_count = len(self.config.objects_config)
        if obj_count > 0:
            dot.node(
                "objects", f"{obj_count} Objects", fillcolor="lightyellow"
            )
            dot.edge("plant", "objects")

        if output_path:
            return _render_diagram(dot, output_path, format)

        return dot.source


def visualize_plant(
    plant: "ManipulationPlant",
    output_dir: str = "diagrams",
    format: str = "png",
) -> dict[str, Optional[str]]:
    """Convenience function to visualize a plant.

    Args:
        plant: ManipulationPlant to visualize
        output_dir: Directory to save diagrams
        format: Output format

    Returns:
        Dictionary of generated diagram paths
    """
    visualizer = PlantVisualizer(plant)
    return visualizer.generate_all_diagrams(output_dir, format)


def visualize_config(
    config: "PlantConfig",
    output_path: str = "config_diagram",
    format: str = "png",
) -> Optional[str]:
    """Convenience function to visualize a configuration.

    Args:
        config: PlantConfig to visualize
        output_path: Path to save diagram
        format: Output format

    Returns:
        Path to generated diagram
    """
    visualizer = ConfigVisualizer(config)
    return visualizer.generate_config_diagram(output_path, format)
