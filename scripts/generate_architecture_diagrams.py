"""Generate architecture diagrams for the modular manipulation system."""

try:
    import graphviz  # type: ignore

    HAS_GRAPHVIZ = True
except ImportError:
    HAS_GRAPHVIZ = False
    print(
        "Warning: graphviz not installed. "
        "Install with: pip install graphviz"
    )


def create_component_diagram():
    """Create diagram showing component relationships."""
    if not HAS_GRAPHVIZ:
        return None

    dot = graphviz.Digraph(comment="ManipulationPlant Architecture")
    dot.attr(rankdir="TB")
    dot.attr("node", shape="box", style="rounded,filled")

    # ManipulationPlant (orchestrator)
    dot.node(
        "plant", "ManipulationPlant\n(Orchestrator)", fillcolor="lightblue"
    )

    # Core components
    dot.node("robot", "Robot\n(Kinematics)", fillcolor="lightgreen")
    dot.node("gripper", "Gripper\n(End-effector)", fillcolor="lightgreen")
    dot.node(
        "state", "StateManager\n(State Tracking)", fillcolor="lightyellow"
    )
    dot.node(
        "objects",
        "ObjectManager\n(Scene Understanding)",
        fillcolor="lightyellow",
    )

    # Optional components
    dot.node(
        "planner", "MotionPlanner\n(Path Planning)", fillcolor="lightcoral"
    )
    dot.node("controller", "Controller\n(Execution)", fillcolor="lightcoral")
    dot.node("sensor", "Sensors\n(Perception)", fillcolor="lightcoral")

    # Relationships
    dot.edge("plant", "robot", label="manages")
    dot.edge("plant", "state", label="uses")
    dot.edge("plant", "objects", label="uses")
    dot.edge("plant", "planner", label="uses", style="dashed")
    dot.edge("plant", "controller", label="uses", style="dashed")
    dot.edge("plant", "sensor", label="uses", style="dashed")
    dot.edge("robot", "gripper", label="has")

    return dot


def create_interface_diagram():
    """Create diagram showing protocol interfaces."""
    if not HAS_GRAPHVIZ:
        return None

    dot = graphviz.Digraph(comment="Protocol Interfaces")
    dot.attr(rankdir="LR")
    dot.attr("node", shape="box", style="filled")

    # Interfaces
    interfaces = [
        "RobotInterface",
        "GripperInterface",
        "MotionPlannerInterface",
        "ControllerInterface",
        "StateManagerInterface",
        "ObjectInterface",
        "SensorInterface",
        "TaskPlannerInterface",
        "SceneUnderstandingInterface",
        "CollisionCheckerInterface",
    ]

    # Add interface nodes
    for iface in interfaces:
        dot.node(iface, f"<<{iface}>>", fillcolor="lightyellow")

    # Implementation examples
    dot.node("Robot", "Robot", fillcolor="lightgreen")
    dot.node("Gripper", "Gripper", fillcolor="lightgreen")
    dot.node("StateManager", "StateManager", fillcolor="lightgreen")
    dot.node("MockRobot", "MockRobot\n(Test)", fillcolor="lightcoral")

    # Relationships
    dot.edge("Robot", "RobotInterface", label="implements", style="dashed")
    dot.edge("Gripper", "GripperInterface", label="implements", style="dashed")
    dot.edge(
        "StateManager",
        "StateManagerInterface",
        label="implements",
        style="dashed",
    )
    dot.edge("MockRobot", "RobotInterface", label="implements", style="dashed")

    return dot


def create_dataflow_diagram():
    """Create diagram showing data flow in the system."""
    if not HAS_GRAPHVIZ:
        return None

    dot = graphviz.Digraph(comment="Data Flow")
    dot.attr(rankdir="TB")
    dot.attr("node", shape="box", style="rounded,filled")

    # Layers
    dot.node("task", "High-Level Task\n(pick, place)", fillcolor="lightblue")
    dot.node("plant", "ManipulationPlant", fillcolor="lightblue")
    dot.node("planner", "Motion Planner", fillcolor="lightyellow")
    dot.node("controller", "Controller", fillcolor="lightyellow")
    dot.node("robot", "Robot", fillcolor="lightgreen")
    dot.node("sim", "MuJoCo Simulation", fillcolor="lightgray")
    dot.node("state", "StateManager", fillcolor="lightcoral")

    # Forward flow (commands)
    dot.edge("task", "plant", label="goal")
    dot.edge("plant", "planner", label="target pose")
    dot.edge("planner", "controller", label="trajectory")
    dot.edge("controller", "robot", label="joint commands")
    dot.edge("robot", "sim", label="actuate")

    # Backward flow (feedback)
    dot.edge("sim", "state", label="sensor data", style="dashed")
    dot.edge("state", "plant", label="current state", style="dashed")

    return dot


def create_pickplace_sequence():
    """Create sequence diagram for pick-and-place."""
    if not HAS_GRAPHVIZ:
        return None

    dot = graphviz.Digraph(comment="Pick-Place Sequence")
    dot.attr(rankdir="TB")
    dot.attr("node", shape="box", style="filled")

    # Steps
    steps = [
        ("1", "Find Object", "lightblue"),
        ("2", "Plan Approach", "lightblue"),
        ("3", "Open Gripper", "lightyellow"),
        ("4", "Move to Pre-Grasp", "lightyellow"),
        ("5", "Move to Grasp", "lightyellow"),
        ("6", "Close Gripper", "lightyellow"),
        ("7", "Verify Grasp", "lightcoral"),
        ("8", "Lift Object", "lightyellow"),
        ("9", "Move to Target", "lightyellow"),
        ("10", "Place Object", "lightyellow"),
        ("11", "Open Gripper", "lightyellow"),
        ("12", "Retreat", "lightyellow"),
    ]

    prev = None
    for num, desc, color in steps:
        node_id = f"step{num}"
        dot.node(node_id, f"{num}. {desc}", fillcolor=color)
        if prev:
            dot.edge(prev, node_id)
        prev = node_id

    return dot


def main():
    """Generate all diagrams."""
    if not HAS_GRAPHVIZ:
        print("Please install graphviz: pip install graphviz")
        print("Also install system graphviz: brew install graphviz (macOS)")
        return

    print("Generating architecture diagrams...")

    # Component diagram
    dot = create_component_diagram()
    if dot:
        dot.render(
            "docs/diagrams/component_architecture", format="png", cleanup=True
        )
        print("✓ Created: docs/diagrams/component_architecture.png")

    # Interface diagram
    dot = create_interface_diagram()
    if dot:
        dot.render(
            "docs/diagrams/protocol_interfaces", format="png", cleanup=True
        )
        print("✓ Created: docs/diagrams/protocol_interfaces.png")

    # Data flow diagram
    dot = create_dataflow_diagram()
    if dot:
        dot.render("docs/diagrams/data_flow", format="png", cleanup=True)
        print("✓ Created: docs/diagrams/data_flow.png")

    # Pick-place sequence
    dot = create_pickplace_sequence()
    if dot:
        dot.render(
            "docs/diagrams/pickplace_sequence", format="png", cleanup=True
        )
        print("✓ Created: docs/diagrams/pickplace_sequence.png")

    print("\n✅ All diagrams generated successfully!")
    print("View them in: docs/diagrams/")


if __name__ == "__main__":
    main()
