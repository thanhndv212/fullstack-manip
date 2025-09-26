# Hardware and Software Requirements

## 1. Hardware Components

- **Manipulator:**
	- Universal Robots UR10 (or compatible serial manipulator)
- **Sensors:**
	- Joint encoders (built-in)
	- Force/torque sensor (optional, e.g., at wrist)
	- Cameras (RGB, RGB-D, stereo, or multi-camera array)
	- IMU (Inertial Measurement Unit)
	- Motion capture markers (for mocap systems)
- **Computers:**
	- Main control PC (recommended: Ubuntu Linux, 8+ cores, 16GB+ RAM, NVIDIA GPU for learning-based tasks)
	- Embedded controllers (if required)
- **Networking:**
	- Ethernet (for UR10 RTDE/ROS2 communication)
	- Wi-Fi (optional, for distributed setups)
- **Safety Devices:**
	- Emergency stop button
	- Safety relays and light curtains (if operating in shared spaces)
	- Protective enclosures (optional)

## 2. Software Dependencies
- **Operating System:**
	- Ubuntu 20.04/22.04 LTS (recommended for ROS2 and MuJoCo)
	- macOS 12 Monterey or newer (supported for development, simulation, and learning-based tasks; ROS2 and MuJoCo compatibility may vary)
- **Middleware:**
	- ROS2 (Foxy, Galactic, Humble, or newer)
	- RTDE (Real-Time Data Exchange) interface for UR10
- **Simulation:**
	- MuJoCo (2.x or later)
	- pybullet
	- mujoco-py or dm_control Python bindings
- **Programming Languages:**
	- Python 3.8+
	- C++ (for performance-critical modules, optional)
- **Python Packages:**
	- numpy, scipy, matplotlib, pandas
    - opencv-python (for vision)
    - mujoco-py, gymnasium, stable-baselines3 (for RL)
    - torch (for learning-based approaches)
    - ur-rtde, urx (for direct UR10 control)
	- pinocchio (for rigid body dynamics and kinematics)
	- figaroh (for system identification, sysID)
    - ROS2 Python/C++ client libraries
    - mink: inverse kinematics

- **Other Tools:**
    - MoveIt (for motion planning)
    - HPP (Humanoid Path Planner, for advanced motion planning)
    - RViz2 (for visualization)
    - Jupyter (for prototyping and notebooks)

## 3. UR10 Kinematics, Dynamics, and Communication

- **Kinematics:**
	- 6-DOF serial manipulator, Denavit-Hartenberg parameters available in URDF
	- Forward and inverse kinematics supported via ROS2 MoveIt and UR libraries
- **Dynamics:**
	- Link masses, inertias, and friction parameters (available in URDF and for sysID)
	- Dynamic model for simulation and model-based control
- **Communication Interfaces:**
	- RTDE (Real-Time Data Exchange) protocol for real-time control and feedback
	- ROS2 topics/services/actions for integration with the robotics stack
	- Dashboard server for robot state and safety monitoring
	- Modbus/TCP (optional, for IO and advanced integration)
