# Quickstart Guide: Digital Twin Simulation for Humanoid Robots

## Prerequisites

- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- ROS 2 Humble Hawksbill installed
- Gazebo Harmonic (or Garden) installed
- Unity Hub with Unity 2022.3 LTS or newer (optional for visualization)
- Python 3.10 or newer
- Docusaurus development environment

## Setup Steps

### 1. Install ROS 2 and Gazebo
```bash
# Follow official ROS 2 Humble installation guide
# Install Gazebo Harmonic
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### 2. Install Unity (Optional - for visualization layer)
```bash
# Download and install Unity Hub from unity.com
# Install Unity 2022.3 LTS through Unity Hub
# Import the ROS# package for Unity-ROS communication
```

### 3. Clone and Setup Documentation
```bash
cd frontend_book
npm install
```

### 4. Verify Installation
```bash
# Test Gazebo
gazebo --version

# Test ROS 2
source /opt/ros/humble/setup.bash
ros2 topic list

# Test Docusaurus
npm start
```

## Basic Simulation Workflow

### 1. Launch Gazebo Environment
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Launch a basic simulation world
ros2 launch gazebo_ros empty_world.launch.py
```

### 2. Spawn a Robot Model
```bash
# In another terminal, spawn a robot (example with simple robot)
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file /path/to/robot.urdf -x 0 -y 0 -z 1
```

### 3. Control the Robot
```bash
# Send commands to the robot using ROS 2 topics
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

### 4. View Documentation
```bash
cd frontend_book
npm start
# Open http://localhost:3000 to view the documentation
```

## Running the Digital Twin Simulation

### Physics Simulation (Gazebo)
1. Launch the simulation environment
2. Load your humanoid robot model
3. Configure physics parameters (gravity, friction, etc.)
4. Connect ROS 2 nodes for control and perception

### Visualization (Unity - Optional)
1. Launch Unity project
2. Connect to ROS 2 network
3. Synchronize robot state between Gazebo and Unity
4. Visualize in high-fidelity environment

## Common Issues and Solutions

### Gazebo won't start
- Check if ROS 2 environment is sourced
- Verify Gazebo installation: `which gazebo`
- Check for conflicting processes

### Robot model not loading
- Verify URDF file syntax
- Check file paths are correct
- Ensure all mesh files are accessible

### ROS 2 communication issues
- Verify ROS_DOMAIN_ID matches between nodes
- Check network configuration for multi-machine setups
- Confirm topic names match expected format

## Next Steps

1. Complete Chapter 4: Physics-Based Simulation with Gazebo
2. Move to Chapter 5: High-Fidelity Environments with Unity
3. Explore Chapter 6: Sensor Simulation
4. Practice with provided training scenarios