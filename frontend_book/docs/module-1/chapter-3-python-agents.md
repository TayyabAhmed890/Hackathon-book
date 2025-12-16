---
title: Chapter 3 - Python Agents & Robot Description
sidebar_label: Chapter 3 - Python Agents & Robot Description
---

# Chapter 3: Python Agents & Robot Description

## Learning Objectives

After completing this chapter, you will be able to:
- Bridge AI agents to ROS using the rclpy Python client library
- Create and structure basic Python ROS 2 nodes with examples
- Understand the conceptual introduction to URDF for humanoids
- Explain the concepts of links, joints, and sensors in robot description

## Bridging AI Agents to ROS using rclpy

The Python client library for ROS 2, known as rclpy, provides the interface between Python applications and the ROS 2 system. This enables AI agents written in Python to interact with robot systems.

### Setting Up rclpy
To use rclpy in your Python applications:
```python
import rclpy
from rclpy.node import Node
```

### Basic Node Structure
Every Python ROS 2 node inherits from the Node class and follows a standard structure:
```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)
```

### Integration with AI Agents
AI agents can use rclpy to:
- Subscribe to sensor data from the robot
- Publish commands to control the robot
- Use services for specific robot capabilities
- Access parameters for configuration

## Basic Python ROS 2 Node Structure

### Node Lifecycle
A Python ROS 2 node typically follows this lifecycle:
1. **Initialization**: Create the node and its communication interfaces
2. **Execution**: Run the main logic, processing callbacks and performing computations
3. **Shutdown**: Clean up resources and gracefully terminate

### Communication Interfaces
A node can create multiple types of communication interfaces:
- Publishers for sending messages
- Subscribers for receiving messages
- Services for providing functionality
- Clients for requesting functionality
- Action servers and clients for goal-oriented tasks

### Example Complete Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)

        # Create timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def control_loop(self):
        # AI logic would go here
        msg = String()
        msg.data = 'Control command'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conceptual Introduction to URDF for Humanoids

URDF (Unified Robot Description Format) is an XML-based format for representing robot models in ROS. For humanoid robots, URDF describes the physical structure and kinematic properties.

### URDF Structure
A URDF file contains:
- **Links**: Rigid parts of the robot (e.g., torso, arms, legs)
- **Joints**: Connections between links that define how they can move relative to each other
- **Materials**: Visual properties for rendering
- **Gazebo plugins**: Simulation-specific configurations

### Example URDF Structure
```xml
<robot name="humanoid_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links, Joints, and Sensors Overview

### Links
Links represent rigid bodies in the robot structure:
- Each link has physical properties like mass, inertia, and geometry
- Links can have visual and collision representations
- Examples: robot base, torso, upper arm, lower leg

### Joints
Joints connect links and define their relative motion:
- **Fixed joints**: No relative motion between connected links
- **Revolute joints**: Single axis of rotation (like a hinge)
- **Prismatic joints**: Single axis of translation (like a sliding joint)
- **Continuous joints**: Unbounded rotation around a single axis
- **Floating joints**: 6 degrees of freedom (rarely used)

### Sensors
Sensors in URDF describe where sensor data originates:
- Camera sensors for vision
- IMU sensors for orientation and acceleration
- Force/torque sensors for contact forces
- Joint state sensors for position, velocity, and effort

## Summary

In this chapter, we've covered:
- How to bridge AI agents to ROS using rclpy
- The basic structure of Python ROS 2 nodes with practical examples
- The conceptual introduction to URDF for describing humanoid robots
- The key concepts of links, joints, and sensors in robot description

## Review Questions

1. What is rclpy and how does it enable AI agents to interact with ROS 2?
2. Describe the basic structure of a Python ROS 2 node.
3. What are the main components of a URDF file?
4. Explain the difference between links and joints in robot description.
5. How might an AI agent use publishers and subscribers to control a humanoid robot?

## Next Steps

[Return to Chapter 2: ROS 2 Communication](./chapter-2-ros2-communication)

Congratulations! You have completed Module 1: The Robotic Nervous System (ROS 2). You now have a foundational understanding of ROS 2 concepts, communication patterns, and how to integrate AI agents with ROS using Python.