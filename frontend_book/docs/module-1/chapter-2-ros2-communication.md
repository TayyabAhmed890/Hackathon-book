---
title: Chapter 2 - ROS 2 Communication
sidebar_label: Chapter 2 - ROS 2 Communication
---

# Chapter 2: ROS 2 Communication

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the node execution model in ROS 2
- Explain the publish/subscribe communication pattern using topics
- Describe the request/response communication pattern using services
- Apply these communication patterns to conceptual humanoid control examples

## Nodes and Execution Model

In ROS 2, a node is the fundamental unit of computation. Nodes are processes that perform specific tasks and communicate with other nodes to achieve complex robot behaviors. The execution model in ROS 2 is based on:

### Node Lifecycle
- **Initialization**: Nodes are created and initialized with specific functionality
- **Execution**: Nodes run their computational tasks and communicate with other nodes
- **Shutdown**: Nodes are properly cleaned up when no longer needed

### Node Implementation
Nodes can be implemented in multiple programming languages (C++, Python, etc.) and can run on the same or different machines. Each node has:
- A unique name within the ROS 2 domain
- The ability to create publishers, subscribers, services, and clients
- Access to parameters for configuration
- The ability to handle callbacks for incoming messages

## Topics and Publish/Subscribe

The publish/subscribe pattern is the primary communication mechanism for streaming data in ROS 2.

### Publishers and Subscribers
- **Publishers** send messages to topics without knowing which subscribers will receive them
- **Subscribers** receive messages from topics without knowing which publishers sent them
- This creates a decoupled communication pattern where publishers and subscribers don't need to know about each other

### Topic Communication Characteristics
- **Unidirectional**: Data flows from publisher to subscriber
- **Asynchronous**: Publishers and subscribers can operate at different rates
- **Many-to-many**: Multiple publishers can send to the same topic, and multiple subscribers can receive from the same topic
- **Typed**: All messages on a topic must be of the same message type

### Example Message Flow
```python
# Publisher example
publisher = node.create_publisher(String, 'robot_status', 10)
msg = String()
msg.data = 'Robot is operational'
publisher.publish(msg)
```

## Services (Request/Response)

The service/client pattern provides synchronous request/response communication in ROS 2.

### Services and Clients
- **Services** provide specific functionality that can be requested by clients
- **Clients** make requests to services and wait for responses
- This creates a direct communication pattern with guaranteed responses

### Service Communication Characteristics
- **Bidirectional**: Request goes from client to service, response goes from service to client
- **Synchronous**: Client waits for the service to complete before continuing
- **One-to-one**: Each request is handled by one service instance
- **Typed**: Both request and response have defined message types

### Example Service Flow
```python
# Service example
def handle_add_two_ints(request, response):
    response.sum = request.a + request.b
    return response

service = node.create_service(AddTwoInts, 'add_two_ints', handle_add_two_ints)
```

## Conceptual Humanoid Control Examples

Let's consider how these communication patterns apply to humanoid robots:

### Using Topics for Sensor Data
- Joint position sensors publish to `/joint_states` topic
- Multiple subscribers (control algorithms, monitoring systems, visualization tools) receive this data
- This allows multiple systems to access sensor data simultaneously without interfering with each other

### Using Services for Actions
- A walking controller service receives requests to move the robot
- A vision service processes images and returns object detection results
- A path planning service receives goal positions and returns navigation plans

### Combined Communication for Complex Behaviors
- High-level behaviors coordinate using a combination of topics and services
- For example, a walking behavior might:
  - Subscribe to sensor data topics to monitor robot state
  - Use services to request specific joint movements
  - Publish to command topics to send control signals to actuators

## Summary

In this chapter, we've explored the communication patterns in ROS 2:
- The node execution model and lifecycle
- The publish/subscribe pattern using topics
- The request/response pattern using services
- How these patterns apply to humanoid robot control

## Review Questions

1. What is the difference between the publish/subscribe and request/response communication patterns?
2. Explain the node lifecycle in ROS 2.
3. How do topics enable decoupled communication in robot systems?
4. What are the advantages and disadvantages of using services versus topics?
5. Provide an example of how topics and services might be used together in a humanoid robot control system.

## Next Steps

[Continue to Chapter 3: Python Agents & Robot Description](./chapter-3-python-agents)

[Return to Chapter 1: ROS 2 Fundamentals](./chapter-1-ros2-fundamentals)