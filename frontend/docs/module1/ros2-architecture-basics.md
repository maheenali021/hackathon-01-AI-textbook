---
sidebar_position: 3
title: "Chapter 3: ROS 2 Architecture Basics"
---

# ROS 2 Architecture Basics

## Introduction to ROS 2 Architecture

The Robot Operating System 2 (ROS 2) provides a flexible framework for writing robot software. Understanding its architecture is crucial for developing effective robotic applications.

## Core Architecture Concepts

### Client Library Implementations

ROS 2 supports multiple client libraries that provide the same functionality in different programming languages:

- **rclcpp**: C++ client library
- **rclpy**: Python client library
- **rcl**: Common client library implementation
- **rclc**: C client library

### Middleware Layer

ROS 2 uses a Data Distribution Service (DDS) middleware layer that provides:

- **Discovery**: Automatic discovery of nodes on the network
- **Transport**: Reliable message transport between nodes
- **Quality of Service (QoS)**: Configurable communication policies

```python
import rclpy
from rclpy.node import Node

class ArchitectureDemoNode(Node):
    def __init__(self):
        super().__init__('architecture_demo_node')

        # Example of QoS configuration
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(String, 'demo_topic', qos_profile)
```

### Nodes and Processes

In ROS 2, nodes are processes that perform computation. Each node:

- Runs independently of other nodes
- Communicates with other nodes through messages
- Can be written in different programming languages
- Can run on different machines

## Communication Patterns

### Topics and Messages

Topics enable publish-subscribe communication between nodes:

```python
# Publisher example
publisher = self.create_publisher(String, 'topic_name', 10)

# Subscriber example
subscriber = self.create_subscription(
    String,
    'topic_name',
    self.listener_callback,
    10
)
```

### Services and Actions

Services provide request-response communication, while actions handle long-running tasks with feedback:

```python
# Service server
from example_interfaces.srv import AddTwoInts

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    return response

service = self.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
```

## Quality of Service (QoS) Settings

QoS settings allow fine-tuning of communication behavior:

:::note
QoS settings are crucial for real-time and safety-critical applications where message delivery guarantees are important.
:::

### Reliability Policy
- **RELIABLE**: All messages are delivered (with retries)
- **BEST_EFFORT**: Messages are delivered without guarantees

### History Policy
- **KEEP_LAST**: Store the last N messages
- **KEEP_ALL**: Store all messages (limited by resource availability)

## Security Features

ROS 2 includes security features for production environments:

- **Authentication**: Verify node identity
- **Encryption**: Encrypt message content
- **Access Control**: Control which nodes can communicate

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the layered architecture of ROS 2
- Configure Quality of Service settings appropriately
- Understand the role of middleware in ROS 2
- Implement basic communication patterns

## Hands-on Exercise

Create a simple ROS 2 node that demonstrates different QoS profiles and observe how message delivery changes under various conditions.

:::tip
Use `ros2 topic info` and `ros2 node info` commands to inspect the QoS settings of running topics and nodes.
:::