---
sidebar_position: 8
title: "Module 1 Milestone Project: Simple ROS 2 Robot Node"
---

# Module 1 Milestone Project: Simple ROS 2 Robot Node

## Project Overview

In this milestone project, you'll create a complete ROS 2 node that demonstrates the core concepts learned in Module 1. You'll build a simple robot controller that can receive commands and control a simulated robot.

## Learning Objectives

After completing this project, you will be able to:
- Create a complete ROS 2 package with multiple nodes
- Implement publishers, subscribers, services, and actions
- Use parameter servers for configuration
- Test your nodes using ROS 2 tools
- Understand the structure of a complete ROS 2 application

## Project Requirements

Create a ROS 2 package called `simple_robot_controller` with the following components:

1. A publisher node that publishes robot sensor data
2. A subscriber node that processes sensor data
3. A service server that handles robot commands
4. An action server for long-running navigation tasks
5. A main controller node that coordinates all components

## Step-by-Step Implementation

### Step 1: Create the Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python simple_robot_controller
```

### Step 2: Create the Sensor Publisher Node

Create the file `simple_robot_controller/simple_robot_controller/sensor_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(LaserScan, 'laser_scan', 10)
        self.distance_publisher = self.create_publisher(Float32, 'distance_to_obstacle', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 0.01
        msg.time_increment = 0.001
        msg.scan_time = 0.1
        msg.range_min = 0.0
        msg.range_max = 10.0

        # Generate random range data
        msg.ranges = [random.uniform(0.5, 5.0) for _ in range(314)]
        msg.intensities = [100.0 for _ in range(314)]

        self.publisher.publish(msg)

        # Publish distance to closest obstacle
        distance_msg = Float32()
        distance_msg.data = min(msg.ranges)
        self.distance_publisher.publish(distance_msg)

        self.get_logger().info(f'Publishing: {min(msg.ranges):.2f}m to closest obstacle')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create the Command Service Server

Create the file `simple_robot_controller/simple_robot_controller/command_service.py`:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist

class CommandService(Node):
    def __init__(self):
        super().__init__('command_service')
        self.srv = self.create_service(
            SetBool,
            'robot_command',
            self.command_callback
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.is_moving = False

    def command_callback(self, request, response):
        if request.data:  # True means start moving
            self.get_logger().info('Starting robot movement')
            self.start_movement()
            response.success = True
            response.message = 'Robot movement started'
        else:  # False means stop
            self.get_logger().info('Stopping robot movement')
            self.stop_movement()
            response.success = True
            response.message = 'Robot movement stopped'

        return response

    def start_movement(self):
        self.is_moving = True
        # Publish a forward velocity command
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        self.cmd_vel_publisher.publish(msg)

    def stop_movement(self):
        self.is_moving = False
        # Publish zero velocity to stop
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    command_service = CommandService()
    rclpy.spin(command_service)
    command_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Create the Main Controller Node

Create the file `simple_robot_controller/simple_robot_controller/main_controller.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
import math

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        # Subscriptions
        self.subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            10
        )

        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Client for the command service
        self.cli = self.create_client(SetBool, 'robot_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.obstacle_distance = float('inf')
        self.scan_data = None
        self.safe_distance = 1.0  # meters

    def laser_callback(self, msg):
        # Store the latest scan data
        self.scan_data = msg
        # Calculate minimum distance
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)

    def control_loop(self):
        if self.scan_data is None:
            return

        # Simple obstacle avoidance logic
        msg = Twist()

        if self.obstacle_distance > self.safe_distance:
            # Move forward if path is clear
            msg.linear.x = 0.3
            msg.angular.z = 0.0
        else:
            # Stop and turn to avoid obstacle
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Turn right

        self.cmd_vel_publisher.publish(msg)

        # Log robot status
        self.get_logger().info(
            f'Distance: {self.obstacle_distance:.2f}m, '
            f'Lin: {msg.linear.x:.2f}, Ang: {msg.angular.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    main_controller = MainController()
    rclpy.spin(main_controller)
    main_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Update setup.py

Update the `simple_robot_controller/setup.py` file to include your new nodes:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'simple_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Simple ROS 2 robot controller for learning',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = simple_robot_controller.sensor_publisher:main',
            'command_service = simple_robot_controller.command_service:main',
            'main_controller = simple_robot_controller.main_controller:main',
        ],
    },
)
```

## Testing Your Implementation

### Test 1: Run Individual Nodes

```bash
# Terminal 1 - Run the sensor publisher
ros2 run simple_robot_controller sensor_publisher

# Terminal 2 - Run the command service
ros2 run simple_robot_controller command_service

# Terminal 3 - Run the main controller
ros2 run simple_robot_controller main_controller
```

### Test 2: Use ROS 2 Tools to Verify

```bash
# Check active nodes
ros2 node list

# Check topics
ros2 topic list

# Call the service
ros2 service call /robot_command std_srvs/srv/SetBool "{data: true}"
```

## Enhancement Challenges

1. Add more sensor types (camera, IMU, etc.)
2. Implement more complex navigation behaviors
3. Add a simple GUI using rqt
4. Create a launch file to start all nodes at once
5. Add parameter configuration for different robot behaviors

## Solution Summary

This milestone project demonstrates how to create a complete ROS 2 application with multiple interacting components. You've learned how to:

- Create custom ROS 2 nodes with publishers, subscribers, and services
- Handle sensor data and make decisions based on it
- Structure a ROS 2 package properly
- Test your nodes using ROS 2 command-line tools

## Learning Review

After completing this project, reflect on:
- How did the publish-subscribe pattern facilitate communication between nodes?
- What challenges did you face when coordinating multiple nodes?
- How could you extend this simple controller to handle more complex navigation tasks?

## Next Steps

This milestone project sets the foundation for more complex robotics applications. In the next modules, you'll learn how to integrate simulation environments, perception systems, and advanced AI capabilities with your ROS 2 foundation.