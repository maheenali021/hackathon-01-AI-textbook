---
sidebar_position: 2
title: "Chapter 2: Gazebo Basics"
---

# Gazebo Basics

## Introduction to Physics Simulation

Gazebo is a powerful robotics simulator that provides accurate physics simulation, high-quality graphics, and convenient programmatic interfaces. It's widely used in robotics research and development for testing algorithms, robot designs, and scenarios without the need for physical hardware.

## Core Concepts

### Physics Engine
Gazebo uses the Open Dynamics Engine (ODE), Bullet, or DART physics engine to simulate realistic physics interactions including:
- Collision detection
- Dynamics simulation
- Contact forces
- Friction and damping

### Sensors Simulation
Gazebo provides simulation of various sensors:
- Camera sensors (monocular, stereo, RGB-D)
- LiDAR and laser range finders
- IMU and GPS sensors
- Force/torque sensors
- Contact sensors

### Models and Worlds
- **Models**: Represent robots, objects, and other entities in the simulation
- **Worlds**: Define the environment, including lighting, physics properties, and initial conditions

## Installation and Setup

### Installing Gazebo Garden
```bash
# For Ubuntu 22.04
sudo apt update
sudo apt install gazebo
```

### Basic Commands
```bash
# Launch Gazebo GUI
gazebo

# Launch with a specific world file
gazebo my_world.world

# Launch without GUI (headless)
gz sim -s
```

## Creating a Simple World

### World File Structure
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Working with Models

### Model Structure
A robot model in Gazebo typically includes:
- **Links**: Rigid bodies with mass, geometry, and visual properties
- **Joints**: Connections between links (revolute, prismatic, fixed, etc.)
- **Sensors**: Simulated sensors attached to links
- **Plugins**: Custom code that controls the robot's behavior

### URDF Integration
Gazebo works seamlessly with URDF (Unified Robot Description Format) files through the libgazebo_ros_xacro package.

## ROS 2 Integration

### Launching with ROS 2
```xml
<!-- launch file example -->
<launch>
  <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py"/>

  <node name="spawn_entity" pkg="gazebo_ros" type="spawn_entity.py"
        args="-topic robot_description -entity my_robot"/>
</launch>
```

## Best Practices

### Performance Optimization
- Use simplified collision geometries when possible
- Limit the number of active sensors in simulation
- Adjust physics update rates based on requirements
- Use LOD (Level of Detail) models when appropriate

### Accuracy Considerations
- Calibrate simulated sensors to match real hardware
- Validate physics parameters against real-world measurements
- Account for simulation vs. reality differences in algorithms

## Learning Objectives

After completing this chapter, you will be able to:
- Install and launch Gazebo
- Understand the core concepts of physics simulation
- Create basic world files and models
- Integrate Gazebo with ROS 2

## Hands-on Exercise

Create a simple world file with a ground plane and a few objects of different shapes. Launch Gazebo with your world and observe the physics simulation.