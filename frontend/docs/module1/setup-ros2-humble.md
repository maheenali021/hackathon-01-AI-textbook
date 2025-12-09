---
sidebar_position: 2
title: "Chapter 2: Setting Up ROS 2 Humble (Ubuntu + Jetson)"
---

# Setting Up ROS 2 Humble (Ubuntu + Jetson)

## Introduction

ROS 2 Humble Hawksbill is a long-term support (LTS) release of the Robot Operating System 2, providing stability and long-term maintenance for robotics projects. This chapter guides you through setting up ROS 2 Humble on Ubuntu and NVIDIA Jetson platforms.

## Interactive Setup Assistant

<Tabs>
<TabItem value="ubuntu" label="Ubuntu Setup">
This guide walks you through setting up ROS 2 Humble on Ubuntu 22.04.
</TabItem>
<TabItem value="jetson" label="Jetson Setup">
This guide walks you through setting up ROS 2 Humble on NVIDIA Jetson platforms.
</TabItem>
</Tabs>

## System Requirements

### Ubuntu Setup
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- 64-bit PC
- Minimum 8GB RAM (16GB recommended)
- At least 5GB free disk space

### Jetson Setup
- NVIDIA Jetson AGX Xavier, Jetson Xavier NX, or Jetson Orin
- JetPack 5.0 or later
- Sufficient power supply for the development board

## Installation on Ubuntu

### Step 1: Set Locale
```bash
locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Setup Sources
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Step 3: Add Repository
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 4: Install ROS 2
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### Step 5: Environment Setup
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Installation on NVIDIA Jetson

### Step 1: Update JetPack
Ensure your Jetson board is running JetPack 5.0 or later.

### Step 2: Install ROS 2 via Debian Packages
```bash
sudo apt update
sudo apt install ros-humble-ros-base
```

### Step 3: Install Additional Packages
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

## Setting Up a Workspace (colcon)

### Step 1: Create Workspace Directory
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Step 2: Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

### Step 3: Build the Workspace
```bash
colcon build --symlink-install
```

### Step 4: Source the Workspace
```bash
source ~/ros2_ws/install/setup.bash
```

## Environment Configuration

### Add to ~/.bashrc
```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Your ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# ROS_DOMAIN_ID (optional, for network isolation)
export ROS_DOMAIN_ID=42
```

## Verification

Test your installation by running a simple demo:

<details>
<summary>Click here to see the talker-listener demo in action</summary>

<Tabs groupId="setup-platform">
<TabItem value="terminal1" label="Terminal 1 (Talker)">
```bash
# Terminal 1 - This will publish messages
ros2 run demo_nodes_cpp talker
```
</TabItem>
<TabItem value="terminal2" label="Terminal 2 (Listener)">
```bash
# Terminal 2 - This will subscribe to messages
ros2 run demo_nodes_py listener
```
</TabItem>
</Tabs>

When you run both commands, you should see the talker publishing messages and the listener receiving them, confirming that your ROS 2 installation is working correctly.

</details>

## Learning Objectives

After completing this chapter, you will be able to:
- Install ROS 2 Humble on Ubuntu and Jetson platforms
- Set up a colcon workspace for development
- Configure the ROS 2 environment
- Verify the installation with basic tests

## Hands-on Exercise

Set up ROS 2 Humble on your system and run the talker-listener demo to verify the installation works correctly.