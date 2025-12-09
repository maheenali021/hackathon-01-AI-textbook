---
sidebar_position: 6
title: "Chapter 6: Bridging Python Agents with ROS 2"
---

# Bridging Python Agents with ROS 2

## Introduction to Python Agent Integration

Modern robotics increasingly involves AI agents that can reason, plan, and make decisions. This chapter covers how to bridge Python-based AI agents with ROS 2 for intelligent robot control.

## AI Agent Architecture with ROS 2

### Agent-Environment Interface

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import numpy as np
import json

class AgentBridgeNode(Node):
    def __init__(self):
        super().__init__('agent_bridge_node')

        # Publishers for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers for sensor data
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Agent state
        self.sensors = {
            'laser': None,
            'image': None,
            'position': None
        }

        # Timer for agent execution
        self.agent_timer = self.create_timer(0.1, self.agent_step)

        self.get_logger().info('Agent bridge initialized')

    def laser_callback(self, msg):
        """Process laser scan data for the agent"""
        self.sensors['laser'] = {
            'ranges': list(msg.ranges),
            'min_range': msg.range_min,
            'max_range': msg.range_max
        }

    def image_callback(self, msg):
        """Process image data for the agent"""
        # Convert ROS Image to numpy array for processing
        # (simplified - actual conversion depends on encoding)
        self.sensors['image'] = {
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding
        }

    def agent_step(self):
        """Execute one step of the AI agent"""
        if all(self.sensors.values()):  # All sensors have data
            # Get action from agent
            action = self.run_agent_policy(self.sensors)

            # Execute action
            self.execute_action(action)

    def run_agent_policy(self, observation):
        """Placeholder for AI agent policy - in real implementation, this would use ML models"""
        # Simple obstacle avoidance policy
        if self.sensors['laser']:
            min_distance = min(self.sensors['laser']['ranges'])
            if min_distance < 0.5:  # Obstacle too close
                return {'linear': 0.0, 'angular': 0.5}  # Turn
            else:
                return {'linear': 0.3, 'angular': 0.0}  # Move forward
        return {'linear': 0.0, 'angular': 0.0}  # Stop

    def execute_action(self, action):
        """Execute the action returned by the agent"""
        cmd = Twist()
        cmd.linear.x = action['linear']
        cmd.angular.z = action['angular']
        self.cmd_vel_pub.publish(cmd)
```

## Integration with Popular AI Libraries

### Using OpenAI Gym for Training

```python
import gym
from gym import spaces
import numpy as np

class ROS2GymEnvironment(gym.Env):
    """Gym environment wrapper for ROS 2 robot control"""

    def __init__(self, node):
        super(ROS2GymEnvironment, self).__init__()

        # Define action and observation spaces
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(360,),  # 360 laser ranges
            dtype=np.float32
        )

        self.node = node
        self.current_obs = None

    def step(self, action):
        # Execute action in ROS 2
        cmd = Twist()
        cmd.linear.x = action[0]
        cmd.angular.z = action[1]
        self.node.cmd_vel_pub.publish(cmd)

        # Wait for next observation
        self.current_obs = self.node.sensors['laser']['ranges'] if self.node.sensors['laser'] else [0]*360

        # Calculate reward (simple example)
        reward = self.calculate_reward(action)
        done = self.is_done()
        info = {}

        return np.array(self.current_obs), reward, done, info

    def reset(self):
        # Reset environment
        return np.array(self.current_obs)

    def calculate_reward(self, action):
        """Calculate reward based on action and sensor data"""
        # Simple reward: positive for moving forward when safe, negative for collisions
        if self.current_obs and min(self.current_obs) < 0.3:  # Collision risk
            return -10.0 if action[0] > 0 else 0.0  # Penalty for moving forward near obstacles
        return action[0] * 0.1  # Small reward for forward movement

    def is_done(self):
        """Check if episode is done"""
        if self.current_obs and min(self.current_obs) < 0.2:  # Collision
            return True
        return False
```

### Integration with Transformers

```python
from transformers import pipeline
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NLPRobotController(Node):
    def __init__(self):
        super().__init__('nlp_robot_controller')

        # Initialize NLP pipeline
        self.nlp_pipeline = pipeline("text-classification",
                                   model="distilbert-base-uncased-finetuned-sst-2-english")

        # Command recognition
        self.command_sub = self.create_subscription(
            String, 'voice_commands', self.command_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Define command mappings
        self.command_map = {
            'forward': {'linear': 0.5, 'angular': 0.0},
            'backward': {'linear': -0.5, 'angular': 0.0},
            'left': {'linear': 0.0, 'angular': 0.5},
            'right': {'linear': 0.0, 'angular': -0.5},
            'stop': {'linear': 0.0, 'angular': 0.0}
        }

    def command_callback(self, msg):
        """Process natural language commands"""
        command_text = msg.data.lower()

        # Simple keyword matching (in practice, use more sophisticated NLP)
        recognized_command = self.recognize_command(command_text)

        if recognized_command in self.command_map:
            action = self.command_map[recognized_command]
            self.execute_command(action)
            self.get_logger().info(f'Executed command: {recognized_command}')
        else:
            self.get_logger().info(f'Unknown command: {command_text}')

    def recognize_command(self, text):
        """Simple command recognition"""
        if 'forward' in text or 'go' in text or 'move' in text:
            return 'forward'
        elif 'backward' in text or 'back' in text:
            return 'backward'
        elif 'left' in text:
            return 'left'
        elif 'right' in text:
            return 'right'
        elif 'stop' in text or 'halt' in text:
            return 'stop'
        return 'unknown'
```

## Advanced Agent Patterns

### Hierarchical Agent Architecture

```python
class HierarchicalAgent(Node):
    def __init__(self):
        super().__init__('hierarchical_agent')

        # High-level planner
        self.planner = HighLevelPlanner()

        # Low-level controller
        self.controller = LowLevelController()

        # State machine
        self.current_state = 'IDLE'
        self.goal = None

        self.state_timer = self.create_timer(0.1, self.state_machine)

    def state_machine(self):
        """Main state machine for hierarchical agent"""
        if self.current_state == 'IDLE':
            new_goal = self.planner.get_next_goal()
            if new_goal:
                self.goal = new_goal
                self.current_state = 'NAVIGATING'

        elif self.current_state == 'NAVIGATING':
            status = self.controller.navigate_to(self.goal)
            if status == 'SUCCESS':
                self.current_state = 'IDLE'
            elif status == 'FAILED':
                self.current_state = 'IDLE'  # Retry or plan alternative
```

### Multi-Agent Coordination

```python
from std_msgs.msg import String
import json

class MultiAgentCoordinator(Node):
    def __init__(self):
        super().__init__('multi_agent_coordinator')

        # Communication with other agents
        self.agent_status_sub = self.create_subscription(
            String, 'agent_status', self.agent_status_callback, 10)
        self.task_assignment_pub = self.create_publisher(
            String, 'task_assignment', 10)

        # Agent registry
        self.agents = {}
        self.tasks = []

    def agent_status_callback(self, msg):
        """Process status from other agents"""
        status = json.loads(msg.data)
        agent_id = status['agent_id']
        self.agents[agent_id] = status

    def assign_task(self, task):
        """Assign task to most suitable agent"""
        best_agent = self.find_best_agent_for_task(task)
        if best_agent:
            assignment = {
                'agent_id': best_agent,
                'task': task
            }
            assignment_msg = String()
            assignment_msg.data = json.dumps(assignment)
            self.task_assignment_pub.publish(assignment_msg)
```

## Performance Considerations

### Threading and Concurrency

```python
import threading
from concurrent.futures import ThreadPoolExecutor

class ThreadedAgent(Node):
    def __init__(self):
        super().__init__('threaded_agent')

        # Use thread pool for AI computations
        self.executor = ThreadPoolExecutor(max_workers=2)

        # Thread-safe data structures
        self.observation_lock = threading.Lock()
        self.current_observation = None

    def async_ai_processing(self, observation):
        """Run AI processing in separate thread"""
        future = self.executor.submit(self.ai_model.predict, observation)
        return future
```

## Learning Objectives

After completing this chapter, you will be able to:
- Bridge Python AI agents with ROS 2 systems
- Integrate popular AI libraries with ROS 2
- Implement hierarchical agent architectures
- Design multi-agent coordination systems
- Consider performance implications of AI integration

## Hands-on Exercise

Create a simple AI agent that uses sensor data to navigate a room and avoid obstacles. Implement both a basic rule-based agent and explore using a machine learning model for decision making.

:::tip
Use `rclpy.qos.QoSProfile` with appropriate settings to balance performance and reliability when bridging AI agents with ROS 2.
:::

:::warning
Be mindful of computational resources when running AI models on robots. Consider the trade-off between intelligence and real-time performance.
:::