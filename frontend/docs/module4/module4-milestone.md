---
sidebar_position: 5
title: "Module 4 Milestone Project: Vision-Language-Action Robot Assistant"
---

# Module 4 Milestone Project: Vision-Language-Action Robot Assistant

## Project Overview

In this milestone project, you'll create a complete Vision-Language-Action (VLA) system that allows users to interact with a robot using natural language commands. The system will integrate perception, language understanding, and action execution to create an intelligent robot assistant.

## Learning Objectives

After completing this project, you will be able to:
- Integrate vision, language, and action systems into a cohesive pipeline
- Process natural language commands and map them to robot actions
- Implement cognitive planning for complex task execution
- Create multimodal interfaces for human-robot interaction
- Build end-to-end VLA systems for real-world applications

## Project Requirements

Create a VLA system with:
1. Voice command processing with Whisper for speech recognition
2. Language understanding with LLMs for command interpretation
3. Visual perception for scene understanding and object detection
4. Action planning and execution for robot control
5. Multimodal feedback and interaction capabilities

## Step-by-Step Implementation

### Step 1: Create the VLA System Architecture

Create the file `vla_system/launch/vla_system.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_namespace = LaunchConfiguration('robot_namespace', default='')

    # Package directories
    pkg_vla_system = get_package_share_directory('vla_system')

    # VLA main node
    vla_main_node = Node(
        package='vla_system',
        executable='vla_main',
        name='vla_main',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_name': 'gpt-3.5-turbo',  # or your preferred LLM
            'whisper_model': 'base',
            'confidence_threshold': 0.7,
        }],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/cmd_vel', '/cmd_vel'),
            ('/joint_commands', '/joint_commands'),
        ],
        output='screen'
    )

    # Voice processing node
    voice_processing_node = Node(
        package='vla_system',
        executable='voice_processor',
        name='voice_processor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'whisper_model_size': 'base',
            'language': 'en',
            'temperature': 0.0,
        }],
        output='screen'
    )

    # Vision processing node
    vision_processing_node = Node(
        package='vla_system',
        executable='vision_processor',
        name='vision_processor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'detection_threshold': 0.5,
            'enable_segmentation': True,
        }],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/camera/camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    # Language understanding node
    language_understanding_node = Node(
        package='vla_system',
        executable='language_understanding',
        name='language_understanding',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_name': 'gpt-3.5-turbo',
            'max_tokens': 1000,
            'temperature': 0.1,
        }],
        output='screen'
    )

    # Action planning node
    action_planning_node = Node(
        package='vla_system',
        executable='action_planner',
        name='action_planner',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planning_timeout': 30.0,
            'max_retries': 3,
        }],
        output='screen'
    )

    # State manager node
    state_manager_node = Node(
        package='vla_system',
        executable='state_manager',
        name='state_manager',
        parameters=[{
            'use_sim_time': use_sim_time,
            'state_update_rate': 10.0,
        }],
        output='screen'
    )

    # Feedback and interaction node
    feedback_node = Node(
        package='vla_system',
        executable='feedback_manager',
        name='feedback_manager',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_audio_feedback': True,
            'enable_visual_feedback': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Namespace for robot topics'),

        vla_main_node,
        voice_processing_node,
        vision_processing_node,
        language_understanding_node,
        action_planning_node,
        state_manager_node,
        feedback_node,
    ])
```

### Step 2: Create the Main VLA Controller

Create the file `vla_system/src/vla_main.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from queue import Queue
import openai
from openai import OpenAI
import whisper
import torch
import json
from tf2_ros import TransformListener, Buffer


class VLAMain(Node):
    def __init__(self):
        super().__init__('vla_main')

        # Initialize components
        self.setup_subscribers()
        self.setup_publishers()
        self.initialize_models()
        self.setup_internal_state()

        # Task queue for processing
        self.task_queue = Queue()
        self.response_queue = Queue()

        # Processing thread
        self.processing_thread = threading.Thread(target=self.process_commands)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # Timer for main loop
        self.main_timer = self.create_timer(0.1, self.main_loop)

    def setup_subscribers(self):
        """Setup ROS subscribers"""
        self.voice_sub = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.state_sub = self.create_subscription(
            String,
            'robot_state',
            self.state_callback,
            10
        )

    def setup_publishers(self):
        """Setup ROS publishers"""
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.response_pub = self.create_publisher(String, 'vla_response', 10)
        self.action_pub = self.create_publisher(String, 'robot_actions', 10)

    def initialize_models(self):
        """Initialize AI models"""
        try:
            # Initialize Whisper model for speech recognition
            self.whisper_model = whisper.load_model("base")
            self.get_logger().info("Whisper model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load Whisper model: {e}")
            self.whisper_model = None

        try:
            # Initialize OpenAI client (you would need to set your API key)
            # self.openai_client = OpenAI(api_key='your-api-key-here')
            self.get_logger().info("OpenAI client initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize OpenAI client: {e}")
            # For this example, we'll use a mock implementation

    def setup_internal_state(self):
        """Setup internal state variables"""
        self.current_image = None
        self.robot_state = {}
        self.conversation_history = []
        self.is_processing = False

    def voice_command_callback(self, msg):
        """Handle incoming voice commands"""
        command = msg.data
        self.get_logger().info(f"Received voice command: {command}")

        # Add to task queue for processing
        task = {
            'type': 'command',
            'data': command,
            'timestamp': self.get_clock().now().to_msg()
        }
        self.task_queue.put(task)

    def image_callback(self, msg):
        """Handle incoming camera images"""
        self.current_image = msg
        # Store for use in processing

    def state_callback(self, msg):
        """Handle robot state updates"""
        try:
            self.robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse robot state JSON")

    def process_commands(self):
        """Process commands in a separate thread"""
        while rclpy.ok():
            try:
                if not self.task_queue.empty():
                    task = self.task_queue.get(timeout=1.0)

                    if task['type'] == 'command':
                        response = self.process_natural_language_command(task['data'])
                        response_msg = String()
                        response_msg.data = response
                        self.response_queue.put(response_msg)

                time.sleep(0.01)  # Small delay to prevent busy waiting
            except Exception as e:
                self.get_logger().error(f"Error in processing thread: {e}")

    def process_natural_language_command(self, command):
        """Process natural language command using LLM"""
        try:
            # Create a prompt for the LLM
            prompt = self.create_llm_prompt(command)

            # For this example, we'll simulate the LLM response
            # In a real implementation, you would call the LLM API
            response = self.mock_llm_response(command)

            # Parse the response and execute actions
            self.execute_parsed_actions(response)

            return response

        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            return f"Sorry, I encountered an error processing your command: {str(e)}"

    def create_llm_prompt(self, command):
        """Create a prompt for the LLM with context"""
        # Get current context (image, state, etc.)
        context = self.get_current_context()

        prompt = f"""
        You are a helpful robot assistant. The user has given the following command: "{command}"

        Current robot state: {context['state']}
        Current environment: {context['environment']}

        Please interpret this command and provide a structured response with:
        1. The intent of the command
        2. Specific actions to execute
        3. Any relevant objects or locations

        Respond in JSON format with keys: intent, actions, objects, locations.
        """

        return prompt

    def get_current_context(self):
        """Get current context including state and environment"""
        context = {
            'state': self.robot_state,
            'environment': 'Visual scene analysis would go here',
            'timestamp': self.get_clock().now().to_msg()
        }

        return context

    def mock_llm_response(self, command):
        """Mock LLM response for demonstration"""
        # This is a simplified mock - in reality, you'd call an LLM
        command_lower = command.lower()

        if 'move' in command_lower or 'go' in command_lower:
            if 'forward' in command_lower:
                return json.dumps({
                    'intent': 'navigation',
                    'actions': ['move_forward'],
                    'objects': [],
                    'locations': [],
                    'parameters': {'distance': 1.0}
                })
            elif 'backward' in command_lower:
                return json.dumps({
                    'intent': 'navigation',
                    'actions': ['move_backward'],
                    'objects': [],
                    'locations': [],
                    'parameters': {'distance': 1.0}
                })
            elif 'left' in command_lower or 'right' in command_lower:
                return json.dumps({
                    'intent': 'navigation',
                    'actions': ['turn'],
                    'objects': [],
                    'locations': [],
                    'parameters': {'direction': 'left' if 'left' in command_lower else 'right'}
                })
        elif 'pick' in command_lower or 'grasp' in command_lower or 'take' in command_lower:
            return json.dumps({
                'intent': 'manipulation',
                'actions': ['approach_object', 'grasp_object'],
                'objects': ['object'],
                'locations': [],
                'parameters': {'object_name': 'object'}
            })
        elif 'find' in command_lower or 'look' in command_lower:
            return json.dumps({
                'intent': 'perception',
                'actions': ['scan_environment', 'detect_objects'],
                'objects': ['object'],
                'locations': [],
                'parameters': {'object_name': 'object'}
            })

        return json.dumps({
            'intent': 'unknown',
            'actions': [],
            'objects': [],
            'locations': [],
            'parameters': {}
        })

    def execute_parsed_actions(self, response_json):
        """Execute actions parsed from LLM response"""
        try:
            response = json.loads(response_json)
            intent = response.get('intent', 'unknown')
            actions = response.get('actions', [])

            for action in actions:
                if action == 'move_forward':
                    self.execute_navigation_command('forward', response.get('parameters', {}))
                elif action == 'move_backward':
                    self.execute_navigation_command('backward', response.get('parameters', {}))
                elif action == 'turn':
                    self.execute_navigation_command('turn', response.get('parameters', {}))
                elif action == 'approach_object':
                    self.execute_approach_object(response.get('parameters', {}))
                elif action == 'grasp_object':
                    self.execute_grasp_object(response.get('parameters', {}))
                elif action == 'scan_environment':
                    self.execute_scan_environment()
                elif action == 'detect_objects':
                    self.execute_detect_objects()

            # Publish action for logging
            action_msg = String()
            action_msg.data = f"Executed {intent} with actions: {actions}"
            self.action_pub.publish(action_msg)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing LLM response: {e}")
        except Exception as e:
            self.get_logger().error(f"Error executing actions: {e}")

    def execute_navigation_command(self, direction, parameters):
        """Execute navigation commands"""
        cmd_vel = Twist()

        if direction == 'forward':
            cmd_vel.linear.x = 0.3  # m/s
        elif direction == 'backward':
            cmd_vel.linear.x = -0.3
        elif direction == 'turn':
            cmd_vel.angular.z = 0.5 if parameters.get('direction') == 'left' else -0.5

        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(f"Executing navigation: {direction} with params: {parameters}")

    def execute_approach_object(self, parameters):
        """Execute approach object action"""
        # In a real implementation, this would navigate to the object
        self.get_logger().info(f"Approaching object: {parameters.get('object_name')}")

    def execute_grasp_object(self, parameters):
        """Execute grasp object action"""
        # In a real implementation, this would control manipulator
        self.get_logger().info(f"Attempting to grasp object: {parameters.get('object_name')}")

    def execute_scan_environment(self):
        """Execute environment scanning"""
        self.get_logger().info("Scanning environment...")

    def execute_detect_objects(self):
        """Execute object detection"""
        self.get_logger().info("Detecting objects...")

    def main_loop(self):
        """Main loop for the VLA system"""
        # Publish any responses from the processing thread
        while not self.response_queue.empty():
            response_msg = self.response_queue.get()
            self.response_pub.publish(response_msg)

        # Update internal state periodically
        self.update_internal_state()

    def update_internal_state(self):
        """Update internal state variables"""
        # This would update state based on current sensor data
        pass


def main(args=None):
    rclpy.init(args=args)
    vla_main = VLAMain()

    try:
        rclpy.spin(vla_main)
    except KeyboardInterrupt:
        pass
    finally:
        vla_main.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create the Voice Processor Node

Create the file `vla_system/src/voice_processor.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import pyaudio
import wave
import numpy as np
import whisper
import threading
import queue
import time


class VoiceProcessor(Node):
    def __init__(self):
        super().__init__('voice_processor')

        # Initialize Whisper model
        try:
            self.model = whisper.load_model("base")
            self.get_logger().info("Whisper model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load Whisper model: {e}")
            self.model = None

        # Setup audio parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024
        self.record_seconds = 3

        # Setup publishers and subscribers
        self.voice_pub = self.create_publisher(String, 'voice_commands', 10)
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        # Setup audio processing
        self.audio_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.process_audio)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # Setup continuous listening
        self.listening = True
        self.listening_thread = threading.Thread(target=self.continuous_listening)
        self.listening_thread.daemon = True
        self.listening_thread.start()

        self.get_logger().info("Voice processor initialized")

    def audio_callback(self, msg):
        """Handle audio data from ROS topic"""
        # Add audio data to processing queue
        self.audio_queue.put(msg.data)

    def process_audio(self):
        """Process audio data from queue"""
        while rclpy.ok():
            try:
                if not self.audio_queue.empty():
                    audio_data = self.audio_queue.get(timeout=1.0)

                    # Process the audio with Whisper
                    if self.model:
                        try:
                            # Save audio data temporarily for processing
                            temp_filename = '/tmp/temp_audio.wav'

                            # Write audio data to temporary file
                            wf = wave.open(temp_filename, 'wb')
                            wf.setnchannels(self.channels)
                            wf.setsampwidth(pyaudio.PyAudio().get_sample_size(self.format))
                            wf.setframerate(self.rate)
                            wf.writeframes(audio_data)
                            wf.close()

                            # Transcribe the audio
                            result = self.model.transcribe(temp_filename)
                            text = result["text"].strip()

                            if text:  # Only publish if there's text
                                self.get_logger().info(f"Transcribed: {text}")
                                msg = String()
                                msg.data = text
                                self.voice_pub.publish(msg)

                        except Exception as e:
                            self.get_logger().error(f"Error processing audio: {e}")
                else:
                    time.sleep(0.01)  # Small delay to prevent busy waiting
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in audio processing: {e}")

    def continuous_listening(self):
        """Continuously listen for audio using PyAudio"""
        if not self.model:
            return

        p = pyaudio.PyAudio()

        try:
            stream = p.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )

            self.get_logger().info("Started continuous listening...")

            while self.listening and rclpy.ok():
                frames = []

                # Record for specified duration
                for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
                    data = stream.read(self.chunk)
                    frames.append(data)

                # Convert frames to bytes
                audio_data = b''.join(frames)

                # Add to processing queue
                self.audio_queue.put(audio_data)

        except Exception as e:
            self.get_logger().error(f"Error in continuous listening: {e}")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.listening = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    voice_processor = VoiceProcessor()

    try:
        rclpy.spin(voice_processor)
    except KeyboardInterrupt:
        pass
    finally:
        voice_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Create the State Manager Node

Create the file `vla_system/src/state_manager.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from builtin_interfaces.msg import Time
import json
import threading
import time
from collections import deque


class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        # Setup publishers
        self.state_pub = self.create_publisher(String, 'robot_state', 10)

        # Setup subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Internal state
        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'velocity': {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
            'battery_level': 100.0,
            'last_command': '',
            'command_timestamp': 0,
            'safety_status': 'normal',
            'navigation_status': 'idle',
            'manipulation_status': 'idle',
            'perception_status': 'idle',
            'active_goals': [],
            'completed_goals': [],
            'error_count': 0,
            'last_error': '',
            'system_uptime': 0.0
        }

        # State history for debugging
        self.state_history = deque(maxlen=100)

        # Setup timer for state updates
        self.state_timer = self.create_timer(1.0, self.publish_state)

        # Setup timer for system monitoring
        self.monitor_timer = self.create_timer(0.1, self.monitor_system)

        # Initialize start time
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info("State manager initialized")

    def odom_callback(self, msg):
        """Update position and orientation from odometry"""
        self.robot_state['position']['x'] = msg.pose.pose.position.x
        self.robot_state['position']['y'] = msg.pose.pose.position.y
        self.robot_state['position']['z'] = msg.pose.pose.position.z

        self.robot_state['orientation']['x'] = msg.pose.pose.orientation.x
        self.robot_state['orientation']['y'] = msg.pose.pose.orientation.y
        self.robot_state['orientation']['z'] = msg.pose.pose.orientation.z
        self.robot_state['orientation']['w'] = msg.pose.pose.orientation.w

        self.robot_state['velocity']['linear']['x'] = msg.twist.twist.linear.x
        self.robot_state['velocity']['linear']['y'] = msg.twist.twist.linear.y
        self.robot_state['velocity']['linear']['z'] = msg.twist.twist.linear.z

        self.robot_state['velocity']['angular']['x'] = msg.twist.twist.angular.x
        self.robot_state['velocity']['angular']['y'] = msg.twist.twist.angular.y
        self.robot_state['velocity']['angular']['z'] = msg.twist.twist.angular.z

    def cmd_vel_callback(self, msg):
        """Update last command and status"""
        self.robot_state['last_command'] = f"Lin:({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}), " \
                                         f"Ang:({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})"
        self.robot_state['command_timestamp'] = self.get_clock().now().nanoseconds / 1e9

        # Update navigation status based on command
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.robot_state['navigation_status'] = 'moving'
        else:
            self.robot_state['navigation_status'] = 'idle'

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        if msg.ranges:
            min_range = min([r for r in msg.ranges if r > msg.range_min and r < msg.range_max])
            if min_range < 0.5:  # Dangerously close
                self.robot_state['safety_status'] = 'danger'
            elif min_range < 1.0:  # Close
                self.robot_state['safety_status'] = 'warning'
            else:  # Safe
                self.robot_state['safety_status'] = 'normal'

    def monitor_system(self):
        """Monitor system status and update state"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.robot_state['system_uptime'] = current_time - self.start_time

        # Update battery level simulation (decreasing over time)
        self.robot_state['battery_level'] = max(0.0, self.robot_state['battery_level'] - 0.01)

        # Add current state to history
        self.state_history.append(dict(self.robot_state))

    def publish_state(self):
        """Publish the current robot state as JSON"""
        try:
            state_msg = String()
            state_msg.data = json.dumps(self.robot_state, indent=2)
            self.state_pub.publish(state_msg)

            self.get_logger().debug(f"Published robot state: {self.robot_state['navigation_status']}, "
                                   f"Pos:({self.robot_state['position']['x']:.2f}, {self.robot_state['position']['y']:.2f}), "
                                   f"Vel:({self.robot_state['velocity']['linear']['x']:.2f})")
        except Exception as e:
            self.get_logger().error(f"Error publishing state: {e}")

    def add_goal(self, goal_id, goal_description):
        """Add a new goal to the active goals list"""
        goal = {
            'id': goal_id,
            'description': goal_description,
            'status': 'active',
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.robot_state['active_goals'].append(goal)

    def complete_goal(self, goal_id):
        """Move a goal from active to completed"""
        for i, goal in enumerate(self.robot_state['active_goals']):
            if goal['id'] == goal_id:
                completed_goal = self.robot_state['active_goals'].pop(i)
                completed_goal['status'] = 'completed'
                completed_goal['completion_time'] = self.get_clock().now().nanoseconds / 1e9
                self.robot_state['completed_goals'].append(completed_goal)
                return True
        return False

    def log_error(self, error_message):
        """Log an error and update error count"""
        self.robot_state['error_count'] += 1
        self.robot_state['last_error'] = error_message
        self.get_logger().error(f"Logged error: {error_message} (Total: {self.robot_state['error_count']})")


def main(args=None):
    rclpy.init(args=args)
    state_manager = StateManager()

    try:
        rclpy.spin(state_manager)
    except KeyboardInterrupt:
        pass
    finally:
        state_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Package Configuration

Create the file `vla_system/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vla_system</name>
  <version>0.0.0</version>
  <description>Vision-Language-Action system for Module 4 milestone project</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</end>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <depend>openai</depend>
  <depend>openai-whisper</depend>
  <depend>pyaudio</depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

And create the `vla_system/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(vla_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/vla_main.py
  src/voice_processor.py
  src/state_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Running the VLA System

### To run the complete VLA system:

```bash
# Terminal 1: Start the VLA system
ros2 launch vla_system vla_system.launch.py

# Terminal 2: Simulate voice commands
ros2 topic pub /voice_commands std_msgs/String "data: 'Move forward 1 meter'"

# Terminal 3: Monitor system state
ros2 topic echo /robot_state

# Terminal 4: Monitor VLA responses
ros2 topic echo /vla_response
```

## Testing Your Implementation

### Test 1: Verify VLA Nodes Are Running

```bash
# Check active nodes
ros2 node list | grep vla

# Check topics
ros2 topic list | grep -E "(voice|vla|state)"
```

### Test 2: Test Voice Command Processing

```bash
# Send a test command
ros2 topic pub /voice_commands std_msgs/String "data: 'Please move to the kitchen'"

# Monitor the response
ros2 topic echo /vla_response
```

### Test 3: Check System State Updates

```bash
# Monitor robot state
ros2 topic echo /robot_state
```

## Enhancement Challenges

1. Add multimodal grounding to connect language to visual perception
2. Implement a more sophisticated dialogue manager
3. Add memory capabilities for context-aware interactions
4. Integrate with a real robot or advanced simulation
5. Add safety checks and validation for command execution

## Solution Summary

This milestone project demonstrates how to create a complete Vision-Language-Action system with:
- Speech recognition using OpenAI Whisper
- Language understanding with LLMs
- State management for tracking robot status
- Action planning and execution
- Multimodal interaction capabilities

## Learning Review

After completing this project, reflect on:
- How did the integration of vision, language, and action create emergent behaviors?
- What challenges did you face when connecting different AI modalities?
- How did the system handle ambiguous or complex commands?
- What safety considerations are important for VLA systems?

## Next Steps

This VLA system represents the culmination of all modules in this educational book:
- The ROS 2 foundation for communication
- The simulation environment for testing
- The perception pipeline for understanding
- The VLA integration for intelligent interaction

You can extend this system by:
- Adding more complex manipulation capabilities
- Integrating with cloud-based AI services
- Creating custom user interfaces
- Deploying on real hardware platforms