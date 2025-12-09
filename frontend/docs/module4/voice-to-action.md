---
sidebar_position: 2
title: "Chapter 2: Voice-to-Action with OpenAI Whisper"
---

# Voice-to-Action with OpenAI Whisper

## Convert Spoken Commands to ROS 2 Actions

Voice-to-action systems enable robots to understand and respond to natural language commands. This chapter focuses on using OpenAI Whisper for speech recognition and converting the recognized text into ROS 2 actions that the robot can execute.

## Overview of Voice-to-Action Pipeline

The voice-to-action system consists of several components:
1. **Audio Capture**: Recording spoken commands from users
2. **Speech Recognition**: Converting audio to text using Whisper
3. **Natural Language Processing**: Understanding the intent behind the text
4. **Action Mapping**: Converting intents to specific ROS 2 actions
5. **Execution**: Performing the requested actions using ROS 2 services/actions

## OpenAI Whisper Integration

### What is Whisper?
Whisper is a robust speech recognition model developed by OpenAI that can handle multiple languages, accents, and background noise. It's particularly suitable for robotics applications due to its accuracy and ability to work with diverse audio conditions.

<Tabs groupId="whisper-models">
<TabItem value="tiny" label="Tiny Model">
```bash
# Smallest model, fastest processing
pip install openai-whisper
# Load tiny model
model = whisper.load_model("tiny")
```
</TabItem>
<TabItem value="base" label="Base Model">
```bash
# Good balance of speed and accuracy
pip install openai-whisper
# Load base model
model = whisper.load_model("base")
```
</TabItem>
<TabItem value="large" label="Large Model">
```bash
# Most accurate, slower processing
pip install openai-whisper
# Load large model
model = whisper.load_model("large")
```
</TabItem>
</Tabs>

### Installation
```bash
pip install openai-whisper
# Additional dependencies for audio processing
pip install pyaudio soundfile
```

### Basic Usage Example
```python
import whisper
import pyaudio
import wave

# Load the Whisper model
model = whisper.load_model("base")

# Transcribe an audio file
result = model.transcribe("command.wav")
print(result["text"])
```

## Audio Capture for Robotics

### Real-time Audio Processing
```python
import pyaudio
import wave
import numpy as np

# Audio parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 1024
WAVE_OUTPUT_FILENAME = "command.wav"

audio = pyaudio.PyAudio()

# Start recording
stream = audio.open(format=FORMAT, channels=CHANNELS,
                   rate=RATE, input=True,
                   frames_per_buffer=CHUNK)

frames = []
print("Listening...")

# Record until silence is detected
for i in range(0, int(RATE / CHUNK * 2)):  # 2 seconds of recording
    data = stream.read(CHUNK)
    frames.append(data)

print("Processing...")

# Stop recording
stream.stop_stream()
stream.close()
audio.terminate()

# Save the recorded audio
wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(audio.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
wf.close()
```

## Natural Language Understanding

### Intent Recognition
```python
def recognize_intent(transcript):
    """Simple intent recognition based on keywords"""
    transcript_lower = transcript.lower()

    if "move" in transcript_lower or "go" in transcript_lower:
        return "navigation"
    elif "pick" in transcript_lower or "grasp" in transcript_lower:
        return "manipulation"
    elif "stop" in transcript_lower or "halt" in transcript_lower:
        return "stop"
    else:
        return "unknown"
```

### Command Extraction
```python
def extract_command_parameters(transcript):
    """Extract parameters from the voice command"""
    # Simple parameter extraction
    params = {}

    # Extract direction if present
    if "forward" in transcript.lower():
        params['direction'] = 'forward'
    elif "backward" in transcript.lower():
        params['direction'] = 'backward'
    elif "left" in transcript.lower():
        params['direction'] = 'left'
    elif "right" in transcript.lower():
        params['direction'] = 'right'

    # Extract distance if specified
    import re
    distance_match = re.search(r'(\d+(?:\.\d+)?)\s*(meters?|m)', transcript.lower())
    if distance_match:
        params['distance'] = float(distance_match.group(1))

    return params
```

## ROS 2 Action Integration

### Creating a Voice Command Node

<Tabs groupId="voice-node-implementation">
<TabItem value="full" label="Complete Implementation">
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatus
import whisper
import threading

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Initialize Whisper model
        self.model = whisper.load_model("base")

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_subscriber = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10)

        # Start audio capture in a separate thread
        self.audio_thread = threading.Thread(target=self.start_audio_capture)
        self.audio_thread.daemon = True
        self.audio_thread.start()

    def start_audio_capture(self):
        """Continuously capture and process audio"""
        while rclpy.ok():
            # Capture audio and process with Whisper
            audio_file = self.capture_audio()
            result = self.model.transcribe(audio_file)
            command_text = result["text"]

            # Publish the recognized command
            msg = String()
            msg.data = command_text
            self.voice_subscriber.publish(msg)

    def voice_callback(self, msg):
        """Process the recognized voice command"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Recognize intent and execute action
        intent = recognize_intent(command)
        params = extract_command_parameters(command)

        if intent == "navigation":
            self.execute_navigation(params)
        elif intent == "stop":
            self.execute_stop()
        # Add more intents as needed
```
</TabItem>
<TabItem value="publisher" label="Publisher Setup">
```python
# Publishers and subscribers setup
self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
self.voice_subscriber = self.create_subscription(
    String, 'voice_commands', self.voice_callback, 10)
```
</TabItem>
<TabItem value="callback" label="Callback Logic">
```python
def voice_callback(self, msg):
    """Process the recognized voice command"""
    command = msg.data
    self.get_logger().info(f'Received command: {command}')

    # Recognize intent and execute action
    intent = recognize_intent(command)
    params = extract_command_parameters(command)

    if intent == "navigation":
        self.execute_navigation(params)
    elif intent == "stop":
        self.execute_stop()
    # Add more intents as needed
```
</TabItem>
</Tabs>

## Advanced Voice Processing

### Wake Word Detection
```python
def detect_wake_word(audio_data, wake_word="robot"):
    """Detect wake word before processing full command"""
    # Use a lightweight model for wake word detection
    # to reduce computational load
    pass
```

### Confidence Scoring
```python
def get_confidence_score(transcript, original_audio):
    """Get confidence score for the transcription"""
    # Whisper doesn't directly provide confidence scores
    # but we can use alternative methods
    pass
```

## Error Handling and Robustness

### Audio Quality Considerations
- Background noise filtering
- Microphone positioning
- Audio format compatibility
- Network latency for cloud processing

### Fallback Mechanisms
```python
def handle_recognition_error():
    """Handle cases where speech recognition fails"""
    # Request repetition
    # Use alternative input method
    # Provide error feedback
    pass
```

## Learning Objectives

After completing this chapter, you will be able to:
- Integrate OpenAI Whisper with ROS 2 for voice recognition
- Design a voice-to-action pipeline for robotics
- Process audio input and convert it to ROS 2 commands
- Handle errors and edge cases in voice processing

## Hands-on Exercise

Implement a simple voice command system that listens for basic navigation commands ("move forward", "turn left", etc.) and converts them to Twist messages for robot navigation.