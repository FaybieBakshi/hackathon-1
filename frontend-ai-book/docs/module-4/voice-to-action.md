---
sidebar_position: 2
title: "Voice-to-Action: OpenAI Whisper for Voice Commands to ROS Actions"
---

# Voice-to-Action: OpenAI Whisper for Voice Commands to ROS Actions

## Overview

This chapter introduces you to processing voice commands using OpenAI Whisper to trigger specific ROS actions on a humanoid robot. You'll learn how to set up voice processing, integrate with ROS 2 action servers, and create a complete voice command pipeline.

## Learning Objectives

By the end of this chapter, you will be able to:

1. Set up OpenAI Whisper for voice command recognition
2. Configure ROS 2 action servers for voice-activated commands
3. Create a complete voice command pipeline from audio input to ROS action execution
4. Implement voice → ROS action conversion in simulation environments
5. Test voice command processing with humanoid robot simulation

## Prerequisites

- Completion of Module 1: The Robotic Nervous System (ROS 2)
- Completion of Module 2: The Digital Twin (Gazebo & Unity)
- Completion of Module 3: The AI-Robot Brain (NVIDIA Isaac)
- Access to OpenAI API for Whisper service
- Basic Python programming knowledge for robotics

## Introduction to Voice Processing for Robotics

Voice processing enables natural language interaction with robots, allowing users to control robotic systems using spoken commands. The process involves:

1. **Audio Capture**: Recording voice commands from users
2. **Speech-to-Text**: Converting audio to text using services like OpenAI Whisper
3. **Command Processing**: Interpreting the text command and determining the appropriate action
4. **Action Execution**: Triggering the corresponding ROS 2 action on the robot

### Key Benefits of Voice Processing

1. **Natural Interaction**: Enables intuitive communication with robots
2. **Hands-Free Operation**: Allows control without physical interfaces
3. **Accessibility**: Provides an alternative control method for users with mobility limitations
4. **Efficiency**: Enables complex commands to be issued quickly

## Setting Up OpenAI Whisper

### API Access Configuration

To use OpenAI Whisper for voice command processing, you'll need:

1. **OpenAI Account**: Sign up for an OpenAI account and obtain an API key
2. **API Key Configuration**: Set up your API key for use in your application
3. **Audio Input Setup**: Configure audio input devices for voice capture

```bash
# Install required Python packages
pip install openai
pip install pyaudio
pip install speechrecognition
```

### Whisper Integration Example

```python
import openai
import pyaudio
import wave
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus

# Configure OpenAI API key
openai.api_key = "YOUR_API_KEY_HERE"

class VoiceToActionNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('voice_to_action_node')

        # Audio configuration
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.record_seconds = 5

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Publisher for processed commands
        self.command_pub = rospy.Publisher('/voice_commands', String, queue_size=10)

        print("Voice-to-Action node initialized")

    def record_audio(self, filename):
        """Record audio from microphone and save to file"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        print("Recording...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)

        print("Finished recording")

        # Stop and close stream
        stream.stop_stream()
        stream.close()

        # Save audio to file
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.audio.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

    def transcribe_audio(self, audio_file):
        """Transcribe audio file using OpenAI Whisper"""
        with open(audio_file, "rb") as audio:
            transcript = openai.Audio.transcribe("whisper-1", audio)
        return transcript.text

    def process_voice_command(self):
        """Main function to process voice command"""
        # Record audio
        audio_filename = "temp_audio.wav"
        self.record_audio(audio_filename)

        # Transcribe audio to text
        command_text = self.transcribe_audio(audio_filename)
        print(f"Transcribed command: {command_text}")

        # Publish command
        self.command_pub.publish(command_text)

        # Clean up temporary file
        import os
        os.remove(audio_filename)

        return command_text

def main():
    node = VoiceToActionNode()

    rate = rospy.Rate(1)  # Process commands at 1 Hz

    while not rospy.is_shutdown():
        try:
            command = node.process_voice_command()
            print(f"Processed command: {command}")
        except Exception as e:
            print(f"Error processing voice command: {e}")

        rate.sleep()

if __name__ == '__main__':
    main()
```

## ROS 2 Action Server Integration

### Action Server Setup

To integrate voice commands with ROS 2 actions, you need to set up action servers that can receive and execute commands:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci  # Example action type

class VoiceCommandActionServer(Node):
    def __init__(self):
        super().__init__('voice_command_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Replace with your specific action type
            'voice_command_action',
            self.execute_callback
        )

        # Subscribe to voice commands
        self.voice_sub = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            10
        )

        self.current_goal = None

    def execute_callback(self, goal_handle):
        """Execute the voice command action"""
        self.get_logger().info('Executing voice command action...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Process the voice command
        command = goal_handle.request.command
        result = self.process_voice_command(command)

        # Send result
        result_msg = Fibonacci.Result()
        result_msg.sequence = result

        goal_handle.succeed()
        return result_msg

    def voice_command_callback(self, msg):
        """Handle incoming voice commands"""
        self.get_logger().info(f'Received voice command: {msg.data}')
        # Process the command and trigger appropriate action
        self.execute_voice_command(msg.data)

    def process_voice_command(self, command):
        """Process the voice command and return result"""
        # Parse and execute the command
        # This would involve NLP processing to determine the specific action
        pass

def main():
    rclpy.init()
    action_server = VoiceCommandActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Voice Command Mapping

Map common voice commands to specific ROS actions:

| Voice Command | ROS Action | Description |
|---------------|------------|-------------|
| "Move forward" | `/navigation/move_forward` | Move robot forward by specified distance |
| "Turn left" | `/navigation/turn_left` | Rotate robot left by specified angle |
| "Raise arm" | `/manipulation/raise_arm` | Move robot arm to raised position |
| "Lower arm" | `/manipulation/lower_arm` | Move robot arm to lowered position |
| "Stop" | `/control/stop` | Stop all robot movement |

## Voice Command Pipeline

### Complete Pipeline Architecture

The complete voice command pipeline consists of:

1. **Audio Input**: Capture voice commands from microphone
2. **Preprocessing**: Clean and format audio data
3. **Speech Recognition**: Convert audio to text using Whisper
4. **Natural Language Processing**: Parse text to extract command intent
5. **Action Mapping**: Map command to specific ROS action
6. **Action Execution**: Execute ROS action on robot
7. **Feedback**: Provide confirmation of action completion

### Pipeline Implementation

```python
import rospy
import openai
import pyaudio
import wave
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VoiceCommandPipeline:
    def __init__(self):
        rospy.init_node('voice_command_pipeline')

        # Audio configuration
        self.audio_config = {
            'chunk': 1024,
            'format': pyaudio.paInt16,
            'channels': 1,
            'rate': 44100,
            'record_seconds': 3
        }

        # Initialize PyAudio
        self.pyaudio_instance = pyaudio.PyAudio()

        # Publishers for different robot actions
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Command mapping
        self.command_mapping = {
            'move forward': self.move_forward,
            'go forward': self.move_forward,
            'move backward': self.move_backward,
            'go backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot
        }

        print("Voice command pipeline initialized")

    def record_audio(self, filename):
        """Record audio from microphone"""
        stream = self.pyaudio_instance.open(
            format=self.audio_config['format'],
            channels=self.audio_config['channels'],
            rate=self.audio_config['rate'],
            input=True,
            frames_per_buffer=self.audio_config['chunk']
        )

        frames = []
        for i in range(0, int(self.audio_config['rate'] /
                             self.audio_config['chunk'] *
                             self.audio_config['record_seconds'])):
            data = stream.read(self.audio_config['chunk'])
            frames.append(data)

        stream.stop_stream()
        stream.close()

        # Save audio
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.audio_config['channels'])
        wf.setsampwidth(self.pyaudio_instance.get_sample_size(self.audio_config['format']))
        wf.setframerate(self.audio_config['rate'])
        wf.writeframes(b''.join(frames))
        wf.close()

    def transcribe_audio(self, audio_file):
        """Transcribe audio using OpenAI Whisper"""
        with open(audio_file, "rb") as audio:
            transcript = openai.Audio.transcribe("whisper-1", audio)
        return transcript.text.lower().strip()

    def parse_command(self, text):
        """Parse voice command and return action"""
        for command, action in self.command_mapping.items():
            if command in text:
                return action
        return None

    def execute_command(self, command_text):
        """Execute the parsed command"""
        action = self.parse_command(command_text)
        if action:
            action()
            rospy.loginfo(f"Executed command: {command_text}")
            return True
        else:
            rospy.logwarn(f"Unknown command: {command_text}")
            return False

    def move_forward(self):
        """Move robot forward"""
        msg = Twist()
        msg.linear.x = 0.5  # Forward speed
        self.cmd_vel_pub.publish(msg)

    def move_backward(self):
        """Move robot backward"""
        msg = Twist()
        msg.linear.x = -0.5  # Backward speed
        self.cmd_vel_pub.publish(msg)

    def turn_left(self):
        """Turn robot left"""
        msg = Twist()
        msg.angular.z = 0.5  # Left turn speed
        self.cmd_vel_pub.publish(msg)

    def turn_right(self):
        """Turn robot right"""
        msg = Twist()
        msg.angular.z = -0.5  # Right turn speed
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        """Stop robot movement"""
        msg = Twist()
        # Zero velocities (stop)
        self.cmd_vel_pub.publish(msg)

    def process_voice_command(self):
        """Main function to process a voice command"""
        audio_file = "temp_command.wav"

        try:
            # Record audio
            self.record_audio(audio_file)

            # Transcribe audio
            command_text = self.transcribe_audio(audio_file)
            rospy.loginfo(f"Recognized: {command_text}")

            # Execute command
            success = self.execute_command(command_text)

            # Clean up
            import os
            os.remove(audio_file)

            return success
        except Exception as e:
            rospy.logerr(f"Error processing voice command: {e}")
            return False

def main():
    pipeline = VoiceCommandPipeline()

    rate = rospy.Rate(0.2)  # Process commands every 5 seconds

    while not rospy.is_shutdown():
        success = pipeline.process_voice_command()
        if success:
            rospy.loginfo("Command processed successfully")
        rate.sleep()

if __name__ == '__main__':
    main()
```

## Practical Exercises

### Exercise 1: Basic Voice Command Setup

1. Set up OpenAI Whisper API access
2. Create a basic voice recording and transcription system
3. Test with simple commands like "move forward" or "stop"
4. Verify that commands are properly recognized and logged

### Exercise 2: ROS Action Integration

1. Create ROS action servers for basic robot movements
2. Integrate the voice processing pipeline with ROS actions
3. Test voice commands triggering robot actions in simulation
4. Validate that the "voice → ROS action conversion" works as expected

### Exercise 3: Advanced Command Processing

1. Expand the command vocabulary to include more complex actions
2. Implement error handling for unrecognized commands
3. Add feedback mechanisms to confirm command execution
4. Test the complete voice command pipeline in simulation

## Performance Optimization

### Audio Quality Considerations

- **Background Noise**: Use noise reduction techniques for clearer audio
- **Microphone Quality**: Use high-quality microphones for better recognition
- **Audio Format**: Optimize audio format and sampling rate for Whisper
- **Processing Speed**: Balance real-time processing with accuracy requirements

### Whisper API Optimization

- **Audio Length**: Keep audio clips to optimal length for processing
- **API Limits**: Consider rate limits and cost optimization
- **Caching**: Cache common command transcriptions to reduce API calls
- **Local Processing**: Consider local speech recognition for common commands

## Troubleshooting Common Issues

### Audio Issues

- **Poor Recognition**: Check microphone quality and background noise
- **No Audio Input**: Verify audio device permissions and configuration
- **Distorted Audio**: Adjust audio input levels and format settings

### API Issues

- **API Key Problems**: Verify API key is correctly configured
- **Rate Limits**: Implement proper rate limiting and error handling
- **Network Issues**: Check internet connectivity for API access

### ROS Integration Issues

- **Action Server Not Responding**: Verify action server is properly initialized
- **Command Mapping Failures**: Check command parsing and mapping logic
- **Communication Issues**: Verify ROS network configuration

## Summary

In this chapter, you've learned how to set up voice processing using OpenAI Whisper and integrate it with ROS 2 action servers. You've explored the complete voice command pipeline from audio input to action execution and implemented the "voice → ROS action conversion" as specified in the deliverable.

The voice processing system provides a natural interface for robot control, enabling users to interact with humanoid robots using spoken commands.

## Next Steps

In the next chapter, you'll explore cognitive planning using Large Language Models to decompose complex commands into executable action sequences, building upon the voice processing foundation you've established here.