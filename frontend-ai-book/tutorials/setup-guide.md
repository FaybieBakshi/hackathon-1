---
sidebar_position: 5
title: "VLA Module Setup Guide"
---

# VLA Module Setup Guide

## Overview

This guide walks you through setting up the Vision-Language-Action (VLA) module for voice-controlled humanoid robotics. This setup builds upon the foundations of Modules 1-3 (ROS 2, simulation, and AI-robot brain) and adds voice processing and LLM integration.

## Prerequisites

Before starting this module, ensure you have completed:

- **Module 1**: The Robotic Nervous System (ROS 2) - Understanding of ROS 2 concepts
- **Module 2**: The Digital Twin (Gazebo & Unity) - Simulation environment knowledge
- **Module 3**: The AI-Robot Brain (NVIDIA Isaac) - Perception and navigation understanding
- **System Requirements**:
  - Python 3.11+
  - ROS 2 Humble Hawksbill or newer
  - Access to OpenAI API (Whisper and GPT-4) or equivalent LLM service
  - Microphone for voice input
  - Modern web browser for Docusaurus interface

## Software Installation

### 1. OpenAI API Access

First, you'll need to set up access to OpenAI services:

```bash
# Install OpenAI Python package
pip install openai

# Set up your API key (replace with your actual key)
export OPENAI_API_KEY="sk-..."
```

### 2. Audio Processing Dependencies

Install required audio processing packages:

```bash
# Install PyAudio for audio recording
pip install pyaudio

# Install SpeechRecognition for additional audio processing
pip install SpeechRecognition

# Install additional audio processing libraries
pip install sounddevice numpy
```

### 3. ROS 2 Packages for VLA

Install or verify these ROS 2 packages:

```bash
# Install standard ROS 2 packages
sudo apt update
sudo apt install ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs
sudo apt install ros-humble-actionlib ros-humble-tf2-ros

# Install navigation packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install manipulation packages (if available)
sudo apt install ros-humble-moveit
```

## Configuration

### 1. OpenAI Configuration

Create a configuration file for your OpenAI settings:

```python
# config/openai_config.py
OPENAI_API_KEY = "YOUR_API_KEY_HERE"
WHISPER_MODEL = "whisper-1"
GPT_MODEL = "gpt-4"
```

### 2. Voice Processing Configuration

Set up audio input parameters:

```python
# config/audio_config.py
AUDIO_CONFIG = {
    'chunk': 1024,           # Audio chunk size
    'format': pyaudio.paInt16,  # Audio format
    'channels': 1,           # Mono audio
    'rate': 44100,           # Sample rate
    'record_seconds': 5,     # Recording duration
    'threshold': 500         # Silence threshold for voice detection
}
```

### 3. VLA System Configuration

Configure the complete VLA system:

```python
# config/vla_config.py
VLA_CONFIG = {
    'voice_processing': {
        'enabled': True,
        'api_service': 'openai_whisper',
        'language': 'en-US'
    },
    'cognitive_planning': {
        'enabled': True,
        'llm_service': 'openai_gpt4',
        'temperature': 0.3,
        'max_tokens': 1000
    },
    'execution': {
        'validation_enabled': True,
        'timeout': 30.0,
        'retry_attempts': 3
    }
}
```

## Testing the Setup

### 1. Test Audio Input

Verify your microphone is working:

```python
# test_audio.py
import pyaudio

def test_microphone():
    audio = pyaudio.PyAudio()

    print("Available audio devices:")
    for i in range(audio.get_device_count()):
        dev = audio.get_device_info_by_index(i)
        print(f"{i}: {dev['name']} - {dev['maxInputChannels']} input channels")

    audio.terminate()
    return True

if __name__ == "__main__":
    test_microphone()
```

### 2. Test OpenAI Connection

Verify your OpenAI API access:

```python
# test_openai.py
import openai

def test_openai_connection():
    try:
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": "Hello"}],
            max_tokens=10
        )
        print("OpenAI connection successful!")
        return True
    except Exception as e:
        print(f"OpenAI connection failed: {e}")
        return False

if __name__ == "__main__":
    test_openai_connection()
```

### 3. Test ROS 2 Connection

Verify ROS 2 is properly configured:

```bash
# Test ROS 2 installation
ros2 topic list

# Test ROS 2 nodes
ros2 node list
```

## Running the Complete VLA System

### 1. Launch Individual Components

Start the VLA system components separately:

```bash
# Terminal 1: Launch voice processing
ros2 run vla_module voice_processor_node

# Terminal 2: Launch cognitive planner
ros2 run vla_module cognitive_planner_node

# Terminal 3: Launch execution coordinator
ros2 run vla_module execution_coordinator_node
```

### 2. Launch Complete System

Or launch the complete system using a launch file:

```bash
# Launch the complete VLA system
ros2 launch vla_module vla_system.launch.py
```

## Verification

### End-to-End Pipeline Test

Test the complete VLA pipeline:

1. Speak a simple command like "move forward" into your microphone
2. Observe the command being processed by the voice processor
3. Watch as the cognitive planner decomposes the command
4. Verify the action is executed in the simulation environment

### Expected Output

During a successful test, you should see:

```
[INFO] [1699000000.000000000] [voice_processor]: Recognized: "move forward"
[INFO] [1699000001.000000000] [cognitive_planner]: Planned action: move_to(location="forward")
[INFO] [1699000002.000000000] [execution_coordinator]: Executing navigation action
[INFO] [1699000003.000000000] [execution_coordinator]: Action completed successfully
```

## Troubleshooting Common Issues

### Audio Issues

**Problem**: "No audio input detected"
**Solution**:
- Check microphone permissions
- Verify microphone is set as default input device
- Test with `arecord -D hw:0,0 -f cd test.wav` (Linux) or equivalent

**Problem**: "Poor voice recognition accuracy"
**Solution**:
- Use a high-quality microphone in a quiet environment
- Adjust audio input levels
- Verify OpenAI Whisper API key is valid

### API Issues

**Problem**: "OpenAI API connection failed"
**Solution**:
- Verify API key is correct and has sufficient credits
- Check internet connectivity
- Ensure rate limits are not exceeded

**Problem**: "LLM responses are inconsistent"
**Solution**:
- Adjust temperature settings in configuration
- Review and refine prompt engineering
- Consider using more specific system prompts

### ROS 2 Issues

**Problem**: "ROS 2 nodes not communicating"
**Solution**:
- Verify ROS_DOMAIN_ID is consistent across terminals
- Check network configuration if using multi-machine setup
- Ensure all required packages are installed

## Next Steps

Once your setup is complete and verified:

1. Proceed to Chapter 1: Voice-to-Action to learn voice processing
2. Continue to Chapter 2: Cognitive Planning to master LLM integration
3. Complete Chapter 3: Capstone Project to build the complete system
4. Explore advanced topics and customization options

Your VLA module is now ready for the complete educational experience!