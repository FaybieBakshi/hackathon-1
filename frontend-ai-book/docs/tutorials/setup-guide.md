---
sidebar_position: 1
title: "Simulation Environment Setup Guide"
---

# Simulation Environment Setup Guide

## Overview

This guide will help you set up the complete simulation environment for Module 2, including Gazebo and Unity integration. Follow these steps to prepare your development environment for digital twin simulation.

## Prerequisites

Before starting the setup, ensure you have:

- **Operating System**: Ubuntu 20.04/22.04 or Windows with WSL2
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 10GB available space
- **Graphics**: OpenGL 3.3+ compatible GPU (for Unity visualization)
- **Completed**: Module 1 - The Robotic Nervous System (ROS 2)

## System Requirements Check

First, verify your system meets the minimum requirements:

```bash
# Check available RAM
free -h

# Check available disk space
df -h

# Check graphics capabilities (Linux)
glxinfo | grep "OpenGL version"
```

## Software Dependencies Installation

### 1. ROS 2 Installation

Ensure you have ROS 2 installed from Module 1. If not, install it:

```bash
# For Ubuntu 22.04 (Humble Hawksbill)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update

# Install ROS 2 Humble
sudo apt install ros-humble-desktop-full
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install ROS 2 development tools
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 2. Gazebo Installation

Install Gazebo Garden (recommended version):

```bash
# Add Gazebo repository
sudo curl -sSL http://get.gazebosim.org | sh

# Install Gazebo Garden
sudo apt install gazebo-garden
```

### 3. Unity Hub Installation

For Unity integration, install Unity Hub:

```bash
# Download and install Unity Hub (Linux)
# Visit https://unity.com/download to download the Unity Hub installer
# Or use the .deb package for Ubuntu
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage
chmod +x UnityHub.AppImage
./UnityHub.AppImage
```

For Windows users:
- Download Unity Hub from https://unity.com/download
- Install and log in to your Unity account

### 4. Python Dependencies

Install additional Python packages needed for simulation:

```bash
pip3 install transforms3d numpy matplotlib
```

## Environment Configuration

### 1. ROS 2 Workspace Setup

Create a workspace for your simulation projects:

```bash
# Create workspace directory
mkdir -p ~/simulation_ws/src
cd ~/simulation_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build
source install/setup.bash
```

### 2. Gazebo Models Setup

Download and configure Gazebo models:

```bash
# Create models directory
mkdir -p ~/.gazebo/models

# Clone common robot models
cd ~/simulation_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
git clone https://github.com/ros-simulation/gazebo_ros_demos.git
```

### 3. Simulation Environment Variables

Add the following to your `~/.bashrc`:

```bash
# Gazebo settings
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models:~/simulation_ws/src/gazebo_ros_demos/gazebo_classic_models/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/simulation_ws/src/gazebo_ros_demos/gazebo_classic_models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/simulation_ws/build/gazebo_ros_pkgs/gazebo_ros

# ROS 2 settings
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Testing the Installation

### 1. Test Gazebo

Launch Gazebo to verify the installation:

```bash
gazebo
```

You should see the Gazebo interface with a default empty world.

### 2. Test ROS 2 Integration

Open a new terminal and test ROS 2:

```bash
# Terminal 1: Launch Gazebo
gazebo

# Terminal 2: Check ROS 2 nodes
source /opt/ros/humble/setup.bash
ros2 node list
```

## Common Issues and Solutions

### Issue: Gazebo fails to start with graphics errors
**Solution**: Check your graphics drivers and ensure you have OpenGL 3.3+ support:
```bash
glxinfo | grep "OpenGL version"
export MESA_GL_VERSION_OVERRIDE=3.3
gazebo
```

### Issue: ROS 2 packages not found
**Solution**: Ensure you've sourced the correct setup files:
```bash
source /opt/ros/humble/setup.bash
source ~/simulation_ws/install/setup.bash
```

### Issue: Permission denied when accessing models
**Solution**: Fix permissions in the models directory:
```bash
sudo chown -R $USER:$USER ~/.gazebo/models
```

## Next Steps

After completing this setup, you're ready to proceed with Chapter 1: Gazebo Fundamentals. The simulation environment is now properly configured for the digital twin exercises in this module.