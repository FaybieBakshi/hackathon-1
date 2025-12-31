# Quickstart Guide: Digital Twin (Gazebo & Unity) Module

## Overview
This guide will help you get started with the Digital Twin simulation module covering Gazebo physics, Unity rendering, and sensor simulation for humanoid robots.

## Prerequisites
- Completion of Module 1: ROS 2 basics
- Basic understanding of robotics concepts
- Computer with sufficient resources for simulation (8GB+ RAM recommended)

## Setup Requirements

### System Requirements
- Operating System: Ubuntu 20.04/22.04 or Windows with WSL2
- RAM: 8GB minimum, 16GB recommended
- Storage: 10GB available space
- Graphics: OpenGL 3.3+ compatible GPU (for Unity visualization)

### Software Dependencies
1. **ROS 2 Installation** (already completed from Module 1)
2. **Gazebo** (Garden or Fortress recommended)
   ```bash
   sudo apt install gazebo
   ```
3. **Unity Hub** (for Unity simulation components)
4. **Python 3.8+** (for supporting scripts)

## Getting Started with the Module

### 1. Access the Documentation
Navigate to the Module 2 section in the Docusaurus documentation:
- Start with `docs/module-2/index.md` for the module overview
- Follow the sequential chapters as outlined in the sidebar

### 2. Chapter 1: Gazebo Fundamentals
Begin with the Gazebo fundamentals chapter to understand:
- Physics simulation concepts (gravity, collisions, rigid body dynamics)
- Environment setup and configuration
- Basic humanoid robot simulation

### 3. Chapter 2: Unity Rendering
After completing Gazebo fundamentals, proceed to Unity rendering:
- High-fidelity visual rendering concepts
- Unity-Gazebo integration techniques
- Human-robot interaction visualization

### 4. Chapter 3: Sensor Simulation
Finally, explore sensor simulation:
- LiDAR sensor modeling and data
- Depth camera simulation
- IMU sensor modeling
- Sensor fusion concepts

## First Exercise: Basic Gazebo Simulation
1. Go to the practical exercises in Chapter 1
2. Follow the "Setting up Your First Gazebo Simulation" exercise
3. Launch the simulation and observe basic physics properties
4. Verify that gravity affects the humanoid robot model

## Troubleshooting Common Issues
- If simulations run slowly, reduce the physics update rate
- If Unity doesn't connect to Gazebo, check ROS bridge connections
- For sensor data issues, verify that sensor plugins are properly configured

## Next Steps
After completing this module, you'll be ready to explore more advanced robotics simulation concepts and potentially integrate with real hardware.