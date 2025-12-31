---
sidebar_position: 2
title: "Simulation Troubleshooting Guide"
---

# Simulation Troubleshooting Guide

## Overview

This guide provides solutions to common issues you may encounter while working with the digital twin simulation environment. Use this as a reference when you face problems during Module 2 exercises.

## Gazebo-Specific Issues

### Issue: Gazebo fails to start or crashes immediately
**Symptoms**: Gazebo window appears briefly then closes, or process terminates with graphics errors.

**Solutions**:
1. Check OpenGL support:
   ```bash
   glxinfo | grep "OpenGL version"
   ```

2. If using virtual machines or WSL2, try software rendering:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gazebo
   ```

3. If using NVIDIA graphics, ensure proper drivers are installed:
   ```bash
   nvidia-smi
   # If not available, install: sudo apt install nvidia-driver-470
   ```

### Issue: Physics simulation runs slowly
**Symptoms**: Low frame rate, laggy robot movement, delayed responses.

**Solutions**:
1. Reduce physics update rate in Gazebo:
   - In Gazebo GUI, go to Edit → Preferences → Physics
   - Lower "Real Time Update Rate" to 500 or 1000
   - Reduce "Max Step Size" to 0.001 or 0.002

2. Optimize simulation parameters:
   ```xml
   <!-- In your world file -->
   <physics type="ode">
     <max_step_size>0.002</max_step_size>
     <real_time_factor>1.0</real_time_factor>
     <real_time_update_rate>500</real_time_update_rate>
   </physics>
   ```

### Issue: Robot model falls through the ground
**Symptoms**: Robot or objects fall through the ground plane or other static objects.

**Solutions**:
1. Check collision properties in your URDF/SDF model:
   - Ensure collision meshes are properly defined
   - Verify that static objects have `<static>true</static>`
   - Check that collision geometries match visual geometries

2. Adjust physics parameters:
   ```xml
   <physics type="ode">
     <solver>
       <type>quick</type>
       <iters>10</iters>
       <sor>1.3</sor>
     </solver>
     <constraints>
       <cfm>0.000001</cfm>
       <erp>0.2</erp>
       <contact_max_correcting_vel>100</contact_max_correcting_vel>
       <contact_surface_layer>0.001</contact_surface_layer>
     </constraints>
   </physics>
   ```

## ROS 2 Integration Issues

### Issue: No communication between ROS 2 and Gazebo
**Symptoms**: Robot controllers don't respond, sensor data not published, joint states not updating.

**Solutions**:
1. Check ROS 2 domain ID:
   ```bash
   echo $ROS_DOMAIN_ID
   # Should be the same for both Gazebo and your ROS nodes
   ```

2. Verify available topics:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ros2 topic echo /joint_states  # Test if sensor data is available
   ```

3. Check Gazebo ROS plugins are loaded:
   ```bash
   # Look for gazebo_ros_pkgs in your workspace
   ls ~/simulation_ws/install/gazebo_ros*
   ```

### Issue: High latency in control commands
**Symptoms**: Delayed robot response to control commands, poor real-time performance.

**Solutions**:
1. Check network configuration:
   ```bash
   # For localhost communication
   export ROS_LOCALHOST_ONLY=1
   ```

2. Optimize QoS settings in your ROS 2 nodes:
   ```python
   # Use appropriate QoS profiles for real-time control
   from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

   qos_profile = QoSProfile(
       depth=1,
       durability=QoSDurabilityPolicy.VOLATILE,
       reliability=QoSReliabilityPolicy.BEST_EFFORT
   )
   ```

## Unity Integration Issues

### Issue: Unity fails to connect to Gazebo
**Symptoms**: No synchronization between Unity visualization and Gazebo physics, connection errors.

**Solutions**:
1. Check ROS bridge connection:
   ```bash
   # Ensure rosbridge_server is running
   ros2 run rosbridge_server rosbridge_websocket
   ```

2. Verify network connectivity between Unity and ROS:
   ```bash
   # Test connection to ROS bridge
   telnet localhost 9090
   ```

3. Check Unity ROS TCP connection settings:
   - Verify IP address and port match ROS bridge settings
   - Ensure firewall allows connections on the specified port

### Issue: Unity visualization lags behind physics
**Symptoms**: Visual representation in Unity doesn't match Gazebo physics state.

**Solutions**:
1. Increase Unity frame rate for synchronization:
   ```csharp
   // In Unity, ensure physics updates align with ROS updates
   Time.fixedDeltaTime = 0.01f; // Match Gazebo update rate
   ```

2. Optimize data transmission frequency:
   - Reduce the frequency of transform updates if network is constrained
   - Implement interpolation for smoother visual updates

## Sensor Simulation Issues

### Issue: LiDAR data shows artifacts or incorrect readings
**Symptoms**: Point cloud has noise, incorrect distances, or missing data.

**Solutions**:
1. Check LiDAR plugin configuration:
   ```xml
   <sensor name="lidar" type="ray">
     <ray>
       <scan>
         <horizontal>
           <samples>720</samples>  <!-- Reduce if performance is poor -->
           <resolution>1</resolution>
           <min_angle>-3.14159</min_angle>
           <max_angle>3.14159</max_angle>
         </horizontal>
       </scan>
       <range>
         <min>0.1</min>
         <max>30.0</max>  <!-- Adjust based on your needs -->
         <resolution>0.01</resolution>
       </range>
     </ray>
   </sensor>
   ```

2. Verify coordinate frame transformations:
   - Ensure LiDAR frame is properly defined in URDF
   - Check that TF tree is properly published

### Issue: Depth camera produces black or incorrect images
**Symptoms**: Depth images are all zeros, incorrect depth values, or no image data.

**Solutions**:
1. Check camera plugin configuration:
   ```xml
   <sensor name="depth_camera" type="depth">
     <camera>
       <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
       <image>
         <width>640</width>
         <height>480</height>
         <format>R8G8B8</format>
       </image>
       <clip>
         <near>0.1</near>
         <far>10.0</far>
       </clip>
     </camera>
   </sensor>
   ```

2. Verify image transport:
   ```bash
   # Check if camera topics are being published
   ros2 topic list | grep camera
   ros2 topic echo /camera/depth/image_raw
   ```

## Performance Optimization

### Issue: Overall simulation performance is poor
**Symptoms**: Low frame rate, high CPU usage, delayed responses.

**Solutions**:
1. Reduce simulation complexity:
   - Simplify collision meshes (use boxes/cylinders instead of complex meshes)
   - Reduce physics update rate when possible
   - Limit the number of active sensors

2. Optimize model complexity:
   ```xml
   <!-- Use simpler collision geometry -->
   <collision name="collision">
     <geometry>
       <box>
         <size>0.1 0.1 0.1</size>
       </box>
     </geometry>
   </collision>
   ```

3. Monitor resource usage:
   ```bash
   # Monitor CPU and memory usage
   htop
   # Monitor graphics performance
   nvidia-smi  # For NVIDIA cards
   ```

## Network and Communication Issues

### Issue: Network timeouts or connection failures
**Symptoms**: Intermittent connection drops, timeout errors, failed communication.

**Solutions**:
1. Check firewall settings:
   ```bash
   # Ubuntu firewall
   sudo ufw status
   # Allow necessary ports for ROS and Unity
   sudo ufw allow 9090  # ROS bridge
   sudo ufw allow 11345  # Gazebo
   ```

2. Configure network settings:
   ```bash
   # Set proper ROS environment variables
   export ROS_IP=127.0.0.1
   export ROS_MASTER_URI=http://localhost:11311
   ```

## Validation Commands

Use these commands to verify your simulation setup:

```bash
# Check all running ROS nodes
ros2 node list

# Verify Gazebo connection
gz topic -l

# Test ROS-Gazebo bridge
ros2 topic list | grep gazebo

# Check TF tree
ros2 run tf2_tools view_frames
```

## Getting Help

If you encounter issues not covered in this guide:

1. Check the official documentation:
   - [Gazebo Documentation](http://gazebosim.org/tutorials)
   - [ROS 2 Documentation](https://docs.ros.org/)
   - [Unity Robotics Documentation](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

2. Search for existing solutions:
   - ROS Answers: https://answers.ros.org/
   - Gazebo Answers: http://answers.gazebosim.org/

3. Create a minimal example to reproduce the issue
4. Include version information when asking for help