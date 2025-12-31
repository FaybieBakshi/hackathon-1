---
sidebar_position: 4
title: "Practical Exercises"
---

# Practical Exercises: Gazebo Physics Simulation

## Overview

This section contains hands-on exercises that will help you apply the physics concepts you've learned. Each exercise builds upon the previous one, gradually increasing in complexity to solidify your understanding of Gazebo physics simulation.

## Exercise 1: Basic Physics Environment

### Objective
Create a simple physics environment with a falling cube and observe realistic gravity effects.

### Prerequisites
- Gazebo installed and running
- Basic understanding of SDF format

### Steps

1. **Create a new world file** (`falling_cube.world`):
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="falling_cube_world">
       <physics type="ode">
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
         <real_time_update_rate>1000</real_time_update_rate>
         <gravity>0 0 -9.8</gravity>
       </physics>

       <light name="sun" type="directional">
         <pose>0 0 10 0 0 0</pose>
         <diffuse>0.8 0.8 0.8 1</diffuse>
         <specular>0.2 0.2 0.2 1</specular>
         <direction>-0.5 0.1 -0.9</direction>
       </light>

       <model name="ground_plane">
         <static>true</static>
         <link name="link">
           <collision name="collision">
             <geometry>
               <plane>
                 <normal>0 0 1</normal>
                 <size>10 10</size>
               </plane>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <plane>
                 <normal>0 0 1</normal>
                 <size>10 10</size>
               </plane>
             </geometry>
             <material>
               <ambient>0.7 0.7 0.7 1</ambient>
               <diffuse>0.7 0.7 0.7 1</diffuse>
             </material>
           </visual>
         </link>
       </model>

       <model name="falling_cube">
         <pose>0 0 2 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.5 0.5 0.5</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.5 0.5 0.5</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.2 0.2 1</ambient>
               <diffuse>0.8 0.2 0.2 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1.0</mass>
             <inertia>
               <ixx>0.0417</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.0417</iyy>
               <iyz>0</iyz>
               <izz>0.0417</izz>
             </inertia>
           </inertial>
         </link>
       </model>
     </world>
   </sdf>
   ```

2. **Save the file** in your Gazebo models directory:
   ```bash
   mkdir -p ~/.gazebo/worlds
   nano ~/.gazebo/worlds/falling_cube.world
   ```

3. **Launch the simulation**:
   ```bash
   gazebo ~/.gazebo/worlds/falling_cube.world
   ```

4. **Observe the behavior**: Watch the cube fall and hit the ground plane. Notice how it bounces and eventually comes to rest.

### Expected Outcome
The cube should fall due to gravity, hit the ground plane, bounce slightly, and come to rest on the surface.

### Questions to Consider
- How does changing the cube's mass affect its fall?
- What happens if you change the gravity value to -19.6 (double Earth's gravity)?
- How does the bounce behavior change with different surface properties?

## Exercise 2: Collision Detection and Response

### Objective
Create multiple objects with different collision properties and observe their interactions.

### Steps

1. **Create a new world file** (`collision_test.world`):
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="collision_test_world">
       <physics type="ode">
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
         <real_time_update_rate>1000</real_time_update_rate>
         <gravity>0 0 -9.8</gravity>
       </physics>

       <light name="sun" type="directional">
         <pose>0 0 10 0 0 0</pose>
         <diffuse>0.8 0.8 0.8 1</diffuse>
         <specular>0.2 0.2 0.2 1</specular>
         <direction>-0.5 0.1 -0.9</direction>
       </light>

       <model name="ground_plane">
         <static>true</static>
         <link name="link">
           <collision name="collision">
             <geometry>
               <plane>
                 <normal>0 0 1</normal>
                 <size>10 10</size>
               </plane>
             </geometry>
             <surface>
               <friction>
                 <ode>
                   <mu>0.5</mu>
                   <mu2>0.5</mu2>
                 </ode>
               </friction>
             </surface>
           </collision>
           <visual name="visual">
             <geometry>
               <plane>
                 <normal>0 0 1</normal>
                 <size>10 10</size>
               </plane>
             </geometry>
             <material>
               <ambient>0.7 0.7 0.7 1</ambient>
               <diffuse>0.7 0.7 0.7 1</diffuse>
             </material>
           </visual>
         </link>
       </model>

       <!-- Sphere with low friction -->
       <model name="low_friction_sphere">
         <pose>-2 0 2 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <sphere>
                 <radius>0.3</radius>
               </sphere>
             </geometry>
             <surface>
               <friction>
                 <ode>
                   <mu>0.1</mu>
                   <mu2>0.1</mu2>
                 </ode>
               </friction>
             </surface>
           </collision>
           <visual name="visual">
             <geometry>
               <sphere>
                 <radius>0.3</radius>
               </sphere>
             </geometry>
             <material>
               <ambient>0.8 0.2 0.2 1</ambient>
               <diffuse>0.8 0.2 0.2 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>0.5</mass>
             <inertia>
               <ixx>0.045</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.045</iyy>
               <iyz>0</iyz>
               <izz>0.045</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Cube with high friction -->
       <model name="high_friction_cube">
         <pose>2 0 2 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.4 0.4 0.4</size>
               </box>
             </geometry>
             <surface>
               <friction>
                 <ode>
                   <mu>1.0</mu>
                   <mu2>1.0</mu2>
                 </ode>
               </friction>
             </surface>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.4 0.4 0.4</size>
               </box>
             </geometry>
             <material>
               <ambient>0.2 0.8 0.2 1</ambient>
               <diffuse>0.2 0.8 0.2 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1.0</mass>
             <inertia>
               <ixx>0.0533</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.0533</iyy>
               <iyz>0</iyz>
               <izz>0.0533</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <!-- Cylinder with bounce -->
       <model name="bouncy_cylinder">
         <pose>0 2 2 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <cylinder>
                 <radius>0.25</radius>
                 <length>0.5</length>
               </cylinder>
             </geometry>
             <surface>
               <bounce>
                 <restitution_coefficient>0.8</restitution_coefficient>
                 <threshold>100000</threshold>
               </bounce>
             </surface>
           </collision>
           <visual name="visual">
             <geometry>
               <cylinder>
                 <radius>0.25</radius>
                 <length>0.5</length>
               </cylinder>
             </geometry>
             <material>
               <ambient>0.2 0.2 0.8 1</ambient>
               <diffuse>0.2 0.2 0.8 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>0.8</mass>
             <inertia>
               <ixx>0.025</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.025</iyy>
               <iyz>0</iyz>
               <izz>0.04</izz>
             </inertia>
           </inertial>
         </link>
       </model>
     </world>
   </sdf>
   ```

2. **Save the file** and launch the simulation:
   ```bash
   nano ~/.gazebo/worlds/collision_test.world
   gazebo ~/.gazebo/worlds/collision_test.world
   ```

3. **Observe the different behaviors**:
   - The low-friction sphere should slide more easily
   - The high-friction cube should grip the surface better
   - The bouncy cylinder should bounce significantly

### Expected Outcome
Each object should behave differently based on its surface properties, demonstrating how friction and bounce affect collision response.

## Exercise 3: Rigid Body Dynamics and Forces

### Objective
Apply external forces to objects and observe their dynamic response.

### Steps

1. **Create a launch file** for a simulation with a simple controller:
   ```bash
   mkdir -p ~/simulation_ws/src/my_robot_simulation/launch
   nano ~/simulation_ws/src/my_robot_simulation/launch/force_demo.launch.py
   ```

2. **Add the following launch file content**:
   ```python
   from launch import LaunchDescription
   from launch.actions import ExecuteProcess
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           # Launch Gazebo with a world file
           ExecuteProcess(
               cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
                    '~/simulation_ws/src/my_robot_simulation/worlds/force_test.world'],
               output='screen'
           ),

           # Launch a simple force application node
           Node(
               package='my_robot_simulation',
               executable='force_controller',
               name='force_controller'
           )
       ])
   ```

3. **Create a simple force application script**:
   ```bash
   mkdir -p ~/simulation_ws/src/my_robot_simulation/scripts
   nano ~/simulation_ws/src/my_robot_simulation/scripts/force_demo.py
   ```

4. **Add the following Python script**:
   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from gazebo_msgs.srv import ApplyBodyWrench
   from geometry_msgs.msg import Wrench
   from std_msgs.msg import Empty
   import time

   class ForceDemoNode(Node):
       def __init__(self):
           super().__init__('force_demo_node')

           # Create service client for applying forces
           self.apply_force_client = self.create_client(
               ApplyBodyWrench, '/gazebo/apply_body_wrench')

           # Wait for service to be available
           while not self.apply_force_client.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('Waiting for apply_body_wrench service...')

           # Apply forces after a delay
           self.timer = self.create_timer(3.0, self.apply_force_to_object)
           self.force_count = 0

       def apply_force_to_object(self):
           if self.force_count < 5:  # Apply force 5 times
               req = ApplyBodyWrench.Request()
               req.body_name = "falling_cube::link"  # Match your model name
               req.reference_frame = "world"
               req.wrench = Wrench()
               req.wrench.force.x = 10.0  # Apply force in x direction
               req.wrench.force.y = 0.0
               req.wrench.force.z = 0.0

               # Apply the force for a short duration
               req.duration.sec = 1

               future = self.apply_force_client.call_async(req)
               self.force_count += 1

               self.get_logger().info(f'Applied force #{self.force_count}')
           else:
               self.timer.cancel()

   def main(args=None):
       rclpy.init(args=args)
       node = ForceDemoNode()
       rclpy.spin(node)
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

5. **Make the script executable**:
   ```bash
   chmod +x ~/simulation_ws/src/my_robot_simulation/scripts/force_demo.py
   ```

6. **Build and run the simulation**:
   ```bash
   cd ~/simulation_ws
   source /opt/ros/humble/setup.bash
   colcon build
   source install/setup.bash
   ros2 launch my_robot_simulation force_demo.launch.py
   ```

### Expected Outcome
The cube should experience external forces applied to it, causing it to move in the direction of the force.

## Exercise 4: Multi-Object Physics Scene

### Objective
Create a complex scene with multiple interacting objects to test physics simulation capabilities.

### Steps

1. **Create a complex world file** (`complex_physics.world`):
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="complex_physics_world">
       <physics type="ode">
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
         <real_time_update_rate>1000</real_time_update_rate>
         <gravity>0 0 -9.8</gravity>
       </physics>

       <light name="sun" type="directional">
         <pose>0 0 10 0 0 0</pose>
         <diffuse>0.8 0.8 0.8 1</diffuse>
         <specular>0.2 0.2 0.2 1</specular>
         <direction>-0.5 0.1 -0.9</direction>
       </light>

       <!-- Ground plane -->
       <model name="ground_plane">
         <static>true</static>
         <link name="link">
           <collision name="collision">
             <geometry>
               <plane>
                 <normal>0 0 1</normal>
                 <size>20 20</size>
               </plane>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <plane>
                 <normal>0 0 1</normal>
                 <size>20 20</size>
               </plane>
             </geometry>
             <material>
               <ambient>0.5 0.5 0.5 1</ambient>
               <diffuse>0.5 0.5 0.5 1</diffuse>
             </material>
           </visual>
         </link>
       </model>

       <!-- Tower of blocks -->
       <model name="tower_base">
         <pose>0 0 0.5 0 0 0</pose>
         <static>true</static>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>2 2 1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>2 2 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.6 0.2 1</ambient>
               <diffuse>0.8 0.6 0.2 1</diffuse>
             </material>
           </visual>
         </link>
       </model>

       <!-- Stack of cubes -->
       <model name="cube_stack_1">
         <pose>-3 0 0.5 0 0 0</pose>
         <link name="base">
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.5 0.5 0.5</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.5 0.5 0.5</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.2 0.2 1</ambient>
               <diffuse>0.8 0.2 0.2 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1.0</mass>
             <inertia>
               <ixx>0.0417</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.0417</iyy>
               <iyz>0</iyz>
               <izz>0.0417</izz>
             </inertia>
           </inertial>
         </link>
         <link name="middle">
           <pose>0 0 0.5 0 0 0</pose>
           <collision name="collision">
             <geometry>
               <box>
                 <size>0.5 0.5 0.5</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>0.5 0.5 0.5</size>
               </box>
             </geometry>
             <material>
               <ambient>0.2 0.8 0.2 1</ambient>
               <diffuse>0.2 0.8 0.2 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1.0</mass>
             <inertia>
               <ixx>0.0417</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.0417</iyy>
               <iyz>0</iyz>
               <izz>0.0417</izz>
             </inertia>
           </inertial>
         </link>
         <joint name="base_to_middle" type="fixed">
           <parent>base</parent>
           <child>middle</child>
         </joint>
       </model>

       <!-- Rolling ball track -->
       <model name="ball_track">
         <pose>3 0 1 0 0 0</pose>
         <static>true</static>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>2 0.1 0.1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>2 0.1 0.1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.2 0.2 0.8 1</ambient>
               <diffuse>0.2 0.2 0.8 1</diffuse>
             </material>
           </visual>
         </link>
       </model>

       <!-- Rolling ball -->
       <model name="rolling_ball">
         <pose>2 0 1.2 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <sphere>
                 <radius>0.1</radius>
               </sphere>
             </geometry>
             <surface>
               <friction>
                 <ode>
                   <mu>0.05</mu>
                   <mu2>0.05</mu2>
                 </ode>
               </friction>
             </surface>
           </collision>
           <visual name="visual">
             <geometry>
               <sphere>
                 <radius>0.1</radius>
               </sphere>
             </geometry>
             <material>
               <ambient>0.8 0.8 0.2 1</ambient>
               <diffuse>0.8 0.8 0.2 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>0.1</mass>
             <inertia>
               <ixx>0.0004</ixx>
               <ixy>0</ixy>
               <ixz>0</ixz>
               <iyy>0.0004</iyy>
               <iyz>0</iyz>
               <izz>0.0004</izz>
             </inertia>
           </inertial>
         </link>
       </model>
     </world>
   </sdf>
   ```

2. **Save and launch the complex simulation**:
   ```bash
   nano ~/.gazebo/worlds/complex_physics.world
   gazebo ~/.gazebo/worlds/complex_physics.world
   ```

3. **Observe the interactions** between multiple objects and note how the physics engine handles multiple simultaneous collisions.

### Expected Outcome
Multiple objects should interact realistically, with proper collision detection and response across the entire scene.

## Exercise 5: Performance and Stability Testing

### Objective
Test the stability and performance of your physics simulation under various conditions.

### Steps

1. **Create a performance test world** with many objects:
   ```xml
   <!-- Create a world with 20+ objects to test performance -->
   ```

2. **Monitor simulation performance**:
   ```bash
   # While simulation is running, check real-time factor
   gz stats
   ```

3. **Adjust physics parameters** and observe the effects:
   - Change `max_step_size` to 0.01 and observe stability vs performance
   - Change `real_time_update_rate` to 100 and observe performance
   - Adjust solver iterations and observe stability

4. **Document your findings**:
   - What is the optimal balance for your system?
   - What trade-offs do you observe?

## Challenge Exercise: Physics-Based Puzzle

### Objective
Create a physics-based puzzle where you need to use forces to achieve a specific goal.

### Challenge Description
Create a simulation where you must apply forces to move a ball through a series of obstacles to reach a target.

### Requirements
- At least 3 obstacles that the ball must navigate around
- A target area that the ball must reach
- Use only external forces (no direct position control)
- Document your approach and the forces required

### Solution Approach
1. Design the obstacle course in a world file
2. Create a controller that applies forces to the ball
3. Use physics properties to your advantage (bounces, friction, etc.)
4. Test and refine your approach

## Troubleshooting Tips

### Common Issues and Solutions

1. **Objects falling through surfaces**:
   - Check collision geometry alignment
   - Verify static property for ground planes
   - Adjust surface layer parameters

2. **Unstable simulations**:
   - Reduce time step size
   - Increase solver iterations
   - Check mass ratios between objects

3. **Poor performance**:
   - Simplify collision geometry
   - Reduce update rates
   - Limit the number of active objects

## Summary

These practical exercises have given you hands-on experience with:
- Basic physics environment setup
- Collision detection and response
- Rigid body dynamics and force application
- Complex multi-object interactions
- Performance and stability testing

## Next Steps

After completing these exercises, you should have a solid understanding of Gazebo physics simulation. In the next chapter, we'll explore Unity integration for high-fidelity rendering and visualization.