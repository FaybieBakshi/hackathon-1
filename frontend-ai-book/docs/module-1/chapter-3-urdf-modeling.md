---
sidebar_position: 3
---

# Chapter 3: Humanoid Robot Modeling with URDF

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robots in ROS. It's essential for humanoid robot modeling as it defines the robot's physical structure, including links, joints, and their relationships.

## Understanding URDF Structure

A URDF file describes a robot using:
- **Links**: Rigid bodies (e.g., limbs, torso)
- **Joints**: Connections between links with specific degrees of freedom
- **Visual**: How the robot appears in simulation
- **Collision**: How the robot interacts with the environment

## Basic Humanoid URDF Example

Here's a simple humanoid robot model with basic structure:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Joint connecting base to torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Left leg -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="torso_to_left_thigh" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="-0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_thigh_to_shin" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right leg (similar to left leg) -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="torso_to_right_thigh" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_thigh_to_shin" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Advanced Humanoid URDF with Arms

Here's a more complex humanoid model that includes arms:

```xml
<?xml version="1.0"?>
<robot name="advanced_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.6"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 -1.57"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_to_upper_arm" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 -0.075" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.03"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_upper_arm_to_forearm" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>

  <!-- Right Arm (similar to left arm) -->
  <link name="right_shoulder">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="torso_to_right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 1.57"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder_to_upper_arm" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0 -0.075" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.03"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_upper_arm_to_forearm" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>

  <!-- Left leg (same as in basic model) -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="torso_to_left_thigh" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="-0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_thigh_to_shin" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right leg (same as in basic model) -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="torso_to_right_thigh" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_thigh_to_shin" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Loading and Validating URDF Files

To load and validate your URDF files in ROS 2, you can use the following Python script:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import xml.etree.ElementTree as ET

class URDFValidator(Node):
    def __init__(self):
        super().__init__('urdf_validator')

        # Publisher for validation results
        self.validation_publisher = self.create_publisher(String, 'urdf_validation', 10)

        # Timer to periodically validate URDF
        self.timer = self.create_timer(5.0, self.validate_urdf)

        self.urdf_path = 'path/to/your/humanoid.urdf'  # Update this path
        self.get_logger().info('URDF Validator initialized')

    def validate_urdf(self):
        """Validate the URDF file"""
        try:
            # Parse the URDF file
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()

            # Check if it's a valid robot description
            if root.tag != 'robot':
                raise ValueError('URDF file must have a robot root element')

            robot_name = root.get('name')
            if not robot_name:
                raise ValueError('Robot must have a name attribute')

            # Count links and joints
            links = root.findall('link')
            joints = root.findall('joint')

            validation_msg = String()
            validation_msg.data = f'Valid URDF: {robot_name} with {len(links)} links and {len(joints)} joints'
            self.validation_publisher.publish(validation_msg)

            self.get_logger().info(f'URDF validation successful: {validation_msg.data}')

        except ET.ParseError as e:
            error_msg = String()
            error_msg.data = f'URDF Parse Error: {str(e)}'
            self.validation_publisher.publish(error_msg)
            self.get_logger().error(f'URDF validation failed: {str(e)}')
        except Exception as e:
            error_msg = String()
            error_msg.data = f'URDF Validation Error: {str(e)}'
            self.validation_publisher.publish(error_msg)
            self.get_logger().error(f'URDF validation failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    urdf_validator = URDFValidator()

    try:
        rclpy.spin(urdf_validator)
    except KeyboardInterrupt:
        pass
    finally:
        urdf_validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Using URDF with RViz2 for Visualization

Here's how to visualize your URDF model in RViz2:

```bash
# Launch RViz2 with robot state publisher
ros2 run rviz2 rviz2

# In another terminal, publish the robot description
ros2 param set /robot_state_publisher robot_description "$(cat path/to/your/humanoid.urdf)"
```

Or create a launch file:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the launch directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # Declare launch arguments
    urdf_model_path = os.path.join(pkg_dir, 'urdf', 'humanoid.urdf')

    # Publish robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_model_path).read()
        }]
    )

    # Joint state publisher (for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])
```

## URDF Best Practices for Humanoid Robots

1. **Consistent Naming**: Use consistent naming conventions for joints and links
2. **Proper Inertial Properties**: Include realistic mass and inertia values
3. **Joint Limits**: Define appropriate limits for each joint
4. **Collision vs Visual**: Use simple shapes for collision to improve performance
5. **Gazebo Integration**: Add Gazebo-specific tags if using Gazebo simulation

## Summary

In this chapter, you've learned about:
- The structure of URDF files for humanoid robot modeling
- How to define links and joints for humanoid robots
- How to include arms, legs, torso, and head in your model
- How to validate and visualize URDF models in ROS 2
- Best practices for creating humanoid robot models

URDF is fundamental for humanoid robot simulation and control, allowing you to create accurate physical models that can be used with your AI agents for perception, planning, and control tasks.