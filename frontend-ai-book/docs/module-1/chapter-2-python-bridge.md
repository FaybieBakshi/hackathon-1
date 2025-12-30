---
sidebar_position: 2
---

# Chapter 2: Python-ROS Bridge for AI Integration

## Introduction to Python-ROS Integration

This chapter explores how to bridge Python-based AI agents with ROS 2 controllers using the `rclpy` library. This integration is essential for AI students who want to connect their machine learning algorithms with robotic systems.

## Understanding rclpy

`rclpy` is the Python client library for ROS 2. It provides a Python API that allows Python programs to interact with ROS 2 systems. This makes it perfect for integrating AI algorithms written in Python with ROS 2-based robotic systems.

## Creating an AI Agent Node

Let's start by creating a simple AI agent that can process data and send commands to a ROS controller:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import random

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Publisher for sending commands to robot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for receiving sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Timer for AI decision making
        self.timer = self.create_timer(1.0, self.ai_decision_loop)

        self.get_logger().info('AI Agent Node initialized')

    def sensor_callback(self, msg):
        """Process incoming sensor data"""
        self.get_logger().info(f'Received sensor data: {msg.data}')
        # Process the sensor data with AI logic
        self.process_sensor_data(msg.data)

    def process_sensor_data(self, sensor_data):
        """AI logic to process sensor data and make decisions"""
        # Simple example: AI decides movement based on sensor input
        # In real applications, this would contain ML algorithms
        if 'obstacle' in sensor_data.lower():
            self.get_logger().info('AI detected obstacle, planning avoidance')
            self.send_avoidance_command()
        else:
            self.get_logger().info('AI detected clear path, moving forward')
            self.send_forward_command()

    def ai_decision_loop(self):
        """Periodic AI decision making"""
        self.get_logger().info('AI making decision...')
        # This is where more complex AI logic would run
        # For now, just a simple example

    def send_forward_command(self):
        """Send forward movement command"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Move forward at 0.5 m/s
        twist_msg.angular.z = 0.0  # No rotation
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Sent forward command')

    def send_avoidance_command(self):
        """Send avoidance command"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0   # Stop forward movement
        twist_msg.angular.z = 0.5  # Rotate to avoid obstacle
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info('Sent avoidance command')

def main(args=None):
    rclpy.init(args=args)
    ai_agent_node = AIAgentNode()

    try:
        rclpy.spin(ai_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced AI Integration Example

Here's a more sophisticated example that demonstrates how to integrate machine learning models with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import time

class MLControllerNode(Node):
    def __init__(self):
        super().__init__('ml_controller_node')

        # Publishers and subscribers
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Simulate ML model loading
        self.ml_model_loaded = True
        self.get_logger().info('ML Controller Node initialized with ML model')

    def image_callback(self, msg):
        """Process incoming camera images with ML model"""
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}')

        if self.ml_model_loaded:
            # Simulate processing the image with ML model
            # For this example, we'll simulate processing

            # Simulate some processing time
            time.sleep(0.1)

            # Simulate different commands based on "image analysis"
            # In reality, this would be the output of your ML model
            if np.random.random() > 0.7:  # 30% chance of obstacle detected
                command = "turn_right"
            elif np.random.random() > 0.8:  # 20% chance of clear path
                command = "forward"
            else:
                command = "turn_left"

            self.execute_command(command)

    def execute_command(self, command):
        """Execute command from ML model"""
        twist_msg = Twist()

        if command == "forward":
            twist_msg.linear.x = 0.5
            twist_msg.angular.z = 0.0
            self.get_logger().info('ML: Moving forward')
        elif command == "turn_left":
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.3
            self.get_logger().info('ML: Turning left')
        elif command == "turn_right":
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = -0.3
            self.get_logger().info('ML: Turning right')

        self.cmd_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    ml_controller_node = MLControllerNode()

    try:
        rclpy.spin(ml_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        ml_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Connecting to ROS Controllers

Here's how to create a ROS controller that can receive commands from Python AI agents:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Subscriber for AI commands
        self.ai_command_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # Timer for simulating robot movement
        self.movement_timer = self.create_timer(0.1, self.update_robot_position)

        self.current_position = [0.0, 0.0]
        self.current_heading = 0.0
        self.get_logger().info('Robot Controller initialized')

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands from AI agent"""
        self.get_logger().info(f'Received command: linear={msg.linear.x}, angular={msg.angular.z}')

        # Store the command to be executed in the timer callback
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z

        # Send status update
        status_msg = String()
        status_msg.data = f'Command received: v={msg.linear.x}, w={msg.angular.z}'
        self.status_publisher.publish(status_msg)

    def update_robot_position(self):
        """Simulate robot movement based on commands"""
        dt = 0.1  # Time step

        # Update heading based on angular velocity
        self.current_heading += self.current_angular_vel * dt

        # Update position based on linear velocity
        dx = self.current_linear_vel * math.cos(self.current_heading) * dt
        dy = self.current_linear_vel * math.sin(self.current_heading) * dt

        self.current_position[0] += dx
        self.current_position[1] += dy

        self.get_logger().info(f'Robot position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f})')

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration Example: AI Agent with Humanoid Control

Here's a practical example that demonstrates how an AI agent can control a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import math
import numpy as np

class HumanoidAIController(Node):
    def __init__(self):
        super().__init__('humanoid_ai_controller')

        # Publishers for controlling humanoid robot
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10
        )

        # Subscribers for sensor feedback
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for AI control loop
        self.control_timer = self.create_timer(0.05, self.ai_control_loop)

        self.joint_names = [
            'left_hip', 'right_hip', 'left_knee', 'right_knee',
            'left_ankle', 'right_ankle', 'left_shoulder', 'right_shoulder'
        ]

        self.current_joint_positions = {name: 0.0 for name in self.joint_names}
        self.get_logger().info('Humanoid AI Controller initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions from feedback"""
        for i, name in enumerate(msg.name):
            if name in self.current_joint_positions:
                self.current_joint_positions[name] = msg.position[i]

    def ai_control_loop(self):
        """AI decision making for humanoid control"""
        self.get_logger().info('AI making control decisions...')

        # Example: Generate a walking gait pattern
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create trajectory point with gait pattern
        point = JointTrajectoryPoint()

        # Calculate target positions based on gait cycle
        cycle_time = self.get_clock().now().nanoseconds / 1e9
        for i, joint_name in enumerate(self.joint_names):
            # Different gait patterns for different joints
            if 'hip' in joint_name:
                target_pos = math.sin(cycle_time * 2.0 + i) * 0.3
            elif 'knee' in joint_name:
                target_pos = math.sin(cycle_time * 2.0 + i + 1) * 0.2
            elif 'ankle' in joint_name:
                target_pos = math.sin(cycle_time * 2.0 + i + 2) * 0.1
            else:  # shoulders
                target_pos = math.sin(cycle_time * 0.5 + i) * 0.1

            point.positions.append(target_pos)

        # Set timing
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000  # 50ms

        trajectory_msg.points.append(point)

        # Publish the trajectory command
        self.joint_trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('Published humanoid trajectory command')

def main(args=None):
    rclpy.init(args=args)
    humanoid_ai_controller = HumanoidAIController()

    try:
        rclpy.spin(humanoid_ai_controller)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_ai_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this chapter, you've learned how to bridge Python-based AI agents with ROS 2 controllers:

- Using `rclpy` to create AI nodes that can communicate with ROS systems
- Processing sensor data with AI algorithms and making decisions
- Sending commands from AI agents to ROS controllers
- Practical examples of AI integration with humanoid robot control

This integration allows AI students to leverage their Python and machine learning skills in robotics applications, creating intelligent robotic systems that can perceive, reason, and act in their environment.