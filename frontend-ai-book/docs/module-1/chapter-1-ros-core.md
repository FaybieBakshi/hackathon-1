---
sidebar_position: 1
---

# Chapter 1: ROS 2 Core Fundamentals

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Why ROS 2?

ROS 2 addresses several limitations of the original ROS:
- **Real-time support**: Critical for robot control applications
- **Multi-robot systems**: Better support for multiple robots working together
- **Security**: Built-in security features for safe robot operation
- **Commercial deployment**: Production-ready features for industrial use

## Nodes

A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Multiple nodes are typically used together to accomplish a robot's purpose.

### Creating a Simple Node

Let's create a basic ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Minimal node started')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    try:
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Messages

Topics are named buses over which nodes exchange messages. The topic name is a unique name in the ROS graph that allows nodes to connect and communicate.

### Publisher Example

Here's how to create a publisher that sends string messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

Here's how to create a subscriber that receives the messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

Services provide a request/response communication pattern. A service client sends a request message to a service server, which processes the request and returns a response.

### Service Example

First, define a service request:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And a client for the service:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: %d' % response.sum)

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Exercise: Publisher-Subscriber Communication

Now let's create a practical example that demonstrates publisher-subscriber communication in the context of humanoid control:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joint_names = ['left_hip', 'right_hip', 'left_knee', 'right_knee', 'left_ankle', 'right_ankle']
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = []

        # Simulate walking motion with sinusoidal patterns
        for j, name in enumerate(self.joint_names):
            # Different phase for each joint to simulate walking
            position = math.sin(self.i * 0.1 + j * 0.5) * 0.5
            msg.position.append(position)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'humanoid_base'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint states: {msg.position}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    humanoid_controller = HumanoidController()

    try:
        rclpy.spin(humanoid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this chapter, you've learned about the fundamental concepts of ROS 2:
- **Nodes**: The basic computational elements
- **Topics**: The communication buses for message passing
- **Services**: Request/response communication patterns

These concepts form the foundation for all ROS 2 applications, including humanoid robot control systems. In the next chapter, we'll explore how to integrate Python-based AI agents with ROS 2 controllers.