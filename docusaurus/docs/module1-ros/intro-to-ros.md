# Introduction to ROS 2

The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS 2 is the latest iteration of this framework, re-architected to address the limitations of ROS 1, particularly in areas like real-time control, multi-robot systems, and embedded platforms.

## Concepts

### What is ROS 2?

ROS 2 is not an operating system in the traditional sense (like Windows, macOS, or Linux). Instead, it's a meta-operating system for robots. It provides services that are typically expected from an operating system, including hardware abstraction, low-level device control, implementation of common functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.

Key features of ROS 2 include:
- **Distributed System:** ROS 2 is built on a Data Distribution Service (DDS) layer, enabling direct communication between nodes without a central master. This enhances reliability, scalability, and real-time capabilities.
- **Real-time Communication:** Designed to meet the demands of real-time control systems, crucial for applications like autonomous driving and industrial automation.
- **Multi-Robot Support:** Improved mechanisms for managing and coordinating multiple robots.
- **Platform Agnostic:** Supports various operating systems, including Linux, Windows, macOS, and embedded systems.
- **Quality of Service (QoS):** Configurable parameters for reliability, durability, and latency of message passing.

### Why ROS 2?

Robotics development is inherently complex due to the interdisciplinary nature of the field, combining hardware, software, sensing, and control. ROS 2 helps manage this complexity by:
- **Modular Design:** Breaking down robot functionalities into smaller, manageable units (nodes) that communicate via message passing.
- **Code Reusability:** Providing a vast collection of existing libraries and tools for common robotics tasks (e.g., navigation, perception, manipulation).
- **Hardware Abstraction:** Offering a standardized interface to interact with diverse robot hardware, allowing developers to focus on high-level algorithms.
- **Community Support:** Backed by a large and active global community that contributes, supports, and maintains the ecosystem.

## Working Code Examples

### Example 1: Basic ROS 2 Publisher and Subscriber (Python)

This example demonstrates the fundamental communication pattern in ROS 2: a publisher node sending messages and a subscriber node receiving them.

#### Publisher (`simple_publisher.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber (`simple_subscriber.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run

1.  **Create a ROS 2 Workspace:**
    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    ```
2.  **Create a Python Package:**
    ```bash
    ros2 pkg create --build-type ament_python my_ros2_package
    ```
3.  **Place Code:** Save `simple_publisher.py` and `simple_subscriber.py` inside `ros2_ws/src/my_ros2_package/my_ros2_package/`.
4.  **Update `setup.py` and `package.xml`:**
    *   In `ros2_ws/src/my_ros2_package/setup.py`, add the entry points for your scripts:
        ```python
        entry_points={
            'console_scripts': [
                'simple_publisher = my_ros2_package.simple_publisher:main',
                'simple_subscriber = my_ros2_package.simple_subscriber:main',
            ],
        },
        ```
    *   In `ros2_ws/src/my_ros2_package/package.xml`, ensure `rclpy` and `std_msgs` dependencies are listed:
        ```xml
        <exec_depend>rclpy</exec_depend>
        <exec_depend>std_msgs</exec_depend>
        ```
5.  **Build the Workspace:**
    ```bash
    cd ros2_ws
    colcon build
    ```
6.  **Source the Workspace:**
    ```bash
    source install/setup.bash
    ```
7.  **Run the Nodes:**
    Open two separate terminals, source the workspace in both, and run:
    *   Terminal 1 (Publisher): `ros2 run my_ros2_package simple_publisher`
    *   Terminal 2 (Subscriber): `ros2 run my_ros2_package simple_subscriber`

## Exercises

1.  Modify the `simple_publisher.py` to publish a different type of message (e.g., `Int32` from `std_msgs.msg`). You'll need to adapt the subscriber accordingly.
2.  Experiment with the `timer_period` in the publisher. How does changing it affect the subscriber's output rate?
3.  Introduce an error in the `simple_subscriber.py` (e.g., misspell the topic name). Observe the output and understand how ROS 2 handles such discrepancies.
4.  Research and explain the purpose of the `10` in `self.create_publisher(String, 'topic', 10)` and `self.create_subscription(String, 'topic', self.listener_callback, 10)`. (Hint: Quality of Service settings).

## Summary

This chapter provided a foundational understanding of ROS 2, its core concepts, and its importance in modern robotics. Through a hands-on example of a publisher-subscriber system, you've seen how ROS 2 nodes communicate, forming the building blocks of complex robotic applications. The exercises encouraged further exploration into message types, communication patterns, and Quality of Service, setting the stage for deeper dives into specific ROS 2 functionalities.