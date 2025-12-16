# ROS 2 Nodes, Topics, and Services

In ROS 2, the communication system is built around a decentralized architecture where individual processes, called **nodes**, interact with each other through various communication patterns. This modularity is a cornerstone of ROS, allowing for flexible and robust robot software development. This chapter will delve into the core communication mechanisms: Topics, Services, and Actions.

## Concepts

### Nodes

A **Node** is an executable process that performs computations. In a ROS 2 system, a robot's capabilities are typically broken down into many small, specialized nodes. For example, one node might control the robot's motors, another might process camera data, and yet another might plan navigation paths. This modular approach enhances reusability, simplifies debugging, and allows for parallel development.

Each node in ROS 2 needs a unique name within the ROS graph to prevent conflicts and ensure proper routing of messages.

### Topics

**Topics** are the most common method of asynchronous, many-to-many, publish-subscribe communication in ROS 2. Nodes publish messages to a named topic, and other nodes subscribe to that same topic to receive those messages. This is a one-way broadcast mechanism, where publishers don't know who their subscribers are, and subscribers don't know who their publishers are.

Key characteristics of Topics:
- **Asynchronous:** Publishers and subscribers operate independently.
- **Many-to-many:** Multiple publishers can send messages to a single topic, and multiple subscribers can receive messages from it.
- **Message Type:** Each topic has a defined message type (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/LaserScan`), ensuring data consistency.
- **Quality of Service (QoS):** ROS 2 allows configuring QoS policies for topics, enabling developers to fine-tune communication reliability, latency, and durability based on application requirements.

### Services

**Services** provide a synchronous, one-to-one, request-reply communication model. When a node (client) needs to request a specific operation from another node (server) and expects an immediate response, it uses a service. This is ideal for operations that are performed infrequently and require a direct result.

Key characteristics of Services:
- **Synchronous:** The client blocks until it receives a reply from the server.
- **One-to-one:** A single client makes a request to a single server.
- **Request/Response Pair:** Services define a request message and a response message.
- **Deterministic:** Useful for deterministic operations where the client needs to know the outcome of its request immediately.

## Working Code Examples

### Example 1: ROS 2 Publisher and Subscriber (Python)

Building upon the previous introduction, here's a detailed example of a publisher and subscriber in Python.

#### Publisher (`my_publisher_node.py`)

This node publishes "Hello ROS 2!" messages with a counter.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0
        self.get_logger().info('MyPublisher node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! This is message number: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MyPublisher node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber (`my_subscriber_node.py`)

This node subscribes to the topic and prints the received messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.get_logger().info('MySubscriber node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MySubscriber node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: ROS 2 Service Client and Server (Python)

This example demonstrates a service where a client requests a sum of two integers and a server provides the result.

#### Server (`simple_add_server.py`)

This node provides an `add_two_ints` service.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Standard ROS 2 service message

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts service server has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Sending back sum: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('AddTwoInts service server stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Client (`simple_add_client.py`)

This node requests the `add_two_ints` service.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = AddTwoInts.Request()
        self.get_logger().info('AddTwoInts service client has been started.')

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f'Requesting sum of {a} and {b}...')


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print('Usage: ros2 run <package_name> simple_add_client <int_a> <int_b>')
        return

    node = AddTwoIntsClient()
    node.send_request(int(sys.argv[1]), int(sys.argv[2]))

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().error(f'Service call failed: {e}')
            else:
                node.get_logger().info(f'Result of add_two_ints: {response.sum}')
            break
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run Examples

1.  **Create a ROS 2 Workspace and Python Package:**
    (Same as in the previous chapter, ensure you have a workspace like `ros2_ws/src/my_ros2_package`).

2.  **Place Code:**
    *   For Publisher/Subscriber: Save `my_publisher_node.py` and `my_subscriber_node.py` in `ros2_ws/src/my_ros2_package/my_ros2_package/`.
    *   For Service Client/Server: Save `simple_add_server.py` and `simple_add_client.py` in the same directory.

3.  **Update `setup.py` and `package.xml`:**
    *   In `ros2_ws/src/my_ros2_package/setup.py`, ensure all `console_scripts` entry points are added:
        ```python
        entry_points={
            'console_scripts': [
                'my_publisher = my_ros2_package.my_publisher_node:main',
                'my_subscriber = my_ros2_package.my_subscriber_node:main',
                'add_server = my_ros2_package.simple_add_server:main',
                'add_client = my_ros2_package.simple_add_client:main',
            ],
        },
        ```
    *   In `ros2_ws/src/my_ros2_package/package.xml`, ensure `rclpy`, `std_msgs`, and `example_interfaces` dependencies are listed:
        ```xml
        <exec_depend>rclpy</exec_depend>
        <exec_depend>std_msgs</exec_depend>
        <exec_depend>example_interfaces</exec_depend>
        ```

4.  **Build and Source the Workspace:**
    ```bash
    cd ros2_ws
    colcon build
    source install/setup.bash
    ```

5.  **Run Publisher/Subscriber:**
    *   Terminal 1 (Publisher): `ros2 run my_ros2_package my_publisher`
    *   Terminal 2 (Subscriber): `ros2 run my_ros2_package my_subscriber`

6.  **Run Service Client/Server:**
    *   Terminal 1 (Server): `ros2 run my_ros2_package add_server`
    *   Terminal 2 (Client): `ros2 run my_ros2_package add_client 5 7` (This will request 5 + 7 from the server)

## Exercises

1.  **Modify Topic Communication:** Change the message type of `my_topic` to `std_msgs/msg/Int32`. Update both the publisher and subscriber to handle integer messages.
2.  **Implement a Custom Message:** Define a custom ROS 2 message type (e.g., `RobotPose` with `x, y, z, roll, pitch, yaw` fields). Create a publisher and subscriber for this custom message.
3.  **Error Handling in Services:** Modify the `add_two_ints_callback` in the server to intentionally raise an error under certain conditions (e.g., if `request.a` is negative). Observe how the client handles this.
4.  **Asynchronous Service Client:** Research how to make a service client non-blocking, allowing it to perform other tasks while waiting for a service response.
5.  **Explore QoS Settings:** Experiment with different QoS profiles for your topic. For instance, set `rclpy.qos.QoSProfile(depth=1)` and `rclpy.qos.ReliabilityPolicy.RELIABLE` for the publisher and `rclpy.qos.ReliabilityPolicy.BEST_EFFORT` for the subscriber. How does this affect message delivery?

## Summary

This chapter provided a comprehensive overview of ROS 2's core communication patterns: Nodes, Topics, and Services. You've learned how to create and run publisher-subscriber systems for asynchronous data streaming and client-server setups for synchronous request-reply interactions. Through practical Python examples and engaging exercises, you've gained a solid foundation in building modular and communicative robot software components in ROS 2. This understanding is crucial for developing more complex robotic behaviors and integrating various functionalities.