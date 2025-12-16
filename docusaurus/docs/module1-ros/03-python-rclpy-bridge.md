# Bridging Python Agents to ROS Controllers using rclpy

Python is a popular language in robotics due to its simplicity, extensive libraries for AI/ML, and rapid prototyping capabilities. `rclpy` is the Python client library for ROS 2, providing a convenient and powerful way to integrate Python-based agents and algorithms with the ROS 2 ecosystem. This chapter explores how to effectively use `rclpy` to bridge your Python code with ROS 2 controllers and components.

## Concepts

### `rclpy`: The Python ROS 2 Client Library

`rclpy` provides Python bindings for the core ROS 2 C++ client library (`rclcpp`). It allows Python developers to write ROS 2 nodes, publishers, subscribers, service clients, and service servers using Python. This means you can leverage Python's strengths (e.g., NumPy, SciPy, TensorFlow, PyTorch) directly within your ROS 2 applications.

Key functionalities provided by `rclpy`:
- **Node Creation:** Instantiate `Node` objects to represent your Python components in the ROS graph.
- **Publisher/Subscriber:** Create publishers to send data to topics and subscribers to receive data from topics.
- **Service Client/Server:** Implement client-server communication for request-reply patterns.
- **Parameters:** Access and manage ROS 2 parameters from your Python nodes.
- **Timers:** Schedule periodic callbacks for repetitive tasks.
- **Executors:** Manage the execution of callbacks (single-threaded or multi-threaded).

### Bridging Python Agents and ROS Controllers

Many advanced robotic behaviors, especially in AI, are developed as Python "agents" (e.g., reinforcement learning agents, behavioral planners). These agents often need to interact with low-level robot controllers, which are typically implemented as ROS 2 nodes. `rclpy` serves as the essential bridge, allowing Python agents to:
- **Sense:** Subscribe to sensor data (e.g., camera feeds, LiDAR scans) from ROS 2 topics.
- **Act:** Publish commands (e.g., joint velocities, twist commands) to ROS 2 topics or call ROS 2 services to control the robot.
- **Plan:** Communicate high-level plans or goals to other ROS 2 planning nodes.

## Working Code Examples

### Example 1: Python Agent Publishing Joint Commands

This example simulates a simple Python agent that calculates a desired joint position and publishes it to a ROS 2 topic. A ROS 2 controller (which would be a separate C++ or Python node) would then subscribe to this topic and actuate the robot.

#### `joint_command_publisher.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(Float64, 'robot/joint1_position_command', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # Publish every 100ms
        self.desired_position = 0.0
        self.increment = 0.01
        self.get_logger().info('JointCommandPublisher node has been started.')

    def timer_callback(self):
        msg = Float64()
        msg.data = self.desired_position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint command: {msg.data:.2f}')

        # Simulate a simple agent changing the desired position
        self.desired_position += self.increment
        if self.desired_position > 1.0 or self.desired_position < -1.0:
            self.increment *= -1 # Reverse direction
            
def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('JointCommandPublisher node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Python Agent Subscribing to Sensor Data and Calling Service

This agent subscribes to a simulated sensor (e.g., a simple range sensor) and, if an obstacle is detected, calls a service to request a stop.

#### `obstacle_avoidance_agent.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 # Simulate a range sensor reading
from std_srvs.srv import Trigger # Standard service for simple actions

class ObstacleAvoidanceAgent(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_agent')
        self.subscription = self.create_subscription(
            Float32,
            'robot/range_sensor_data',
            self.range_sensor_callback,
            10)
        self.client = self.create_client(Trigger, 'robot/stop_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stop command service not available, waiting...')
        self.get_logger().info('ObstacleAvoidanceAgent node has been started.')
        self.obstacle_distance_threshold = 0.5 # meters

    def range_sensor_callback(self, msg):
        current_range = msg.data
        self.get_logger().info(f'Received range sensor data: {current_range:.2f} m')
        
        if current_range < self.obstacle_distance_threshold:
            self.get_logger().warn(f'Obstacle detected at {current_range:.2f} m! Requesting stop.')
            self.send_stop_command()

    def send_stop_command(self):
        request = Trigger.Request()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.stop_command_response_callback)

    def stop_command_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Robot stop command successful.')
            else:
                self.get_logger().error(f'Robot stop command failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ObstacleAvoidanceAgent node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run Examples

1.  **Create/Utilize ROS 2 Workspace and Python Package:**
    (Ensure your `ros2_ws/src/my_ros2_package` is set up as described in previous chapters).

2.  **Place Code:**
    *   Save `joint_command_publisher.py` and `obstacle_avoidance_agent.py` in `ros2_ws/src/my_ros2_package/my_ros2_package/`.

3.  **Update `setup.py` and `package.xml`:**
    *   In `ros2_ws/src/my_ros2_package/setup.py`, add new entry points:
        ```python
        entry_points={
            'console_scripts': [
                'joint_command_publisher = my_ros2_package.joint_command_publisher:main',
                'obstacle_avoidance_agent = my_ros2_package.obstacle_avoidance_agent:main',
                # ... existing entry points ...
            ],
        },
        ```
    *   In `ros2_ws/src/my_ros2_package/package.xml`, ensure necessary dependencies (`std_msgs`, `std_srvs`) are listed:
        ```xml
        <exec_depend>rclpy</exec_depend>
        <exec_depend>std_msgs</exec_depend>
        <exec_depend>std_srvs</exec_depend>
        <exec_depend>example_interfaces</exec_depend>
        ```

4.  **Build and Source the Workspace:**
    ```bash
    cd ros2_ws
    colcon build
    source install/setup.bash
    ```

5.  **Run `joint_command_publisher`:**
    ```bash
    ros2 run my_ros2_package joint_command_publisher
    ```
    This will continuously publish desired joint positions.

6.  **Simulate `robot/stop_command` Service and `robot/range_sensor_data` Topic (for `obstacle_avoidance_agent`):**
    You'll need to create temporary nodes to simulate these for testing.

    #### Temporary Stop Service Server (`mock_stop_server.py`)
    ```python
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    class MockStopService(Node):
        def __init__(self):
            super().__init__('mock_stop_server')
            self.srv = self.create_service(Trigger, 'robot/stop_command', self.stop_callback)
            self.get_logger().info('Mock Stop Command service server started.')

        def stop_callback(self, request, response):
            self.get_logger().info('Received stop command!')
            response.success = True
            response.message = 'Robot stopped successfully.'
            return response

    def main(args=None):
        rclpy.init(args=args)
        node = MockStopService()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

    #### Temporary Range Sensor Publisher (`mock_range_sensor.py`)
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32
    import time

    class MockRangeSensor(Node):
        def __init__(self):
            super().__init__('mock_range_sensor')
            self.publisher_ = self.create_publisher(Float32, 'robot/range_sensor_data', 10)
            self.timer = self.create_timer(1.0, self.timer_callback) # Publish every 1 second
            self.range_data = 1.0
            self.get_logger().info('MockRangeSensor node started.')

        def timer_callback(self):
            msg = Float32()
            # Simulate an obstacle appearing then disappearing
            if self.get_clock().now().nanoseconds % (10 * 10**9) < (5 * 10**9): # For 5 seconds every 10 seconds
                self.range_data = 0.3 # Simulate obstacle close
            else:
                self.range_data = 1.0 # No obstacle
            
            msg.data = self.range_data
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing mock range: {msg.data:.2f} m')

    def main(args=None):
        rclpy.init(args=args)
        node = MockRangeSensor()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

    *   **Add Mock Nodes to `setup.py`:**
        ```python
        entry_points={
            'console_scripts': [
                # ... existing entry points ...
                'mock_stop_server = my_ros2_package.mock_stop_server:main',
                'mock_range_sensor = my_ros2_package.mock_range_sensor:main',
            ],
        },
        ```
    *   **Rebuild and Source:** `colcon build` and `source install/setup.bash`.
    *   **Run `obstacle_avoidance_agent`:**
        *   Terminal 1: `ros2 run my_ros2_package mock_stop_server`
        *   Terminal 2: `ros2 run my_ros2_package mock_range_sensor`
        *   Terminal 3: `ros2 run my_ros2_package obstacle_avoidance_agent`
        Observe the agent reacting to the simulated range data and calling the stop service.

## Exercises

1.  **Refine Joint Command:** Modify `joint_command_publisher.py` to publish commands for multiple joints, potentially simulating a simple gait for a humanoid.
2.  **Add Emergency Stop:** Extend `obstacle_avoidance_agent.py` to implement an emergency stop logic. If an obstacle is detected within a critical distance, it should not only call the stop service but also publish a "critical_stop" message to a new topic.
3.  **Service with Data:** Create a new service that allows the Python agent to request the current robot pose (e.g., a custom `GetPose` service).
4.  **Error Resilience:** Enhance the `obstacle_avoidance_agent.py` to gracefully handle cases where the `robot/stop_command` service is not available.

## Summary

This chapter highlighted the power of `rclpy` in enabling seamless integration between Python-based AI agents and the ROS 2 ecosystem. You've learned how to develop Python nodes that publish control commands, subscribe to sensor data, and interact with services, forming intelligent behaviors for robots. The examples demonstrated practical bridging mechanisms, while the exercises encouraged you to build more complex and robust Python-ROS 2 interactions, laying the groundwork for developing sophisticated autonomous systems.