# High-Fidelity Rendering and Human-Robot Interaction in Unity

Unity is a powerful cross-platform game engine that has found increasing utility in robotics simulation due to its advanced rendering capabilities, robust physics engine, and rich ecosystem for interactive experiences. This chapter explores how to leverage Unity for high-fidelity visualization, realistic environment creation, and engaging human-robot interaction (HRI) simulations.

## Concepts

### Unity's Rendering Pipeline

Unity excels in creating visually stunning 3D environments. It offers various rendering pipelines that allow for photorealistic graphics, advanced lighting, post-processing effects, and detailed material rendering. This is particularly valuable for:
- **Vision-based AI:** Training computer vision models with synthetic data that closely resembles real-world camera feeds.
- **Human-Robot Interaction (HRI):** Creating immersive and intuitive interfaces for human operators to interact with virtual robots.
- **Marketing & Communication:** Producing high-quality visualizations of robot designs and capabilities.

### Physics in Unity (Unity Physics/PhysX)

While Unity is a game engine, its physics capabilities are robust enough for many robotics applications. Unity can utilize both its own optimized **Unity Physics** (DOTS-based) or the well-established **NVIDIA PhysX** engine. These engines handle rigid body dynamics, collision detection, and realistic physical interactions between objects.

### Human-Robot Interaction (HRI) in Simulation

Unity provides a rich set of tools for developing interactive experiences, which directly translates to advanced HRI simulations:
- **User Interface (UI):** Create custom dashboards, control panels, and feedback mechanisms for users to interact with robots.
- **Input Systems:** Integrate various input devices (keyboard, mouse, joysticks, VR/AR controllers) to command and teleoperate robots.
- **Animation Systems:** Implement complex robot animations for realistic movements and expressions, enhancing communication with humans.
- **Real-time Feedback:** Visualize sensor data, robot states, and mission progress in real-time within the simulated environment.

### ROS 2 Integration with Unity

Unity can be seamlessly integrated with ROS 2, allowing it to act as a powerful visualization and control frontend for ROS-based robot systems. Key components for this integration include:
- **`ROS-TCP-Connector`:** A Unity package that facilitates communication between Unity and ROS 2 using TCP.
- **`ros2-for-unity`:** A module that enables native ROS 2 node creation within Unity, allowing direct topic, service, and action communication.
- **URDF Importer:** Tools to import robot models defined in URDF format directly into Unity.

## Working Code Examples

### Example 1: Simple Robot Arm in Unity with ROS 2 Control

This example demonstrates how to create a basic robot arm model in Unity and control its joint positions via ROS 2 topics.

#### **Unity Setup:**

1.  **Create a New 3D Project** in Unity.
2.  **Install `ROS-TCP-Connector` and `ros2-for-unity`:** Follow the official documentation to install these packages via Unity's Package Manager.
3.  **Create a Simple Robot Arm:**
    *   Create empty GameObject (e.g., "RobotArm").
    *   Add a "Base" cube.
    *   Add an "UpperArm" cube, parented to "Base". Add a `HingeJoint` component to "UpperArm" and configure its `Connected Body` to "Base".
    *   Add a "ForeArm" cube, parented to "UpperArm". Add a `HingeJoint` component to "ForeArm" and configure its `Connected Body` to "UpperArm".
4.  **Add `ROSConnection` Component:** Create an empty GameObject named `ROSConnection` and add the `ROSConnection` script from the `ROS-TCP-Connector` package. Configure the ROS IP and Port (e.g., `127.0.0.1`, `10000`).
5.  **Create a Joint Control Script (`JointController.cs`):** Attach this script to each joint GameObject (e.g., "UpperArm", "ForeArm").

#### `JointController.cs` (C# Script in Unity)

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // For Float64 message type

public class JointController : MonoBehaviour
{
    public string rosTopicName = "joint_commands";
    private ArticulationBody articulationBody;
    private ROSConnection ros;

    void Start()
    {
        articulationBody = GetComponent<ArticulationBody>();
        if (articulationBody == null)
        {
            Debug.LogError("ArticulationBody not found on this GameObject.", this);
            enabled = false;
            return;
        }

        ros = ROSConnection.Get = ROSConnection.instance;
        if (ros == null)
        {
            Debug.LogError("ROSConnection not found. Make sure it's set up.", this);
            enabled = false;
            return;
        }

        ros.Subscribe<Float64Msg>(rosTopicName, ReceiveJointCommand);
        Debug.Log($"Subscribed to ROS topic: {rosTopicName}");
    }

    void ReceiveJointCommand(Float64Msg message)
    {
        if (articulationBody != null)
        {
            // Assuming this is a single revolute joint
            ArticulationDrive drive = articulationBody.xDrive;
            drive.target = (float)message.data * Mathf.Rad2Deg; // ROS uses radians, Unity HingeJoint uses degrees
            articulationBody.xDrive = drive;
            Debug.Log($"Received command for {rosTopicName}: {message.data}");
        }
    }
}
```
**Note:** For more complex robots, Unity's ArticulationBody is preferred for realistic joint physics. `HingeJoint` is simpler for quick demos.

#### `ros2_joint_publisher.py` (Python ROS 2 Node)

This script publishes a sinusoidal joint position command.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 # For single joint command

class ROS2JointPublisher(Node):
    def __init__(self):
        super().__init__('ros2_joint_publisher')
        self.publisher_ = self.create_publisher(Float64, 'joint_commands', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz
        self.time_ = 0.0
        self.get_logger().info('ROS2JointPublisher node has been started.')

    def timer_callback(self):
        msg = Float64()
        # Publish a sinusoidal command
        msg.data = 0.5 * (1 + rclpy.clock.Clock().now().seconds_nanoseconds[0] % 10 / 5) # Oscillates between 0 and 1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint command: {msg.data:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ROS2JointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ROS2JointPublisher node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run

1.  **Unity Editor:** Open your Unity project, ensure `ROSConnection` and `JointController` scripts are correctly set up and running in Play Mode.
2.  **ROS 2 Workspace:** Build and source your ROS 2 workspace (refer to Module 1 for setup).
3.  **Run ROS 2 Publisher:** In a terminal, run `ros2 run <your_package_name> ros2_joint_publisher`.
4.  Observe the robot arm's joint moving in the Unity editor in response to ROS 2 commands.

## Diagrams

### Diagram 1: Unity-ROS 2 Communication Flow

```
+------------------+     +------------------+     +------------------+
|                  |     |                  |     |                  |
|  Unity Simulator |<--->| ROS TCP Connector|<--->|  ROS 2 Network   |
| (Robot Arm Model)|     | (Unity Package)  |     | (Topics, Services)|
|                  |     |                  |     |                  |
+------------------+     +------------------+     +------------------+
                                                        ^
                                                        |
                                                  +------------------+
                                                  |                  |
                                                  | ROS 2 Nodes      |
                                                  | (Python/C++ Ctrls)|
                                                  |                  |
                                                  +------------------+
```

## Exercises

1.  **Advanced Joint Control:** Extend the `JointController.cs` script to control multiple joints using `Float64MultiArray` messages from ROS 2. Update the Python publisher to send commands for multiple joints.
2.  **Teleoperation Interface:** Create a simple UI in Unity (using Unity UI Canvas) with sliders or buttons to manually control the robot arm's joints. Use ROS 2 topics to send these commands to a separate ROS 2 node that interprets them.
3.  **Sensor Visualization:** Simulate a simple distance sensor in Unity (e.g., a raycast) and publish its readings to a ROS 2 topic (e.g., `sensor_msgs/msg/Range`). Create a ROS 2 node to subscribe to this data and visualize it (e.g., in `rviz2`).
4.  **URDF Import:** Import a complex URDF model (e.g., a humanoid robot) into Unity using the provided importer tools. Verify its visual and physical representation.

## Summary

This chapter demonstrated the power of Unity for high-fidelity rendering and human-robot interaction in robotics simulation. You learned how to integrate Unity with ROS 2, allowing for seamless communication and control of virtual robots. By creating interactive interfaces and visualizing robot behaviors, Unity provides an excellent platform for developing and testing complex robotic systems with a strong emphasis on visual realism and user experience. This skill is invaluable for tasks requiring photorealistic data generation, intuitive teleoperation, and engaging HRI research.