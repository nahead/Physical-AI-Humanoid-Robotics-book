# Simulating Physics, Gravity, and Collisions in Gazebo

Gazebo is a powerful 3D robot simulator that accurately models the physics of the real world. It allows roboticists to test algorithms, design robots, and perform various experiments in a safe, repeatable virtual environment. Understanding how Gazebo handles physics, gravity, and collisions is fundamental to creating realistic and reliable robotic simulations.

## Concepts

### Gazebo's Physics Engine

Gazebo is not a physics engine itself, but rather integrates various high-performance physics engines. Historically, it has relied heavily on ODE (Open Dynamics Engine) and, in recent versions, has added support for Bullet, DART, and Simbody. The choice of physics engine can impact the accuracy and performance of your simulations.

Key aspects of Gazebo's physics simulation:
- **World File (`.world`):** Defines the simulation environment, including models, lights, sensors, and physics properties.
- **Models (`.sdf` or `.urdf`):** Describe individual robots or objects, including their links (rigid bodies), joints, visual characteristics, and collision geometries.
- **Simulation Time:** Gazebo operates on its own simulation clock, which can run faster or slower than real-time depending on computational load.
- **Physics Parameters:** Configurable parameters within the `.world` file that control aspects like gravity, solver type, time steps, and more.

### Gravity

Gravity is a force that acts on all objects with mass. In Gazebo, gravity is a global parameter defined in the physics section of the `.world` file. By default, it simulates Earth's gravity, pulling objects downwards.

**Configuring Gravity in `.world` file:**

```xml
<physics name="default_physics" default="0" type="ode">
  <gravity>0 0 -9.81</gravity> <!-- x, y, z components of gravity -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
      <erp>0.2</erp>
    </solver>
    <constraints>
      <cfm>0</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

### Collisions

Collision detection and response are critical for any robot simulation. Gazebo uses the collision geometries defined within your robot models to detect when two objects are interpenetrating. When a collision is detected, the physics engine applies forces to prevent further interpenetration, simulating a physical interaction.

**Defining Collision Geometry:**

Within a `<link>` tag of an `.sdf` or `.urdf` file, the `<collision>` tag specifies the geometry used for collision detection. It's often simpler than the `<visual>` geometry to reduce computational overhead.

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

**Friction and Restitution:** These material properties affect how objects interact during collisions.
- **Friction:** Opposes relative motion between surfaces in contact.
- **Restitution (Bounciness):** Determines how much kinetic energy is conserved after a collision.

These can be defined in `<surface>` tags within `<collision>` or as `<material>` properties.

## Working Code Examples

### Example 1: Creating a Simple Gazebo World with a Falling Box

This example demonstrates how to create a basic `.world` file where a box falls onto a ground plane under gravity.

#### `falling_box.world`

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <gravity>0 0 -9.81</gravity>
    <physics name="default_physics" default="0" type="ode">
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
          <erp>0.2</erp>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <cast_shadows>1</cast_shadows>
      <direction>-0.5 -0.5 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
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
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="falling_box">
      <pose>0 0 5 0 0 0</pose> <!-- Start 5 meters above ground -->
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
        </inertial>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
            <specular>0 0 1 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

#### How to Run

1.  **Save the file:** Save the above XML content as `falling_box.world` in a convenient location (e.g., `ros2_ws/src/my_gazebo_worlds/worlds/`).
2.  **Launch Gazebo:**
    ```bash
    gazebo --verbose falling_box.world
    ```
    You should see a blue box fall onto the gray ground plane under gravity.

### Example 2: ROS 2 Control of a Simple Physics Model

This example demonstrates how to spawn an SDF model in Gazebo and control its movement using ROS 2 commands. We will define a simple model with a joint and then publish velocity commands to it via a ROS 2 topic.

#### `simple_robot.sdf` (Model Definition)

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_robot">
    <link name="base_link">
      <inertial>
        <mass>0.1</mass>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
          <specular>0 1 0 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </collision>
    </link>

    <link name="wheel_link">
      <inertial>
        <mass>0.05</mass>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder radius="0.03" length="0.02"/>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>1 0 0 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder radius="0.03" length="0.02"/>
        </geometry>
      </collision>
    </link>

    <joint name="wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_link</child>
      <pose>0.05 0 0 0 1.5707 0</pose> <!-- Offset and orientation for wheel -->
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>

    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <robot_param>robot_description</robot_param>
      <robot_param_server>robot_description</robot_param_server>
      <control_period>0.01</control_period>
      <controller_manager_timeout>5</controller_manager_timeout>
    </plugin>

  </model>
</sdf>
```

#### `ros2_control_config.yaml` (Controller Configuration)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

forward_velocity_controller:
  ros__parameters:
    type: "velocity_controllers/JointGroupVelocityController"
    joints:
      - wheel_joint
```

#### Python ROS 2 Publisher to Control Wheel (`wheel_controller.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.velocity = 0.5 # radians/second
        self.get_logger().info('WheelController node started, publishing velocity commands.')

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [float(self.velocity)]
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing wheel velocity: {self.velocity}')

def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('WheelController node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### How to Run

1.  **Place Files:**
    *   Save `simple_robot.sdf` in `ros2_ws/src/my_gazebo_models/models/simple_robot/model.sdf`.
    *   Save `ros2_control_config.yaml` in `ros2_ws/src/my_ros2_package/config/`.
    *   Save `wheel_controller.py` in `ros2_ws/src/my_ros2_package/my_ros2_package/`.
2.  **Update `setup.py` and `package.xml` for `my_ros2_package`:**
    *   `setup.py`: Add `wheel_controller` entry point.
    *   `package.xml`: Add `ros2_control`, `ros2_controllers`, `gazebo_ros2_control` as `exec_depend`.
3.  **Build and Source:** `colcon build` and `source install/setup.bash`.
4.  **Launch Gazebo with the model:**
    ```bash
    ros2 launch gazebo_ros2_control_demos diffdrive_system_with_controller.launch.py \
      use_sim_time:=true \
      robot_description_content:=$(cat simple_robot.sdf) \
      controllers_file:=ros2_control_config.yaml
    ```
    (Note: You might need to install `gazebo_ros2_control_demos` for the launch file, or create your own simple launch file.)
5.  **Run the controller:**
    ```bash
    ros2 run my_ros2_package wheel_controller
    ```
    You should see the wheel of the `simple_robot` model rotating in Gazebo.

## Diagrams

### Diagram 1: Gazebo Physics Loop

```
+----------------+       +-------------------+       +--------------------+
|  World File    |------>| Physics Engine    |------>| Collision Detector |
|  (.world)      |       | (ODE, Bullet)     |       |                    |
+----------------+       +-------------------+       +--------------------+
       ^                        |                             |
       |                        V                             V
+----------------+       +-------------------+       +--------------------+
| Model Files    |<------| Actuator Outputs  |<------| Force Calculator   |
| (.sdf, .urdf)  |       |                   |       | (Gravity, Contacts)|
+----------------+       +-------------------+       +--------------------+
       |                                                    ^
       |                                                    |
       +----------------------------------------------------+
       (Defines Links, Joints, Visuals, Collisions, Inertia)
```

## Exercises

1.  **Change Gravity:** Modify `falling_box.world` to simulate gravity on the Moon (`0 0 -1.62`). Observe the difference in how the box falls.
2.  **Add Multiple Objects:** Extend `falling_box.world` to include multiple boxes of different masses and sizes. Observe their interactions.
3.  **Friction Experiment:** Modify the `ground_plane` and `falling_box` surfaces to have different friction coefficients (e.g., `mu=0.1` vs `mu=5.0`). Simulate an initial horizontal velocity on the box and observe the sliding behavior.
4.  **ROS 2 Control of a Humanoid Joint:** Adapt Example 2 to control a specific joint of a humanoid robot (e.g., the hip joint from the URDF example in Module 1). Publish target positions or velocities to it.

## Summary

This chapter laid the groundwork for effective robotics simulation in Gazebo. You've gained a fundamental understanding of how Gazebo models physics, applies gravity, and handles complex collision interactions. Through hands-on examples, you learned to create custom world files, define robot models with appropriate physics properties, and even interface with ROS 2 for dynamic control. This knowledge is crucial for building and testing realistic robot behaviors in a virtual environment before deploying them to physical hardware.
