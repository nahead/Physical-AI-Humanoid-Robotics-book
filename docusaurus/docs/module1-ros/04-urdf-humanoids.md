# Understanding URDF (Unified Robot Description Format) for Humanoids

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. It's a crucial component for defining the kinematics, dynamics, visual appearance, and collision properties of a robot, enabling simulation, visualization, and motion planning. This chapter focuses on applying URDF to describe humanoid robots, which present unique challenges due to their complex articulated structure.

## Concepts

### What is URDF?

URDF is a standard way to describe a robot as a collection of **links** (rigid bodies) connected by **joints** (allowing relative motion between links). Each link can have a visual representation, collision properties, and inertial properties. Joints define the type of motion allowed (e.g., revolute, prismatic, fixed) and their limits.

Key elements of URDF:
- **`<robot>` tag:** The root element, containing the entire robot description.
- **`<link>` tag:** Defines a rigid body segment of the robot. It can contain:
    - **`<visual>`:** Describes the graphical appearance of the link (e.g., mesh file, color).
    - **`<collision>`:** Defines the collision geometry of the link.
    - **`<inertial>`:** Specifies the mass, center of mass, and inertia matrix of the link.
- **`<joint>` tag:** Defines the kinematic and dynamic properties of a connection between two links. It includes:
    - **`parent` and `child` links:** Specifies which links are connected.
    - **`type`:** The type of joint (e.g., `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`).
    - **`origin`:** The transform from the parent link to the child link.
    - **`axis`:** The axis of rotation for revolute/continuous joints or translation for prismatic joints.
    - **`limit`:** Joint limits (upper, lower, velocity, effort).

### URDF for Humanoid Robots

Humanoid robots are highly articulated systems, typically featuring a torso, head, two arms, and two legs, mimicking human anatomy. Describing such a complex system in URDF requires careful attention to:
- **Kinematic Chains:** Defining numerous links and joints that form complex kinematic chains for legs and arms.
- **Degrees of Freedom (DoF):** Humanoids possess many DoF, which must be accurately modeled to enable precise motion.
- **Balance and Stability:** While URDF primarily describes geometry and kinematics, the inertial properties are crucial inputs for dynamic simulation and balance control algorithms.
- **Mesh Integration:** Humanoid robots often use detailed 3D mesh models for their visual representation, requiring proper scaling and placement within the URDF.

### Xacro: Simplifying URDF

Writing complex URDF files manually can be tedious and prone to errors. **Xacro (XML Macros)** is an XML macro language that allows for more concise and maintainable URDF files. It supports:
- **Macros:** Defining reusable blocks of URDF code.
- **Properties:** Using variables to define values (e.g., joint limits, link dimensions), making it easier to change parameters.
- **Mathematics:** Performing simple mathematical operations to calculate values.

## Working Code Examples

### Example 1: Basic Humanoid Leg Segment (URDF)

This example shows a simplified URDF description of a single leg segment, demonstrating links and joints.

#### `simple_leg.urdf`

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_leg">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1" />
      </material>
    </visual>
  </link>

  <link name="upper_leg_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0" />
      <mass value="1.0" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
  </link>

  <joint name="hip_joint" type="revolute">
    <parent link="base_link" />
    <child link="upper_leg_link" />
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="1.0" />
  </joint>

</robot>
```

### Example 2: Using Xacro for a Reusable Joint Macro

This example demonstrates how Xacro can simplify URDF by creating a reusable macro for a generic revolute joint.

#### `robot.xacro` (Main file)

```xml
<?xml version="1.0"?>
<robot name="humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define a property for joint limit -->
  <xacro:property name="joint_limit_val" value="1.57" />

  <!-- Macro for a generic revolute joint -->
  <xacro:macro name="revolute_joint" params="parent_link_name child_link_name joint_name origin_xyz origin_rpy axis_xyz">
    <joint name="${joint_name}" type="revolute">
      <parent link="${parent_link_name}" />
      <child link="${child_link_name}" />
      <origin xyz="${origin_xyz}" rpy="${origin_rpy}" />
      <axis xyz="${axis_xyz}" />
      <limit lower="${-joint_limit_val}" upper="${joint_limit_val}" effort="100.0" velocity="1.0" />
    </joint>
  </xacro:macro>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1" />
      </material>
    </visual>
  </link>

  <link name="link_a">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5" />
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
  </link>

  <!-- Use the macro -->
  <xacro:revolute_joint
    parent_link_name="base_link"
    child_link_name="link_a"
    joint_name="joint_ba"
    origin_xyz="0 0 0.1"
    origin_rpy="0 0 0"
    axis_xyz="0 0 1"
  />

</robot>
```

### How to Run and Visualize URDF

1.  **Place URDF/Xacro Files:**
    Save `simple_leg.urdf` or `robot.xacro` into your ROS 2 package (e.g., `ros2_ws/src/my_ros2_package/urdf/`).

2.  **Convert Xacro to URDF (if applicable):**
    If you are using Xacro, you need to convert it to a pure URDF file first:
    ```bash
    ros2 run xacro xacro --inorder robot.xacro > robot.urdf
    ```

3.  **Visualize with `rviz2`:**
    ```bash
    # For a simple URDF file
    ros2 launch urdf_tutorial display.launch.py model:=simple_leg.urdf gui:=true

    # For a converted Xacro file
    ros2 launch urdf_tutorial display.launch.py model:=robot.urdf gui:=true
    ```
    (Note: You might need to install `urdf_tutorial` package: `sudo apt install ros-humble-urdf-tutorial`).
    This will open `rviz2`, allowing you to see the robot model and manipulate its joints using a GUI.

## Exercises

1.  **Expand `simple_leg.urdf`:** Add a `lower_leg_link` and a `knee_joint` to the `simple_humanoid_leg` robot. Define appropriate visual, collision, and inertial properties.
2.  **Create a Full Humanoid Arm (Xacro):** Using Xacro macros, define a reusable `joint_and_link` macro. Use it to construct a simple humanoid arm with a shoulder, elbow, and wrist joint.
3.  **Add `gazebo_ros2_control`:** Research how to integrate `gazebo_ros2_control` into a URDF to allow simulated joints to be controlled via ROS 2 topics. This is a crucial step for controlling your simulated humanoid.
4.  **Mesh Integration:** Replace the primitive geometries (box, cylinder) in your URDF with 3D mesh files (e.g., `.stl` or `.dae`).

## Summary

This chapter provided a foundational understanding of URDF and Xacro, essential tools for describing robot hardware in ROS 2. You learned how to define links and joints, specify their properties, and visualize robot models. The focus on humanoids highlighted the challenges and considerations for highly articulated systems. Through practical examples and exercises, you've gained the skills to create and manage complex robot descriptions, which is vital for both simulation and real-world robot deployment. This knowledge is a prerequisite for understanding how to control and simulate humanoid robots effectively.