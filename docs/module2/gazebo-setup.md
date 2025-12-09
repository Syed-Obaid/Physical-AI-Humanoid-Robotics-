---
id: gazebo-setup
title: "Chapter 2.2: Gazebo Setup"
sidebar_label: "2.2 Gazebo Setup"
sidebar_position: 2
description: Install Gazebo Garden and integrate with ROS 2 for robot simulation
keywords: [Gazebo Garden, installation, ros_gz_bridge, SDF, world files]
---

# Chapter 2.2: Gazebo Setup

## Learning Objectives

By the end of this chapter, you will be able to:

1. Install Gazebo Garden on Ubuntu 22.04 and verify the installation
2. Create SDF world files with physics properties, lighting, and models
3. Use `ros_gz_bridge` to connect Gazebo topics to ROS 2
4. Launch simulated robots with ROS 2 launch files

## Prerequisites

### Required Knowledge
- ROS 2 workspace management (colcon build, sourcing)
- URDF robot modeling basics
- Linux package management (apt)

### Previous Chapters
- [Chapter 2.1: Overview](./overview.md) - Digital twin concepts

## Content

### Installing Gazebo Garden

Gazebo Garden is the Long-Term Support (LTS) release compatible with ROS 2 Humble.

**Step 1: Add Gazebo repository**
```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable \
  $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

**Step 2: Install Gazebo Garden**
```bash
sudo apt update
sudo apt install gz-garden -y
```

**Step 3: Verify installation**
```bash
gz sim --version
# Expected output: Gazebo Sim, version 7.x.x
```

**Step 4: Install ROS 2 integration packages**
```bash
sudo apt install ros-humble-ros-gz -y
```

This installs `ros_gz_bridge`, `ros_gz_sim`, and `ros_gz_image` for ROS 2 ↔ Gazebo communication.

### SDF World Files

Gazebo uses **SDF (Simulation Description Format)** XML files to define simulation worlds. Unlike URDF (robot-centric), SDF describes entire environments.

**Minimal World Example** (`empty_world.sdf`):
```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="empty_world">
    <!-- Physics settings -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
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
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Key SDF Elements**:
- `<physics>`: Timestep (1ms typical for robotics), real-time factor (1.0 = run at wall-clock speed)
- `<light>`: Directional sun with shadow casting
- `<model>`: Static ground plane with collision and visual geometry

**Launch Gazebo with world**:
```bash
gz sim empty_world.sdf
```

### ROS 2 - Gazebo Bridge

The `ros_gz_bridge` translates messages between Gazebo topics and ROS 2 topics.

**Bridge Configuration Example**:
```yaml
# config/gz_ros_bridge.yaml
- topic_name: "/model/robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"

- topic_name: "/model/robot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
```

**Launch bridge node**:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

This creates bidirectional topic mapping: ROS 2 `/cmd_vel` ↔ Gazebo `/model/robot/cmd_vel`.

### Spawning URDF in Gazebo

To spawn a ROS 2 URDF model in Gazebo:

**Step 1: Convert URDF to SDF**
```bash
gz sdf -p robot.urdf > robot.sdf
```

**Step 2: Spawn model via service**
```bash
ros2 run ros_gz_sim create -entity my_robot -file robot.sdf
```

**Alternative: Python launch file** (recommended):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        Node(
            package='ros_gz_sim',
            executable='gzserver',
            arguments=['empty_world.sdf'],
            output='screen'
        ),
        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-entity', 'robot',
                      '-file', 'robot.sdf',
                      '-x', '0', '-y', '0', '-z', '0.5'],
            output='screen'
        ),
        # Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        )
    ])
```

### Troubleshooting

**Issue 1: "gz: command not found"**
- Solution: Source Gazebo environment: `source /usr/share/gazebo/setup.bash`

**Issue 2: Models not loading**
- Solution: Set model path: `export GZ_SIM_RESOURCE_PATH=/path/to/models:$GZ_SIM_RESOURCE_PATH`

**Issue 3: Bridge topics not appearing**
- Solution: Check topic names match exactly (case-sensitive): `gz topic -l` and `ros2 topic list`

## Summary

### Key Takeaways
- **Gazebo Garden** is the LTS version for ROS 2 Humble (installed via apt)
- **SDF files** define simulation worlds with physics, lighting, and models
- **ros_gz_bridge** connects Gazebo and ROS 2 via topic mapping
- **Launch files** orchestrate Gazebo server, model spawning, and bridge nodes

### What's Next
In Chapter 2.3, you'll add realistic sensors (cameras, lidar, IMU) to simulated robots.

## Exercises

1. **Basic World Creation**: Create an SDF world with a textured ground plane and two point lights
2. **Bridge Setup**: Launch a bridge for `/joint_states` topic and verify data flow with `ros2 topic echo`
3. **URDF Spawn**: Convert the 3-DOF arm from Module 1 to SDF and spawn it at position (1, 2, 0.5)

## References

- Open Robotics. (2023). *Gazebo Garden installation guide*. Retrieved December 8, 2025, from https://gazebosim.org/docs/garden/install_ubuntu
- Open Robotics. (2023). *SDF specification*. Retrieved December 8, 2025, from http://sdformat.org/spec
- Open Robotics. (2023). *ros_gz integration*. Retrieved December 8, 2025, from https://github.com/gazebosim/ros_gz

---

**Word Count**: ~900 words
