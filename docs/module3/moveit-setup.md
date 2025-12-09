---
id: moveit-setup
title: "Chapter 3.2: MoveIt 2 Setup"
sidebar_label: "3.2 MoveIt Setup"
sidebar_position: 2
description: Installing MoveIt 2 and configuring robot models with the Setup Assistant
keywords: [MoveIt 2, Setup Assistant, SRDF, planning groups, collision checking]
---

# Chapter 3.2: MoveIt 2 Setup

## Learning Objectives

By the end of this chapter, you will be able to:

1. Install MoveIt 2 for ROS 2 Humble
2. Use MoveIt Setup Assistant to configure a robot
3. Create SRDF (Semantic Robot Description Format) files
4. Define planning groups and end effectors
5. Configure self-collision checking

## Prerequisites

### Required Knowledge
- ROS 2 workspace management
- URDF robot modeling
- Linux package management (apt)

### Previous Chapters
- [Chapter 3.1: Motion Planning Overview](./overview.md)
- [Chapter 1.4: URDF Basics](../module1/urdf-basics.md)

## Content

### Installing MoveIt 2

MoveIt 2 is available as Debian packages for ROS 2 Humble on Ubuntu 22.04.

#### Installation Steps

```bash
# Update package lists
sudo apt update

# Install MoveIt 2 main package
sudo apt install ros-humble-moveit -y

# Install additional tools
sudo apt install ros-humble-moveit-visual-tools -y
sudo apt install ros-humble-moveit-planners-ompl -y
sudo apt install ros-humble-moveit-simple-controller-manager -y

# Install Setup Assistant
sudo apt install ros-humble-moveit-setup-assistant -y

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

#### Verify Installation

Test that MoveIt is installed correctly by launching the demo:

```bash
# Launch Panda robot demo
ros2 launch moveit2_tutorials demo.launch.py

# You should see RViz with the Panda robot arm
# Try dragging the interactive marker to plan and execute motions
```

**Expected Output**: RViz window opens with a Panda robot arm. You can drag the blue/green/red arrows (interactive markers) to set goal poses, then click "Plan" and "Execute" buttons.

### Understanding SRDF

The **SRDF (Semantic Robot Description Format)** augments URDF with motion planning-specific information:

- **Planning Groups**: Which joints/links form a kinematic chain for planning
- **End Effectors**: Which links are end effectors (grippers, hands)
- **Passive Joints**: Joints with no actuators (e.g., free-spinning wheels)
- **Disable Collisions**: Pairs of links that never collide or are always in contact
- **Virtual Joints**: Fixed transforms to world frame

**URDF vs SRDF**:
- **URDF**: Physical description (link geometry, joint axes, inertia)
- **SRDF**: Semantic information for motion planning

### MoveIt Setup Assistant Workflow

The Setup Assistant is a GUI tool that generates MoveIt configuration packages.

#### Step 1: Launch Setup Assistant

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

You'll see a wizard with these steps:

#### Step 2: Load URDF

Click "Create New MoveIt Configuration Package" → "Browse" and select your robot's URDF file.

**Example**: For a humanoid arm URDF at `~/ros2_ws/src/my_robot/urdf/arm.urdf`, navigate and select that file.

Click "Load Files". You should see your robot displayed in the 3D viewer.

#### Step 3: Define Self-Collision Matrix

Click "Self-Collisions" in the left panel.

- **Sampling Density**: Slider controls how thoroughly to check collisions (default 10,000 samples is usually sufficient)
- Click "Generate Collision Matrix"

The tool will move robot through random configurations and detect which link pairs never collide (e.g., shoulder link and wrist link are too far apart to collide).

**Result**: A list of disabled collision pairs (reduces collision checking computational cost).

**Important**: Don't disable adjacent links that might collide in extreme configurations. Review the matrix carefully.

#### Step 4: Create Planning Groups

Click "Planning Groups" → "Add Group".

**Example Configuration for Humanoid Arm**:

```
Group Name: arm
Kinematic Solver: kdl_kinematics_plugin/KDLKinematicsPlugin
Group Default Planner: RRTConnectkConfigDefault
Joints:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
  - wrist_4_joint (if 7-DOF)
```

**Kinematic Solver Options**:
- **KDL**: Numerical IK solver (general purpose, slower)
- **TRAC-IK**: Improved KDL with better convergence (recommended)
- **IKFast**: Analytical solver (fastest but requires separate generation step)

**Add Group**: Click "Save" to store the planning group.

**For Humanoid Robots**, you might define multiple groups:
- `left_arm`: Left shoulder-to-wrist joints
- `right_arm`: Right shoulder-to-wrist joints
- `torso`: Torso pitch/roll/yaw joints
- `upper_body`: Arms + torso (for whole-body reaching)

#### Step 5: Define Robot Poses

Click "Robot Poses" → "Add Pose".

Create named poses for common configurations:

```
Pose Name: home
Joints:
  shoulder_pan_joint: 0.0
  shoulder_lift_joint: -1.57
  elbow_joint: 1.57
  wrist_1_joint: 0.0
  wrist_2_joint: 0.0
  wrist_3_joint: 0.0
```

**Common Poses**:
- `home`: Safe starting position (arms by sides)
- `ready`: Pre-grasp position (arms extended forward)
- `tucked`: Compact pose for walking/navigating

These poses can be commanded programmatically: `move_group.set_named_target("home")`.

#### Step 6: Define End Effectors

Click "End Effectors" → "Add End Effector".

```
End Effector Name: gripper
End Effector Group: gripper (if gripper has its own planning group)
Parent Link: wrist_3_link
Parent Group: arm
```

**Note**: If your gripper doesn't have movable fingers (fixed gripper), you can skip this or create a planning group with 0 joints.

#### Step 7: Configure Virtual Joints

Click "Virtual Joints" → "Add Virtual Joint".

```
Virtual Joint Name: virtual_joint
Child Link: base_link (root of robot)
Parent Frame: world
Type: fixed
```

This creates a fixed transform from RViz's world frame to your robot's base. Required for visualizing the robot in correct position.

#### Step 8: Configure Passive Joints

Click "Passive Joints".

If your robot has passive joints (joints without motors), add them here. Most humanoid arms have no passive joints, so this is often skipped.

#### Step 9: Perception (Optional)

Click "Perception".

This configures 3D sensors (depth cameras, lidars) for obstacle detection. Skip for now if using simulation without external obstacles.

**Example Octomap Configuration**:
```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/DepthImageOctomapUpdater
    image_topic: /camera/depth/image_raw
    filtered_cloud_topic: /camera/depth/filtered_points
```

#### Step 10: Author Information

Click "Author Information" and fill in:
```
Author Name: Your Name
Email: your.email@example.com
```

#### Step 11: Generate Configuration Package

Click "Configuration Files" → Browse to output location (e.g., `~/ros2_ws/src/my_robot_moveit_config`).

Click "Generate Package".

**Generated Files**:
```
my_robot_moveit_config/
├── config/
│   ├── my_robot.srdf              # Semantic description
│   ├── joint_limits.yaml           # Velocity/acceleration limits
│   ├── kinematics.yaml             # IK solver config
│   ├── ompl_planning.yaml          # OMPL planner settings
│   └── moveit_controllers.yaml    # Controller configuration
├── launch/
│   ├── demo.launch.py              # Launch with fake controllers
│   ├── move_group.launch.py        # Launch planning server
│   └── ...
├── package.xml
└── CMakeLists.txt
```

### Building and Testing

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the MoveIt config package
colcon build --packages-select my_robot_moveit_config

# Source workspace
source install/setup.bash

# Launch demo with fake controllers (no real robot needed)
ros2 launch my_robot_moveit_config demo.launch.py
```

**Expected Behavior**:
- RViz opens with your robot model
- MotionPlanning panel on left side
- Planning tab shows "Planning Group" dropdown (select "arm")
- Drag interactive marker to set goal pose
- Click "Plan" → trajectory displays in RViz
- Click "Execute" → robot moves to goal

### Troubleshooting Common Issues

**Problem**: Robot model doesn't appear in RViz
- **Solution**: Check that URDF loads without errors: `check_urdf my_robot.urdf`
- Ensure all mesh files referenced in URDF exist

**Problem**: "No IK solver loaded" error
- **Solution**: Install kinematics plugin: `sudo apt install ros-humble-trac-ik-kinematics-plugin`
- Edit `config/kinematics.yaml` to use `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`

**Problem**: Planning fails instantly
- **Solution**: Check that start state is collision-free (robot might spawn intersecting ground plane)
- Adjust robot's virtual joint to lift base off ground

**Problem**: "No planning library loaded" error
- **Solution**: Install OMPL: `sudo apt install ros-humble-moveit-planners-ompl`

## Summary

### Key Takeaways
- **MoveIt 2** installs via `apt install ros-humble-moveit`
- **Setup Assistant** GUI generates MoveIt configuration packages from URDF
- **SRDF** adds semantic information: planning groups, end effectors, collision matrix
- **Planning groups** define kinematic chains for motion planning
- **Kinematic solvers** (KDL, TRAC-IK, IKFast) compute inverse kinematics
- **demo.launch.py** tests configuration with fake controllers before hardware integration

### What's Next
In Chapter 3.3, you'll dive deep into kinematics solvers and implement custom IK solutions.

## Exercises

See [Module 3 Exercises](./exercises.md) - Exercise 3.1 and 3.2 cover installation and URDF configuration.

## References

- MoveIt 2 Setup Assistant Tutorial. (2023). Retrieved from https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html
- Coleman, D., Sucan, I., Chitta, S., & Correll, N. (2014). Reducing the barrier to entry of complex robotic software: A MoveIt! case study. *Journal of Software Engineering for Robotics*, 5(1), 3-16.

---

**Word Count**: ~750 words
