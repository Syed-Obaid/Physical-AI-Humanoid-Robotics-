---
id: exercises
title: "Module 3: Motion Planning Exercises"
sidebar_label: "Exercises"
sidebar_position: 1
description: Hands-on exercises for motion planning with MoveIt 2
keywords: [MoveIt, inverse kinematics, trajectory planning, collision avoidance]
---

# Module 3: Motion Planning Exercises

## Learning Objectives

By completing these exercises, you will:
1. Install and configure MoveIt 2 for humanoid arm control
2. Implement inverse kinematics solvers for end-effector positioning
3. Plan collision-free trajectories using RRT and other motion planners
4. Integrate motion planning with Gazebo simulation

## Prerequisites

- Module 1: ROS 2 Foundation (URDF modeling, nodes/topics)
- Module 2: Digital Twin (Gazebo simulation, sensor integration)
- Understanding of forward/inverse kinematics concepts

## Exercises

### Exercise 3.1: MoveIt 2 Installation (5 points)

**Task**: Install MoveIt 2 for ROS 2 Humble and verify with demo robot.

**Commands**:
```bash
sudo apt install ros-humble-moveit -y
ros2 launch moveit2_tutorials demo.launch.py
```

**Deliverable**: Screenshot of MoveIt RViz interface with Panda robot

---

### Exercise 3.2: URDF Configuration (10 points)

**Task**: Create MoveIt configuration for 3-DOF arm from Module 1.

**Steps**:
1. Use MoveIt Setup Assistant: `ros2 launch moveit_setup_assistant setup_assistant.launch.py`
2. Load arm URDF, define planning group "arm" (shoulder, elbow, wrist joints)
3. Configure collision checking
4. Generate config package

**Deliverable**: MoveIt config package with SRDF file

---

### Exercise 3.3: Inverse Kinematics (15 points)

**Task**: Implement IK solver for 5-DOF humanoid arm to reach target positions.

**Requirements**:
- Use KDL or TRAC-IK solver
- Test with 5 target positions in workspace
- Handle singularities gracefully

**Deliverable**: Python node that publishes joint states for target end-effector poses

---

### Exercise 3.4: Trajectory Planning (15 points)

**Task**: Plan collision-free trajectory around obstacles using RRT planner.

**Setup**:
- Place 3 box obstacles in Gazebo world
- Plan from start pose to goal pose avoiding obstacles

**Deliverable**: RViz video showing planned trajectory execution (10 seconds)

---

### Exercise 3.5: Capstone - Pick and Place (25 points)

**Objective**: Implement complete pick-and-place pipeline.

**Requirements**:
1. Detect object position using camera (simple red cube detection)
2. Plan grasp approach trajectory
3. Close gripper (simulate with joint position)
4. Plan trajectory to drop location
5. Open gripper

**Grading**:
- Object detection (5 pts)
- Grasp planning (8 pts)
- Collision-free motion (7 pts)
- Successful place (5 pts)

**Deliverable**: Launch file + video demonstration

---

## Resources

- MoveIt 2 Tutorials: https://moveit.picknik.ai/humble/
- KDL Kinematics: http://wiki.ros.org/kdl
- RRT Algorithm: https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree

## Submission

Submit via course platform with:
- Source code (Python/C++ nodes)
- Launch files
- README with setup instructions
- Video demonstrations for Exercises 3.4-3.5

**Total Points**: 70 points
**Passing**: 49/70 (70%)

---

**Word Count**: ~500 words
