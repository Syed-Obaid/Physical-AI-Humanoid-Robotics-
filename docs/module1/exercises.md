---
id: exercises
title: "Chapter 1.6: Module 1 Exercises"
sidebar_label: "1.6 Exercises"
sidebar_position: 6
description: Comprehensive exercises and capstone project for Module 1
keywords: [exercises, capstone project, grading, ROS 2 practice]
---

# Chapter 1.6: Module 1 Exercises

## Learning Objectives

By the end of this chapter, you will be able to:

1. Apply all Module 1 concepts (URDF, nodes, topics, services, actions) to solve practical problems
2. Integrate multiple ROS 2 components into a cohesive robotic system
3. Debug complex issues using ROS 2 CLI tools and visualization
4. Demonstrate mastery through a graded capstone mini-project

## Prerequisites

### Required Knowledge
- Completion of Chapters 1.1-1.5
- Proficiency in Python programming and XML editing
- Understanding of ROS 2 workspace build process

### Previous Chapters
- All previous Module 1 chapters

## Content

### Overview of Exercise Structure

Module 1 includes **18 total exercises** distributed across difficulty levels:

**Difficulty Levels**:
- **⭐ Easy**: Basic comprehension and tool usage (5 points each)
- **⭐⭐ Medium**: Applied understanding and implementation (10 points each)
- **⭐⭐⭐ Hard**: Synthesis, debugging, and technical analysis (15 points each)

**Total Points**: 180
- Easy exercises (6 total): 30 points
- Medium exercises (9 total): 90 points
- Hard exercises (3 total): 45 points

**Passing Score**: 140/180 (78%)

**Submission Requirements**:
- GitHub repository with organized code (separate folders per exercise)
- Screenshots showing successful execution
- Technical reports (1 page PDF per hard exercise) covering methodology, challenges, and results

### Installation & Workspace Exercises

**Exercise 1.1** (⭐ Easy - 5 points)
- **Task**: Verify ROS 2 installation by running `ros2 topic list` in a new terminal
- **Requirement**: Identify at least 2 active topics from the system
- **Deliverable**: Screenshot showing topic list output

**Exercise 1.2** (⭐⭐ Medium - 10 points)
- **Task**: Create a workspace named `humanoid_ws` in your home directory, build it, and verify activation
- **Requirements**:
  - Workspace builds successfully with `colcon build`
  - `echo $AMENT_PREFIX_PATH` includes workspace path
  - Add automatic sourcing to `~/.bashrc`
- **Deliverable**: Screenshot showing build output and environment variable

**Exercise 1.3** (⭐⭐ Medium - 10 points)
- **Task**: Intentionally break workspace by deleting `install/setup.bash`, then troubleshoot and fix
- **Requirements**:
  - Document error messages encountered
  - Explain root cause in 2-3 sentences
  - Successfully rebuild workspace
- **Deliverable**: Short write-up (250 words) describing debugging process

### URDF Modeling Exercises

**Exercise 1.4** (⭐ Easy - 5 points)
- **Task**: Modify `simple_arm.urdf` to change upper arm length from 30cm to 40cm
- **Requirement**: Verify change in RViz by visual inspection
- **Deliverable**: Modified URDF file + RViz screenshot

**Exercise 1.5** (⭐⭐ Medium - 10 points)
- **Task**: Add a 4th joint (`gripper_joint`, prismatic, 0-0.05m range) with simple gripper geometry
- **Requirements**:
  - Gripper consists of two small boxes (2cm x 1cm x 1cm) that move apart when joint extends
  - Joint slider in RViz controls gripper opening
- **Deliverable**: Updated URDF + RViz screenshot showing gripper at 0m and 0.05m positions

**Exercise 1.6** (⭐⭐⭐ Hard - 15 points)
- **Task**: Create URDF for 6-DOF humanoid leg with anatomically accurate joint limits
- **Requirements**:
  - **Hip joint (3-DOF)**: Flexion/extension ±120°, abduction/adduction ±45°, internal/external rotation ±40°
  - **Knee joint (1-DOF)**: 0° to 150° (no hyperextension)
  - **Ankle joint (2-DOF)**: Dorsiflexion/plantarflexion ±30°, inversion/eversion ±20°
  - URDF passes `check_urdf` validation
  - No self-collision at mid-range joint positions
- **Grading Breakdown**:
  - URDF syntax correctness (3 pts)
  - Joint limits match specifications within ±5° (5 pts)
  - RViz visualization without errors (4 pts)
  - Technical report explaining link dimensions and inertia calculations (3 pts)
- **Deliverable**: URDF file, RViz screenshots, 1-page technical report

### Nodes and Topics Exercises

**Exercise 1.7** (⭐ Easy - 5 points)
- **Task**: Inspect `/joint_states` topic using CLI tools
- **Requirements**:
  - Run `ros2 topic list` to confirm topic exists
  - Use `ros2 topic echo /joint_states` to view messages
  - Measure publishing rate with `ros2 topic hz /joint_states`
- **Deliverable**: Terminal screenshots showing all three commands

**Exercise 1.8** (⭐⭐ Medium - 10 points)
- **Task**: Modify joint state publisher to operate at 100 Hz instead of 50 Hz
- **Requirements**:
  - Update timer period in code
  - Verify rate with `ros2 topic hz`
  - Calculate latency: difference between `msg.header.stamp` and `self.get_clock().now()`
- **Deliverable**: Modified Python file + screenshot showing measured 100 Hz rate + latency measurement

**Exercise 1.9** (⭐⭐⭐ Hard - 15 points)
- **Task**: Implement low-pass filter node
- **Requirements**:
  - Subscribes to `/joint_states`, publishes to `/joint_states_filtered`
  - Moving average filter over 10 samples using `collections.deque`
  - Maintains 50 Hz output rate (no buffer overflow)
  - Plot original vs filtered signal for shoulder joint over 10 seconds
- **Grading Breakdown**:
  - Correct filter implementation (5 pts)
  - No rate degradation (5 pts)
  - Technical report with signal plot and analysis (5 pts)
- **Deliverable**: Python node, signal plot, 1-page technical report

### Services and Actions Exercises

**Exercise 1.10** (⭐ Easy - 5 points)
- **Task**: Call `add_two_ints` service from command line
- **Command**: `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"`
- **Requirement**: Verify service returns sum of 8
- **Deliverable**: Screenshot of successful service call

**Exercise 1.11** (⭐⭐ Medium - 10 points)
- **Task**: Create `/set_joint_position` service
- **Requirements**:
  - Service takes joint name (string) and target angle (float64)
  - Returns success flag (bool) and confirmation message (string)
  - Implement server node and client node
  - Validate angle is within ±3.14 radians
- **Deliverable**: Service definition file (.srv), server.py, client.py, test screenshot

**Exercise 1.12** (⭐⭐⭐ Hard - 15 points)
- **Task**: Implement `/move_arm` action server with trajectory interpolation
- **Requirements**:
  - Action accepts 3 target joint angles (goal)
  - Uses cubic polynomial for smooth trajectory over 5 seconds
  - Publishes feedback (progress percentage) every 0.5s
  - Returns final position error (result)
- **Grading Breakdown**:
  - Action accepts/rejects goals based on joint limits (5 pts)
  - Feedback published at 2 Hz (5 pts)
  - Technical report with position vs time plot showing cubic trajectory (5 pts)
- **Deliverable**: Action definition, action server node, motion profile plot, technical report

### Integration Exercises

**Exercise 1.13** (⭐ Easy - 5 points)
- **Task**: Launch RViz with custom URDF using provided launch file
- **Requirement**: Use `ros2 launch` with `urdf_visualizer.launch.py`, verify joint sliders control arm
- **Deliverable**: Screenshot of RViz showing 3-DOF arm with sliders

**Exercise 1.14** (⭐⭐ Medium - 10 points)
- **Task**: Create keyboard teleoperation node for joint control
- **Requirements**:
  - Use `pynput` library to capture arrow key input
  - Publish velocity commands on `/joint_velocity_commands` (Float64MultiArray)
  - Emergency stop on Space key (publishes zeros)
  - Smooth control (no jerky motion)
- **Grading Breakdown**:
  - Correct keyboard handling (5 pts)
  - Emergency stop functionality (3 pts)
  - Code comments explaining input processing (2 pts)
- **Deliverable**: Python node, demo video showing smooth teleoperation

**Exercise 1.15** (⭐⭐ Medium - 10 points)
- **Task**: Multi-robot coordination with synchronized movements
- **Requirements**:
  - Launch 2 simulated robots using different URDF namespaces (`/robot_a/`, `/robot_b/`)
  - Robot B mirrors Robot A's joint movements with 1-second delay
  - Use ROS 2 parameters to configure delay dynamically
  - No topic name collisions between robots
- **Grading Breakdown**:
  - Correct namespace usage (5 pts)
  - Delay implementation using timers (3 pts)
  - Technical discussion of synchronization challenges (2 pts)
- **Deliverable**: Launch file, coordination node, short write-up on sync challenges

### Graded Capstone Mini-Project

The capstone integrates all Module 1 skills into a comprehensive robotics project.

**Project Prompt**: Design and implement a 5-DOF robotic arm capable of pick-and-place operations in RViz simulation.

---

**Exercise 1.16** (⭐⭐⭐ Hard - 15 points): **URDF Model Design**

**Requirements**:
1. 5-DOF arm structure:
   - 3-DOF shoulder/elbow (revolute joints)
   - 2-DOF wrist (revolute joints)
   - End-effector gripper (prismatic joint, 0-0.1m range)
2. Visual geometry using primitive shapes (cylinders, boxes, spheres)
3. Realistic joint limits based on industrial robot arms (e.g., UR5 specifications)
4. Accurate inertial properties (mass, inertia tensor for each link)

**Grading Criteria** (15 points total):
- **Syntax correctness** (5 pts): URDF passes `check_urdf` validation with no errors
- **Joint limits** (5 pts): All 6 joints have physically realistic limits and dynamics parameters
- **Visualization quality** (5 pts): RViz displays arm without visual artifacts, collision geometry matches visual geometry

**Deliverable**: `capstone_arm.urdf` file, RViz screenshots showing arm at 3 different configurations

---

**Exercise 1.17** (⭐⭐⭐ Hard - 15 points): **Inverse Kinematics Service**

**Requirements**:
1. Service `/compute_ik` that accepts target end-effector pose: (x, y, z, roll, pitch, yaw)
2. Returns joint angles (array of 5 floats) to reach target pose
3. Uses analytical IK or PyKDL library for solution
4. Handles unreachable targets gracefully (returns success=false with error message)

**Grading Criteria** (15 points total):
- **Accuracy** (10 pts): IK solution places end-effector within 1cm of target position
- **Error handling** (5 pts): Service rejects unreachable targets and provides informative messages

**Deliverable**: `ik_service.py` node, service definition file, test results showing 5 successful IK solutions

---

**Exercise 1.18** (⭐⭐⭐ Hard - 15 points): **Pick-and-Place Action Server**

**Requirements**:
1. Action `/pick_and_place` with goal: (object_pose, place_pose)
2. Executes motion in 4 phases:
   - **Approach**: Move above object
   - **Grasp**: Lower gripper, close gripper
   - **Lift**: Raise object 10cm
   - **Place**: Move to target, open gripper
3. Publishes feedback with current phase and progress percentage
4. Uses IK service from Exercise 1.17 for trajectory waypoints

**Grading Criteria** (15 points total):
- **Smooth trajectories** (5 pts): Cubic interpolation between waypoints, no sudden velocity changes
- **Accurate feedback** (5 pts): Progress updates at 2 Hz, phase transitions reported correctly
- **Error handling** (5 pts): Gracefully handles IK failures, collision detection (simulated)

**Deliverable**: `pick_place_action.py` node, action definition, 5-minute demo video showing complete pick-and-place cycle

---

### Capstone Submission Package

**Required Files**:
- GitHub repository with organized structure:
  ```
  capstone_project/
  ├── urdf/
  │   └── capstone_arm.urdf
  ├── src/
  │   ├── ik_service.py
  │   └── pick_place_action.py
  ├── srv/
  │   └── ComputeIK.srv
  ├── action/
  │   └── PickAndPlace.action
  ├── launch/
  │   └── capstone.launch.py
  ├── README.md (setup instructions)
  └── docs/
      └── technical_report.pdf (3 pages)
  ```

**Technical Report Contents** (3 pages):
1. **Design Decisions** (1 page): Explain link dimensions, joint limit choices, gripper design
2. **IK Algorithm** (1 page): Describe analytical or numerical approach, convergence criteria, singularity handling
3. **Challenges & Solutions** (1 page): Document issues encountered (e.g., RViz configuration, trajectory smoothness), how resolved, lessons learned

**Demo Video Requirements**:
- 5 minutes maximum length
- Show RViz visualization throughout
- Demonstrate successful pick-and-place from 3 different object locations
- Narrate key phases and explain robot behavior

**Grading Summary**:
- Exercise 1.16 (URDF): 15 points
- Exercise 1.17 (IK Service): 15 points
- Exercise 1.18 (Action Server): 15 points
- **Total Capstone**: 45 points

### Submission Guidelines

**Due Date**: End of Week 3 (from module start)

**Format**:
- All code pushed to GitHub repository (public or provide read access)
- README.md with setup instructions (must run on Ubuntu 22.04 + ROS 2 Humble)
- Screenshots in `docs/screenshots/` folder
- Technical reports as PDFs in `docs/reports/`

**Late Policy**: 10% deduction per day (max 3 days late, then 0 points)

**Collaboration Policy**: Individual work only. You may discuss concepts with peers but must write all code independently. Code reviews with instructors are encouraged.

**Resources Available**:
- Office hours (schedule via course portal)
- ROS 2 documentation (https://docs.ros.org)
- Forum discussions (post questions, help others)
- Example code from chapters (but cite if directly adapted)

## Summary

### Key Takeaways
- **18 exercises cover all Module 1 topics**: From basic installation to complex integration
- **Grading emphasizes practical skills**: Working code + technical understanding
- **Capstone demonstrates mastery**: 5-DOF arm with IK and pick-and-place capabilities
- **Incremental difficulty**: Easy exercises build confidence, hard exercises challenge understanding
- **Real-world relevance**: Skills transfer directly to industrial and research robotics projects

### What's Next
After completing Module 1, you're ready for Module 2: Digital Twin Simulation. You'll spawn your URDF robots in Gazebo, simulate physics (gravity, collisions), and integrate realistic sensors (IMU, depth cameras).

## References

- Robot Operating System 2. (2023). *ROS 2 Humble tutorials*. Retrieved December 7, 2025, from https://docs.ros.org/en/humble/Tutorials.html
- Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer Handbook of Robotics* (2nd ed.). Springer.
- Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.
- Universal Robots. (2023). *UR5 technical specifications*. Retrieved December 7, 2025, from https://www.universal-robots.com/products/ur5-robot/

---

**Word Count**: ~1150 words
