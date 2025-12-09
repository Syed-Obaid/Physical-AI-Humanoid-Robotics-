---
id: exercises
title: "Chapter 2.4: Module 2 Exercises"
sidebar_label: "2.4 Exercises"
sidebar_position: 4
description: Hands-on exercises for digital twin simulation with Gazebo
keywords: [exercises, Gazebo, simulation, project, grading]
---

# Chapter 2.4: Module 2 Exercises

## Learning Objectives

By the end of this chapter, you will have:

1. Created a complete Gazebo simulation environment with custom worlds
2. Integrated multiple sensors (IMU, camera, lidar) on a simulated robot
3. Bridged all sensor data to ROS 2 and visualized in RViz
4. Completed a capstone project simulating a humanoid robot in a complex environment

## Prerequisites

All chapters from Module 2 (Overview, Gazebo Setup, Sensor Simulation).

## Exercise Structure

12 exercises total across 3 chapters + 1 capstone project:
- **Exercises 2.1-2.4**: Gazebo Setup (4 exercises)
- **Exercises 2.5-2.8**: Sensor Simulation (4 exercises)
- **Exercises 2.9-2.11**: Integration (3 exercises)
- **Exercise 2.12**: Capstone Project (graded, 30 points)

---

## Exercises 2.1-2.4: Gazebo World Creation

### Exercise 2.1: Basic World with Physics (5 points)

**Task**: Create an SDF world file with:
- Physics timestep: 0.001s (1ms)
- Gravity: -9.81 m/s² (Earth gravity)
- Ground plane with friction coefficient 0.8
- Directional sun light with shadow casting enabled

**Deliverable**: `basic_world.sdf` file that launches with `gz sim basic_world.sdf`

**Grading**:
- World launches without errors (2 pts)
- Physics parameters correct (2 pts)
- Lighting and shadows visible (1 pt)

---

### Exercise 2.2: Custom Environment (5 points)

**Task**: Add obstacles to the world:
- 3 box obstacles (dimensions: 0.5×0.5×1.0 m)
- 1 cylinder obstacle (radius: 0.3m, height: 1.5m)
- Place obstacles at positions that form a navigation challenge

**Deliverable**: `obstacle_world.sdf` with static obstacles

**Grading**:
- All 4 obstacles present (2 pts)
- Collision geometry matches visual (2 pts)
- Obstacles positioned strategically (1 pt)

---

### Exercise 2.3: ROS 2 Bridge Configuration (5 points)

**Task**: Create a bridge configuration YAML file for:
- `/cmd_vel` topic (Twist messages)
- `/odom` topic (Odometry messages)
- `/joint_states` topic (JointState messages)

**Deliverable**: `bridge_config.yaml` and launch file that starts bridge

**Grading**:
- All 3 topics bridged correctly (3 pts)
- Bidirectional communication verified (2 pts)

---

### Exercise 2.4: URDF to SDF Conversion (5 points)

**Task**: Convert the 3-DOF arm URDF from Module 1 to SDF format and spawn in Gazebo at position (0, 0, 0.5).

**Deliverable**: `arm.sdf` and spawn command or launch file

**Grading**:
- Conversion preserves joint/link structure (2 pts)
- Model spawns successfully (2 pts)
- Joint states published to ROS 2 (1 pt)

---

## Exercises 2.5-2.8: Sensor Integration

### Exercise 2.5: IMU Sensor Addition (5 points)

**Task**: Add IMU sensor to the 3-DOF arm with:
- Update rate: 100 Hz
- Gaussian noise: stddev 0.01 rad/s (angular velocity), 0.02 m/s² (linear accel)
- Publish to `/arm/imu` topic

**Deliverable**: Modified `arm.sdf` with IMU plugin

**Grading**:
- IMU publishes at 100 Hz (2 pts)
- Noise parameters configured (2 pts)
- Topic visible in `ros2 topic list` (1 pt)

---

### Exercise 2.6: Camera Integration (5 points)

**Task**: Add RGB camera to arm end-effector with:
- Resolution: 640×480
- FOV: 60° horizontal
- Frame rate: 30 FPS
- Publish to `/arm/camera/image_raw`

**Deliverable**: Modified `arm.sdf` with camera plugin

**Grading**:
- Camera images visible in RViz (2 pts)
- Resolution and FOV match specs (2 pts)
- CameraInfo published (1 pt)

---

### Exercise 2.7: Lidar Sensor (5 points)

**Task**: Add 2D lidar to arm base with:
- 360 horizontal samples (1° resolution)
- Range: 0.1m to 10m
- Update rate: 10 Hz
- Publish to `/arm/scan`

**Deliverable**: Modified `arm.sdf` with lidar plugin

**Grading**:
- Lidar detects obstacles (2 pts)
- 360° coverage verified (2 pts)
- Scan data in RViz (1 pt)

---

### Exercise 2.8: Multi-Sensor Visualization (5 points)

**Task**: Create RViz configuration file showing:
- Robot model (from URDF/SDF)
- IMU orientation (Axes display)
- Camera feed (Image display)
- Lidar scan (LaserScan display)

**Deliverable**: `sensors.rviz` configuration file

**Grading**:
- All 4 displays configured (3 pts)
- Appropriate color schemes (1 pt)
- Fixed frame set correctly (1 pt)

---

## Exercises 2.9-2.11: Integration

### Exercise 2.9: Launch File Integration (5 points)

**Task**: Create a single launch file that:
- Starts Gazebo with obstacle world
- Spawns robot with all sensors
- Launches ros_gz_bridge for all topics
- Opens RViz with sensor configuration

**Deliverable**: `full_simulation.launch.py`

**Grading**:
- All nodes launch successfully (3 pts)
- No manual steps required (2 pts)

---

### Exercise 2.10: Sensor Noise Comparison (5 points)

**Task**: Record rosbag data with:
1. IMU with stddev=0.001 (low noise)
2. IMU with stddev=0.05 (high noise)

Compare noise impact on orientation estimate over 30 seconds.

**Deliverable**: 2 rosbag files + analysis plot (orientation drift vs time)

**Grading**:
- Both rosbags recorded (2 pts)
- Plot shows noise comparison (2 pts)
- Written analysis (1-2 sentences) (1 pt)

---

### Exercise 2.11: Physics Parameter Tuning (5 points)

**Task**: Test ground friction coefficients (0.1, 0.5, 1.0) by:
- Applying constant torque to robot joints
- Measuring slip distance when robot contacts ground

**Deliverable**: Report with friction vs slip distance table

**Grading**:
- 3 friction values tested (2 pts)
- Slip measured quantitatively (2 pts)
- Conclusion about realistic friction value (1 pt)

---

## Exercise 2.12: Capstone Project (30 points)

### Project: Humanoid Sensor Suite in Complex Environment

**Objective**: Create a complete simulation of a simplified humanoid (torso + 2 legs, 6 DOF total) in a multi-room environment with sensor-based localization.

**Requirements**:

**Part 1: World Creation (10 pts)**
- Multi-room environment (at least 3 rooms connected by doorways)
- 10+ obstacles (boxes, cylinders, walls)
- Textured ground and walls
- Realistic lighting (at least 2 light sources)

**Part 2: Humanoid Model (10 pts)**
- 6-DOF robot: torso link + 2 legs (3 DOF each: hip, knee, ankle)
- URDF with proper mass/inertia for all links
- Convert to SDF and spawn in world
- Joint controllers for basic walking (position or velocity control)

**Part 3: Sensor Integration (10 pts)**
- IMU in torso (100 Hz)
- Front-facing camera (640×480, 30 FPS)
- 2D lidar at torso height (360°, 10 Hz)
- All sensors bridged to ROS 2 with realistic noise

**Part 4: Bonus (Extra 10 pts, max total 30)**
- Implement simple walking gait (forward motion using joint commands) (+5 pts)
- Record rosbag during 30s walk and visualize sensor data (+3 pts)
- Obstacle avoidance using lidar (stop if object < 0.5m) (+2 pts)

### Deliverables

Submit a single directory containing:
1. `humanoid.urdf` and `humanoid.sdf`
2. `multi_room_world.sdf`
3. `launch_humanoid.launch.py` (launches everything)
4. `sensors.rviz` configuration
5. `README.md` with:
   - Installation instructions
   - How to run simulation
   - Description of walking gait (if implemented)
   - Screenshots of RViz showing robot + sensors

### Grading Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| World complexity | 10 | Multi-room design, obstacles, textures |
| Robot model | 10 | 6-DOF structure, URDF validity, physics parameters |
| Sensors | 10 | IMU + camera + lidar with noise, ROS 2 bridging |
| **Bonus** | +10 | Walking gait (+5), rosbag (+3), obstacle avoidance (+2) |
| **Total** | 30 | Passing: 21/30 (70%) |

---

## Submission Guidelines

**Individual Exercises (2.1-2.11)**:
- Submit via course platform as individual files
- Each exercise graded independently (5 points each)
- Total: 55 points

**Capstone Project (2.12)**:
- Submit as ZIP archive named `module2_capstone_[lastname].zip`
- Due: End of week 6
- Total: 30 points (baseline) + 10 bonus

**Module 2 Total**: 85 points (baseline) + 10 bonus = 95 points max

**Passing Criteria**: 60/85 points (70.6%)

---

## Testing Checklist

Before submission, verify:
- [ ] Gazebo launches without errors
- [ ] All sensors publish data: `ros2 topic hz /arm/imu /arm/scan /arm/camera/image_raw`
- [ ] RViz displays all sensor streams
- [ ] Launch file starts all nodes automatically
- [ ] No hardcoded paths (use relative paths or ROS 2 package paths)

---

## Additional Resources

- Gazebo tutorials: https://gazebosim.org/docs/garden/tutorials
- SDF specification: http://sdformat.org/
- ros_gz examples: https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_sim_demos

---

**Word Count**: ~900 words
