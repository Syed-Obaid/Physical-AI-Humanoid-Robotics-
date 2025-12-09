---
id: glossary
title: Glossary
sidebar_label: Glossary
description: Terminology and definitions for Physical AI and Humanoid Robotics
keywords: [glossary, robotics, ROS 2, AI, terminology]
---

# Glossary

Alphabetically sorted terms and definitions used throughout the Physical AI & Humanoid Robotics Hackathon Guide.

---

## A

### Action

Long-running goal-oriented ROS 2 communication pattern that supports feedback during execution and cancellation. Consists of three parts: goal (task parameters), feedback (progress updates), and result (final outcome). Used for tasks like navigation or object manipulation.

**Related:** Service, Topic, ROS 2

---

## C

### colcon

Build system for ROS 2 workspaces that compiles packages in dependency order and generates setup scripts for environment configuration. Supports parallel builds and symlink installation for faster iteration during development.

**Related:** ROS 2, Workspace, Package

---

## D

### Domain Randomization

Sim-to-real transfer technique that randomizes physics parameters (mass, friction, sensor noise) during training to make policies robust to model uncertainties. Reduces performance gap when deploying from simulation to real hardware.

**Related:** Sim-to-Real, Reinforcement Learning, Transfer Learning

---

## G

### Gazebo

Open-source 3D robotics simulator that provides realistic physics simulation (gravity, friction, collisions) for testing robot behaviors in virtual environments before hardware deployment. Gazebo Garden is the LTS version used in this guide.

**Related:** Digital Twin, Simulation, Physics Engine

---

## I

### IMU (Inertial Measurement Unit)

Sensor that measures acceleration and angular velocity using accelerometers and gyroscopes. Common in robotics for estimating robot orientation and detecting motion. Subject to drift over time requiring calibration or sensor fusion.

**Related:** Sensor Simulation, Sim-to-Real Gap

### Inverse Kinematics (IK)

Computational problem of determining joint angles required to position a robot's end-effector at a desired pose (position + orientation). Solvers include analytical methods for simple kinematic chains and numerical methods (Jacobian-based, TRAC-IK) for complex manipulators.

**Related:** Forward Kinematics, MoveIt, Jacobian, End-Effector

### Isaac Sim

NVIDIA's robotics simulation platform built on Omniverse, providing GPU-accelerated physics, photorealistic rendering, and AI training capabilities via Isaac Gym for reinforcement learning.

**Related:** Reinforcement Learning, Digital Twin, Simulation

---

## J

### Joint

Mechanical connection between two links in a robot that defines their relative motion. ROS 2 URDF supports six joint types: revolute (1-DOF rotation), prismatic (1-DOF translation), continuous (unlimited rotation), fixed (no motion), floating (6-DOF), and planar (2-DOF in plane). Each joint has limits, dynamics, and axis specifications.

**Related:** Link, URDF, Kinematics

### Jetson

NVIDIA's embedded AI computing platform for edge deployment. Jetson Orin Nano (8GB) and Orin NX (16GB) are used in this guide for deploying trained AI models to physical robots with real-time inference constraints.

**Related:** Edge AI, Deployment, TensorRT

---

## L

### Launch File

Python or XML file that orchestrates the startup of multiple ROS 2 nodes with specified parameters, remappings, and configurations. Python launch files (`.launch.py`) use the `launch` API for programmatic node composition, conditional execution, and dynamic parameter loading. Replaces roslaunch from ROS 1.

**Related:** Node, ROS 2, Package

### Link

Rigid body component in a URDF robot model representing a physical part (e.g., arm segment, wheel, chassis). Each link defines visual geometry (for rendering), collision geometry (for physics), and inertial properties (mass, center of mass, inertia tensor). Connected to other links via joints.

**Related:** Joint, URDF, RViz

---

## M

### MoveIt

Motion planning framework for ROS 2 that provides inverse kinematics, collision checking, and trajectory optimization for robotic manipulators. Includes integration with Gazebo and RViz for simulation and visualization.

**Related:** Motion Planning, Inverse Kinematics, ROS 2

### mAP (Mean Average Precision)

Performance metric for object detection models, measuring the average precision across all object classes and IoU (Intersection over Union) thresholds. Higher mAP indicates better detection accuracy.

**Related:** Object Detection, Perception, YOLO

---

## N

### Node

Fundamental execution unit in ROS 2 representing a single-purpose process (e.g., sensor driver, controller, planner). Nodes communicate via topics (pub-sub), services (request-reply), and actions (goal-oriented). Created using rclpy (Python) or rclcpp (C++). Multiple nodes form a distributed robotic system.

**Related:** Topic, Service, Action, ROS 2

---

## P

### package.xml

Manifest file defining a ROS 2 package's metadata, dependencies, and build configuration. Specifies package name, version, maintainer, license, and dependencies (build, exec, test). Required for colcon build system to resolve dependency order. Follows REP-140 format specification.

**Related:** colcon, ROS 2, Workspace

### Publisher

ROS 2 node component that sends messages to a topic. Created with `create_publisher(msg_type, topic_name, qos_profile)`. Publishers use Quality of Service (QoS) policies to configure reliability, durability, and history depth. Multiple publishers can send to the same topic.

**Related:** Subscriber, Topic, Node, rclpy

---

## R

### ROS 2 (Robot Operating System 2)

Open-source middleware framework for robot software development, providing inter-process communication (topics, services, actions), hardware abstraction, and package management. ROS 2 Humble is the LTS version used in this guide.

**Related:** URDF, Node, Topic, Publisher, Subscriber

### rclpy

ROS 2 Python client library providing Pythonic APIs for creating nodes, publishers, subscribers, services, actions, and parameters. Part of the core ROS 2 distribution. Enables rapid prototyping and development of robot behaviors without C++ compilation overhead. Version 3.3.0+ required for ROS 2 Humble.

**Related:** Node, Publisher, Subscriber, ROS 2

### RViz

3D visualization tool for ROS 2 that displays robot models, sensor data (point clouds, camera images), transforms (TF), and planning trajectories in real-time. Essential for debugging robot state and verifying URDF models.

**Related:** URDF, ROS 2, Visualization

---

## S

### SDF (Simulation Description Format)

XML-based format used by Gazebo to define complete simulation environments including worlds, models, physics properties, and sensors. More comprehensive than URDF which focuses only on robot structure.

**Related:** Gazebo, URDF, Simulation

### Service

Synchronous request-reply ROS 2 communication pattern for infrequent operations requiring acknowledgment (e.g., "set parameter", "compute IK"). Client sends request and blocks until server responds. Service definitions (`.srv` files) specify request and response message types. Typically ~10ms latency.

**Related:** Action, Topic, Node, ROS 2

### Subscriber

ROS 2 node component that receives messages from a topic via callback function. Created with `create_subscription(msg_type, topic_name, callback, qos_profile)`. Callbacks execute in executor threads. QoS policies must be compatible with publisher for message delivery.

**Related:** Publisher, Topic, Node, rclpy

### Sim-to-Real (Gap)

The performance degradation observed when transferring AI policies trained in simulation to physical robots. Typical degradation is 10-20% due to unmodeled dynamics, sensor noise, and physics inaccuracies. Mitigation strategies include domain randomization and system identification.

**Related:** Reinforcement Learning, Isaac Sim, Gazebo, Domain Randomization

---

## T

### TF (Transform Tree)

ROS 2 system for tracking coordinate frame transformations over time. Maintains a tree-structured graph of 3D transforms (position + orientation) between robot links, sensors, and world frames. Published via `/tf` topic using `geometry_msgs/TransformStamped`. Essential for sensor fusion and motion planning.

**Related:** URDF, RViz, Link, Joint

### Topic

Named communication channel for asynchronous message streaming between ROS 2 nodes. Publishers send messages; subscribers receive them. Supports many-to-many communication with configurable Quality of Service (QoS) policies. Topic names follow hierarchical namespaces (e.g., `/robot/camera/image`). Core primitive for sensor data and state updates.

**Related:** Publisher, Subscriber, Node, ROS 2

---

## U

### URDF (Unified Robot Description Format)

XML-based format for defining robot kinematics, geometry, and visual appearance. Specifies joints (revolute, prismatic, fixed), links (visual, collision, inertial properties), and transforms between coordinate frames.

**Related:** ROS 2, Robot Model, Kinematics

---

## V

### VLA (Vision-Language-Action)

Multi-modal AI pipeline that processes natural language commands and visual input to generate robot action sequences. Combines speech recognition (ASR), large language models (LLMs) for intent parsing, and action planning for embodied AI control.

**Related:** LLM, Embodied AI, Natural Language Processing, Action Planning

---

## Y

### YOLO (You Only Look Once)

Real-time object detection deep learning model that processes images in a single forward pass. YOLOv8 is commonly used in robotics for detecting objects, people, and obstacles at 30+ FPS for vision-based manipulation and navigation.

**Related:** Computer Vision, Object Detection, Perception

---

## Notes

- **First Use**: Terms are linked on first use in each chapter to this glossary
- **APA Citations**: Technical definitions reference peer-reviewed sources in [References](./references.md)
- **Updates**: New terms added as modules are developed
