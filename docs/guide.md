---
id: guide
title: "Course Guide"
sidebar_position: 1
sidebar_label: "Course Guide"
description: Complete module overview and navigation for the Physical AI & Humanoid Robotics Hackathon Guide
keywords: [course guide, modules, curriculum, learning path]
---

# Course Guide

Navigate through the complete curriculum for building intelligent humanoid robotic systems. Each module builds progressively from fundamentals to advanced integration.

---

## 📚 Module Overview

### [Module 1: ROS 2 Foundation](./module1/overview)

**Duration**: Weeks 1-3 (12 hours)
**Level**: Foundational ⭐

Master the Robot Operating System 2 middleware framework that powers modern robots.

**What You'll Build**:
- 3-DOF robotic arm URDF model
- Publisher/subscriber nodes in Python
- ROS 2 launch files for multi-node systems
- RViz visualizations for robot state

**Key Topics**:
- ROS 2 Humble installation and workspace setup
- URDF robot modeling with links and joints
- Nodes, topics, services, and actions
- Joint state publishing and control
- RViz 3D visualization

**Capstone**: 5-DOF robotic arm with inverse kinematics and pick-and-place control

<div style={{textAlign: 'center', margin: '20px 0'}}>
  <a href="./module1/overview" className="button button--primary button--lg">
    Start Module 1 →
  </a>
</div>

---

### [Module 2: Digital Twin Simulation](./module2/overview)

**Duration**: Weeks 4-6 (10 hours)
**Level**: Intermediate ⭐⭐

Create realistic physics-based simulations for testing algorithms before hardware deployment.

**What You'll Build**:
- Gazebo worlds with obstacles and physics
- Simulated IMU, camera, and lidar sensors
- ROS 2-Gazebo bridge for data flow
- Multi-room navigation environments

**Key Topics**:
- Gazebo Garden installation and SDF files
- Physics engine configuration (gravity, friction, contacts)
- Sensor plugins (IMU, RGB camera, 2D/3D lidar)
- Sensor noise models for sim-to-real transfer
- ros_gz_bridge for topic mapping

**Capstone**: Humanoid sensor suite in complex environment (6-DOF robot, 3 sensor types, multi-room world)

<div style={{textAlign: 'center', margin: '20px 0'}}>
  <a href="./module2/overview" className="button button--primary button--lg">
    Start Module 2 →
  </a>
</div>

---

### [Module 3: Motion Planning](./module3/exercises)

**Duration**: Weeks 7-8 (8 hours)
**Level**: Intermediate ⭐⭐

Implement collision-free trajectories and manipulation planning for robotic arms.

**What You'll Build**:
- MoveIt 2 configuration for custom robots
- Inverse kinematics solvers (KDL, TRAC-IK)
- RRT trajectory planning with obstacles
- Pick-and-place pipeline

**Key Topics**:
- MoveIt 2 Setup Assistant workflow
- Forward and inverse kinematics
- Collision checking and scene understanding
- Motion planners (RRT, RRT*, OMPL)
- Grasp planning and execution

**Capstone**: Complete pick-and-place system (object detection → grasp planning → collision-free motion → place)

<div style={{textAlign: 'center', margin: '20px 0'}}>
  <a href="./module3/exercises" className="button button--primary button--lg">
    Start Module 3 →
  </a>
</div>

---

### [Module 4: Embodied AI](./module4/exercises)

**Duration**: Weeks 9-10 (10 hours)
**Level**: Advanced ⭐⭐⭐

Integrate vision, language, and action models for intelligent robot control.

**What You'll Build**:
- YOLO object detection in ROS 2
- LLM-based command parser (OpenAI API)
- Reinforcement learning policy (PPO in Isaac Sim)
- Vision-Language-Action (VLA) pipeline

**Key Topics**:
- YOLOv8 for real-time object detection
- Large language model integration
- Policy learning with Stable-Baselines3
- Isaac Sim for GPU-accelerated training
- End-to-end VLA system design

**Capstone**: Household task automation (3 options: sorting objects, cleaning, or serving)

<div style={{textAlign: 'center', margin: '20px 0'}}>
  <a href="./module4/exercises" className="button button--primary button--lg">
    Start Module 4 →
  </a>
</div>

---

### [Module 5: Integration & Capstone](./module5/exercises)

**Duration**: Weeks 11-12 (12 hours)
**Level**: Advanced ⭐⭐⭐

Deploy complete systems and complete an open-ended hackathon challenge.

**What You'll Build**:
- Unified launch file for entire robotics stack
- Automated testing framework (CI/CD)
- Jetson edge deployment with TensorRT
- Final hackathon project

**Key Topics**:
- System integration patterns
- ROS 2 lifecycle nodes and health monitoring
- Docker containerization for deployment
- Performance profiling and optimization
- Edge AI with NVIDIA Jetson

**Capstone Options**:
- **Autonomous Kitchen Assistant**: Navigate, identify ingredients, follow recipe via LLM, manipulate objects
- **Search and Rescue**: Navigate cluttered space, classify objects, report via natural language
- **Warehouse Automation**: Process natural language orders, locate/manipulate 5+ objects

<div style={{textAlign: 'center', margin: '20px 0'}}>
  <a href="./module5/exercises" className="button button--primary button--lg">
    Start Module 5 →
  </a>
</div>

---

## 🎯 Learning Pathways

### Sequential Path (Recommended)
Follow modules 1→2→3→4→5 for comprehensive understanding. Each module builds on previous concepts.

### Fast Track (Experienced Developers)
If you have ROS experience:
- **Skip Module 1** if proficient with ROS 2 Humble
- **Start Module 2** to focus on simulation and AI integration
- **Complete Modules 3-5** for motion planning and embodied AI

### Hackathon Sprint (Weekend Projects)
For rapid prototyping:
- **Module 1 Basics** (4 hours): ROS 2 setup + URDF
- **Module 2 Simulation** (3 hours): Gazebo world + sensors
- **Module 4 AI** (5 hours): YOLO + LLM integration
- **Module 5 Integration** (8 hours): Full system + capstone

---

## 📊 Assessment & Grading

Each module includes graded exercises:

| Module | Exercises | Points | Capstone | Total | Pass (70%) |
|--------|-----------|--------|----------|-------|------------|
| Module 1 | 15 | 135 | 45 | 180 | 126 |
| Module 2 | 11 | 55 | 30 | 85 | 60 |
| Module 3 | 4 | 30 | 25 | 70 | 49 |
| Module 4 | 4 | 65 | 35 | 100 | 70 |
| Module 5 | 3 | 50 | 50 | 100 | 70 |
| **Total** | **37** | **335** | **185** | **535** | **375** |

**Grading Scale**:
- **90-100%**: Excellent (A) - All features implemented with optimization
- **70-89%**: Good (B) - Core functionality working, minor issues
- **50-69%**: Adequate (C) - Basic requirements met, significant gaps
- **Below 50%**: Incomplete - Major functionality missing

---

## 🛠️ Setup Checklist

Before starting Module 1, ensure you have:

### Software Installation
- [ ] Ubuntu 22.04 LTS (native or WSL2)
- [ ] Python 3.10 or higher
- [ ] Git version control
- [ ] VS Code or preferred IDE
- [ ] ROS 2 Humble (installed in Module 1.2)

### Hardware Check
- [ ] 8GB+ RAM available
- [ ] 50GB+ free disk space
- [ ] GPU (optional, for Module 4 RL training)

### Accounts (Optional)
- [ ] GitHub account (for code repositories)
- [ ] OpenAI API key (for Module 4 LLM)
- [ ] NVIDIA Developer account (for Isaac Sim)

---

## 📖 Additional Resources

### Reference Pages
- **[Glossary](./glossary)** - Technical terminology and definitions (27 terms)
- **[Notation Guide](./notation)** - Mathematical symbols and conventions
- **[References](./references)** - Academic papers and documentation (20+ sources, APA format)

### Code Examples
All working code examples available in `/examples` directory:
- Module 1: `examples/module1/ros2-basics/` (7 files)
- Module 2: `examples/module2/gazebo-basics/` (3 files)

### Community Support
- **GitHub Issues**: Report bugs or request clarifications
- **Office Hours**: Check syllabus for schedule
- **Discussion Forum**: Connect with other learners

---

## 🚀 Ready to Begin?

<div style={{textAlign: 'center', margin: '40px 0'}}>
  <a href="./module1/overview" className="button button--success button--lg" style={{marginRight: '10px'}}>
    📚 Start Module 1: ROS 2 Foundation
  </a>
  <a href="./" className="button button--secondary button--lg">
    ← Back to Introduction
  </a>
</div>

---

**Course Version**: 1.0.0
**Last Updated**: December 2025
**Estimated Completion Time**: 52 hours
