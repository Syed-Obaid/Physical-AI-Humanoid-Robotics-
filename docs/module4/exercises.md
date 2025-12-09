---
id: exercises
title: "Module 4: Embodied AI Exercises"
sidebar_label: "Exercises"
sidebar_position: 1
description: Hands-on exercises for vision-language-action models
keywords: [VLA, vision models, LLM, policy learning, reinforcement learning]
---

# Module 4: Embodied AI Exercises

## Learning Objectives

By completing these exercises, you will:
1. Integrate computer vision models for object detection in ROS 2
2. Connect large language models for natural language command parsing
3. Train simple policies using reinforcement learning in Isaac Sim
4. Build a vision-language-action pipeline for humanoid control

## Prerequisites

- Module 1: ROS 2 Foundation
- Module 2: Digital Twin Simulation
- Module 3: Motion Planning
- Python deep learning basics (PyTorch or TensorFlow)

## Exercises

### Exercise 4.1: YOLO Object Detection (10 points)

**Task**: Integrate YOLOv8 for real-time object detection in Gazebo camera feed.

**Requirements**:
- Install Ultralytics YOLO: `pip install ultralytics`
- Subscribe to `/robot/camera/image_raw`
- Detect objects and publish bounding boxes
- Visualize detections in RViz

**Deliverable**: ROS 2 node publishing `vision_msgs/Detection2DArray`

---

### Exercise 4.2: LLM Command Parser (15 points)

**Task**: Use OpenAI API or local LLM to parse natural language commands into robot actions.

**Example Commands**:
- "Pick up the red cube" → `{action: 'pick', object: 'cube', color: 'red'}`
- "Move forward 2 meters" → `{action: 'navigate', distance: 2.0, direction: 'forward'}`

**Requirements**:
- Parse 5 different command types
- Handle ambiguity with clarification questions
- Integrate with action server

**Deliverable**: Python node with LLM integration + test cases

---

### Exercise 4.3: RL Policy Training (20 points)

**Task**: Train a reaching policy in Isaac Sim using reinforcement learning.

**Setup**:
- Task: Reach random target positions with robot arm
- Reward: Negative distance to target
- Algorithm: PPO (use Stable-Baselines3)
- Training: 1M timesteps

**Requirements**:
- Implement environment wrapper
- Log training metrics (reward, success rate)
- Test trained policy on 100 random targets

**Deliverable**: Trained policy weights + training plots

---

### Exercise 4.4: VLA Pipeline Integration (20 points)

**Task**: Combine vision, language, and action into end-to-end pipeline.

**Pipeline**:
1. User says: "Put the blue box on the table"
2. LLM parses → `{action: 'place', object: 'box', color: 'blue', location: 'table'}`
3. Vision detects blue box position
4. Motion planner generates grasp trajectory
5. Robot executes pick-and-place

**Deliverable**: Complete launch file + demonstration video (30 seconds)

---

### Exercise 4.5: Capstone - Household Task (35 points)

**Objective**: Complete realistic household task using full VLA stack.

**Task Options** (choose one):
1. **Sorting**: Sort 5 colored objects into 2 bins based on color
2. **Cleaning**: Pick up 3 scattered objects and place in trash bin
3. **Serving**: Pick up cup from counter and place on tray

**Requirements**:
- Natural language input for task specification
- Vision-based object localization
- Collision-free motion planning
- Success metric: 80%+ success rate over 10 trials

**Grading**:
- System integration (15 pts)
- Robustness to variations (10 pts)
- Success rate (10 pts)

**Deliverable**: Full codebase + video demonstration + performance report

---

## Resources

- YOLOv8: https://docs.ultralytics.com/
- OpenAI API: https://platform.openai.com/docs/
- Isaac Sim RL: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_gym_isaac_gym.html
- Stable-Baselines3: https://stable-baselines3.readthedocs.io/

## Submission

Submit via course platform with:
- Source code (organized as ROS 2 packages)
- Trained model weights
- Video demonstrations
- Performance analysis report

**Total Points**: 100 points
**Passing**: 70/100 (70%)

---

**Word Count**: ~500 words
