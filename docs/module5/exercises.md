---
id: exercises
title: "Module 5: Integration & Capstone"
sidebar_label: "Exercises"
sidebar_position: 1
description: Final integration exercises and hackathon capstone project
keywords: [integration, deployment, capstone, hackathon, full system]
---

# Module 5: Integration & Capstone

## Learning Objectives

By completing this module, you will:
1. Integrate all subsystems (ROS 2, Gazebo, MoveIt, VLA pipeline)
2. Deploy to Jetson embedded platform for real-time inference
3. Complete a hackathon-style capstone project demonstrating full stack
4. Present technical demo and document design decisions

## Prerequisites

All modules (1-4) completed with passing grades.

## Exercises

### Exercise 5.1: Full System Integration (20 points)

**Task**: Create unified launch file that starts entire humanoid control stack.

**Components to Launch**:
- Gazebo simulation with humanoid model
- MoveIt motion planning
- Vision perception (YOLO detection)
- VLA command parser
- RViz visualization

**Requirements**:
- Single `launch_all.py` file
- Automatic topic remapping
- Health monitoring (restart crashed nodes)

**Deliverable**: Launch file + system architecture diagram

---

### Exercise 5.2: Testing Framework (15 points)

**Task**: Implement automated testing for robotic system.

**Test Types**:
1. Unit tests: Individual node functionality
2. Integration tests: Multi-node communication
3. System tests: End-to-end task completion

**Coverage Requirements**:
- 80%+ code coverage
- CI/CD pipeline (GitHub Actions)
- Automated build + test on PR

**Deliverable**: Test suite + CI/CD configuration

---

### Exercise 5.3: Jetson Deployment (15 points)

**Task**: Deploy vision + control pipeline to NVIDIA Jetson Orin Nano.

**Optimization Steps**:
- Convert PyTorch model to TensorRT
- Profile inference latency (target: &lt;50ms)
- Benchmark against desktop GPU

**Requirements**:
- Docker container for deployment
- Real-time performance metrics
- Power consumption analysis

**Deliverable**: Deployment guide + benchmark report

---

### Exercise 5.4: Hackathon Capstone Project (50 points)

**Objective**: Complete open-ended humanoid robotics challenge demonstrating all learned skills.

**Challenge Options** (choose one):

#### Option A: Autonomous Kitchen Assistant
- Navigate to kitchen counter
- Identify ingredients using vision
- Follow recipe instructions via LLM
- Manipulate objects (pick/place)
- Deliver prepared item to user

#### Option B: Search and Rescue Scenario
- Navigate cluttered environment
- Detect and classify objects (survivors vs obstacles)
- Plan collision-free path
- Report findings via natural language
- Demonstrate in Gazebo simulation

#### Option C: Warehouse Automation
- Receive natural language order ("Get 3 blue boxes")
- Locate objects using vision + navigation
- Pick and place in designated zones
- Handle 5+ objects with 90%+ success rate

**Capstone Requirements**:
1. **Technical Implementation** (25 pts)
   - Full ROS 2 pipeline
   - Robust error handling
   - Real-time performance

2. **Innovation** (10 pts)
   - Novel approach or optimization
   - Goes beyond basic requirements

3. **Documentation** (10 pts)
   - Architecture diagram
   - Design decisions explained
   - User manual

4. **Demonstration** (5 pts)
   - 5-minute video demo
   - Narrated technical explanation
   - Shows success cases + failure handling

**Deliverable**:
- Complete codebase (GitHub repo)
- Technical report (5-10 pages)
- Video demonstration
- Presentation slides (10 slides)

---

## Capstone Grading Rubric

| Category | Excellent (90-100%) | Good (70-89%) | Adequate (50-69%) | Poor (&lt;50%) |
|----------|---------------------|---------------|-------------------|-------------|
| **Technical** | All subsystems integrated, real-time, robust | Most features work, minor bugs | Basic functionality, significant issues | Incomplete, major failures |
| **Innovation** | Novel contributions, optimization | Some creative elements | Standard implementation | No innovation |
| **Documentation** | Comprehensive, clear | Good coverage, minor gaps | Basic docs, missing details | Minimal or unclear |
| **Demo** | Polished, professional | Clear demonstration | Functional but rough | Incomplete demo |

**Total Module 5 Points**: 100 points
**Passing**: 70/100 (70%)

---

## Hackathon Logistics

**Timeline**:
- Week 10-11: Development
- Week 12: Testing and refinement
- Week 12 Friday: Final demos (in-person or video)

**Resources**:
- Office hours: Tuesday/Thursday 2-4pm
- Slack channel: #capstone-help
- Hardware lab: Access by appointment

**Submission**:
- GitHub repo link (public or shared with instructors)
- Technical report (PDF)
- Video upload (YouTube or Vimeo link)
- Slides (PDF format)

**Presentation Day**:
- 5-minute demo
- 3-minute Q&A
- Peer evaluation (10% of grade)

---

## Success Tips

1. **Start Early**: Integration always takes longer than expected
2. **Test Incrementally**: Verify each subsystem before integrating
3. **Document as You Go**: Don't wait until the end
4. **Version Control**: Commit frequently, use branches
5. **Ask for Help**: Use office hours and Slack

---

**Word Count**: ~500 words
