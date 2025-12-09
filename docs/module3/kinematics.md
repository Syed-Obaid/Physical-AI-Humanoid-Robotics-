---
id: kinematics
title: "Chapter 3.3: Inverse Kinematics"
sidebar_label: "3.3 Inverse Kinematics"
sidebar_position: 3
description: Deep dive into IK solvers for humanoid arms - KDL, TRAC-IK, and analytical solutions
keywords: [inverse kinematics, KDL, TRAC-IK, IKFast, redundancy resolution, Jacobian]
---

# Chapter 3.3: Inverse Kinematics

## Learning Objectives

By the end of this chapter, you will be able to:

1. Implement forward kinematics using Denavit-Hartenberg parameters
2. Understand numerical vs analytical IK approaches
3. Configure and use TRAC-IK for humanoid arms
4. Handle kinematic redundancy and singularities
5. Optimize IK solutions using null-space projections

## Prerequisites

### Required Knowledge
- Linear algebra (matrices, transformations, eigenvalues)
- Calculus (partial derivatives, Jacobian matrices)
- ROS 2 TF2 for coordinate transformations

### Previous Chapters
- [Chapter 3.1: Motion Planning Overview](./overview.md)
- [Chapter 3.2: MoveIt Setup](./moveit-setup.md)
- [Chapter 1.4: URDF Basics](../module1/urdf-basics.md)

## Content

### The Inverse Kinematics Problem

**Forward Kinematics (FK)**: Given joint angles θ = [θ₁, θ₂, ..., θₙ], compute end-effector pose (position + orientation)
```
FK: θ → (x, y, z, qx, qy, qz, qw)
```

**Inverse Kinematics (IK)**: Given desired end-effector pose, find joint angles that achieve it
```
IK: (x, y, z, qx, qy, qz, qw) → θ
```

**Challenge**: IK is inherently difficult:
1. **Non-unique**: 7-DOF arms have infinite solutions for any reachable pose
2. **Non-existence**: Target poses outside workspace have no solution
3. **Nonlinearity**: Trigonometric functions make IK equations nonlinear

### Forward Kinematics with DH Parameters

**Denavit-Hartenberg (DH) Convention**: Standard method to attach coordinate frames to robot links.

**Four DH Parameters per joint**:
- **a**: Link length (distance along x from old z to new z)
- **α (alpha)**: Link twist (rotation about x from old z to new z)
- **d**: Link offset (distance along z from old x to new x)
- **θ (theta)**: Joint angle (rotation about z from old x to new x)

**Homogeneous Transform from joint i-1 to i**:
```
T_i = Rz(θ_i) * Tz(d_i) * Tx(a_i) * Rx(α_i)
    = [cos(θ_i)  -sin(θ_i)cos(α_i)   sin(θ_i)sin(α_i)   a_i*cos(θ_i)]
      [sin(θ_i)   cos(θ_i)cos(α_i)  -cos(θ_i)sin(α_i)   a_i*sin(θ_i)]
      [    0           sin(α_i)          cos(α_i)             d_i     ]
      [    0               0                 0                1       ]
```

**Example: 3-DOF Planar Arm**

DH Table:
| Joint | a   | α   | d   | θ   |
|-------|-----|-----|-----|-----|
| 1     | L1  | 0   | 0   | θ1* |
| 2     | L2  | 0   | 0   | θ2* |
| 3     | L3  | 0   | 0   | θ3* |

Forward kinematics:
```python
import numpy as np

def fk_planar_arm(theta, L):
    """Forward kinematics for 3-DOF planar arm."""
    theta1, theta2, theta3 = theta
    L1, L2, L3 = L

    x = L1*np.cos(theta1) + L2*np.cos(theta1+theta2) + L3*np.cos(theta1+theta2+theta3)
    y = L1*np.sin(theta1) + L2*np.sin(theta1+theta2) + L3*np.sin(theta1+theta2+theta3)
    phi = theta1 + theta2 + theta3  # end-effector orientation

    return x, y, phi
```

### Numerical IK: Jacobian-Based Methods

Numerical IK solves the problem iteratively using the **Jacobian matrix** (relates joint velocities to end-effector velocities).

#### Jacobian Matrix

For an n-DOF robot:
```
J(θ) = ∂(x, y, z) / ∂(θ₁, ..., θₙ)
```

Each column of J represents how the end-effector moves when joint i changes (with all other joints fixed).

#### Newton-Raphson IK Algorithm

```
while |error| > tolerance:
    error = target_pose - current_pose
    Δθ = J^(-1) * error
    θ = θ + α * Δθ  # α is step size
```

**Problem**: J^(-1) doesn't exist at singularities (when J loses rank).

#### Damped Least Squares (Levenberg-Marquardt)

More robust approach that handles singularities:
```
Δθ = J^T * (J*J^T + λ²I)^(-1) * error
```

Parameter λ (damping factor) prevents large steps near singularities.

### KDL Kinematics Plugin

**KDL (Kinematics and Dynamics Library)** provides numerical IK solvers for MoveIt.

#### Configuration in kinematics.yaml

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005  # Joint angle discretization
  kinematics_solver_timeout: 0.05             # Max solve time (seconds)
  kinematics_solver_attempts: 3                # Retry attempts
```

#### Using KDL Programmatically

```python
import rclpy
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

def solve_ik(pose_target):
    """Call KDL IK service to solve for joint angles."""
    node = rclpy.create_node('ik_client')
    client = node.create_client(GetPositionIK, '/compute_ik')

    request = GetPositionIK.Request()
    request.ik_request.group_name = "arm"
    request.ik_request.pose_stamped = pose_target
    request.ik_request.timeout.sec = 1

    response = client.call(request)
    if response.error_code.val == response.error_code.SUCCESS:
        return response.solution.joint_state
    else:
        return None
```

**Limitations of KDL**:
- Slow convergence (50-200 ms per solve)
- Fails ~15-20% of time for reachable poses
- Poor performance near singularities

### TRAC-IK: Improved Numerical Solver

**TRAC-IK (Tracy's Inverse Kinematics)** combines gradient descent with random reseeding for better convergence.

#### Installation

```bash
sudo apt install ros-humble-trac-ik-kinematics-plugin
```

#### Configuration

```yaml
arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  solve_type: Distance  # or "Speed" or "Manipulation"
```

**Solve Types**:
- **Speed**: Fastest, less accurate
- **Distance**: Balanced (default)
- **Manipulation**: Most accurate, slower

#### Performance Comparison

| Metric               | KDL      | TRAC-IK  |
|----------------------|----------|----------|
| Success Rate         | 82%      | 96%      |
| Avg Solve Time       | 180 ms   | 12 ms    |
| Near Singularities   | Fails    | Robust   |

**Recommendation**: Use TRAC-IK for humanoid robots—its superior success rate prevents planning failures.

### Handling Kinematic Redundancy

A 7-DOF humanoid arm is **redundant** (more DOF than needed for 6-DOF end-effector pose). Infinite IK solutions exist.

#### Null-Space Optimization

The **null space** of Jacobian represents joint motions that don't affect end-effector.

**Secondary Objective**: While maintaining end-effector pose, optimize for:
1. **Manipulability**: Maximize distance from singularities
2. **Joint Limits**: Stay away from joint range limits
3. **Collision Avoidance**: Prefer elbow-up vs elbow-down configurations

**Algorithm**:
```
Primary task: Δθ_primary = J^+ * Δx  (reach target)
Secondary task: Δθ_secondary = (I - J^+J) * ∂f/∂θ  (null-space gradient)
Combined: Δθ = Δθ_primary + α * Δθ_secondary
```

Where J^+ is the pseudoinverse and (I - J^+J) projects into null space.

#### Example: Maximizing Manipulability

Manipulability measure:
```
w(θ) = sqrt(det(J * J^T))
```

High w means robot is far from singularities (can move freely in all directions).

```python
def manipulability(theta):
    J = compute_jacobian(theta)
    return np.sqrt(np.linalg.det(J @ J.T))

def null_space_gradient(theta):
    """Gradient of manipulability for secondary objective."""
    eps = 1e-6
    grad = np.zeros_like(theta)
    w0 = manipulability(theta)
    for i in range(len(theta)):
        theta_plus = theta.copy()
        theta_plus[i] += eps
        grad[i] = (manipulability(theta_plus) - w0) / eps
    return grad
```

### Singularities and Avoidance

**Singularity**: Configuration where Jacobian loses rank (end-effector loses DOF).

**Types**:
1. **Workspace Boundary**: Arm fully extended (can't move further away)
2. **Wrist Singularity**: Two joint axes align
3. **Elbow Singularity**: Elbow fully bent or straight

**Example**: 3-DOF planar arm at full extension—Jacobian rank drops from 3 to 2 (can't move outward).

**Detection**:
```python
def is_near_singularity(theta, threshold=0.01):
    J = compute_jacobian(theta)
    manipulability = np.sqrt(np.linalg.det(J @ J.T))
    return manipulability < threshold
```

**Avoidance Strategies**:
1. **Penalty in Cost Function**: Add cost proportional to 1/manipulability during planning
2. **Filter IK Solutions**: Reject solutions with low manipulability
3. **Null-Space Repulsion**: Use secondary task to move away from singularities

### Analytical IK with IKFast

For specific robot geometries, **closed-form IK solutions** exist (e.g., 6R arms with spherical wrist).

#### Advantages of Analytical IK
- **Speed**: Microseconds vs milliseconds
- **Completeness**: Returns all solutions (up to 16 for 6-DOF arms)
- **Reliability**: No convergence failures

#### Generating IKFast Plugin

```bash
# Install OpenRAVE (IKFast generator)
sudo apt install openrave

# Generate IK solver from URDF
openrave.py --database inversekinematics --robot=my_robot.urdf --iktype=transform6d

# Compile generated C++ code into MoveIt plugin
# (Detailed steps at: http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html)
```

**Limitation**: Only works for arms with specific geometric properties (e.g., 3 consecutive intersecting joint axes). Most custom humanoid arms don't qualify.

## Summary

### Key Takeaways
- **Forward kinematics** uses DH parameters to compute end-effector pose from joint angles
- **Numerical IK** (KDL, TRAC-IK) uses Jacobian-based iterative methods
- **TRAC-IK** outperforms KDL with 96% success rate vs 82% and 15× faster solving
- **Redundant arms** (7+ DOF) have infinite IK solutions—use null-space optimization for secondary objectives
- **Singularities** occur when Jacobian loses rank—avoid with manipulability-based cost functions
- **Analytical IK** (IKFast) is fastest but requires specific robot geometry

### What's Next
In Chapter 3.4, you'll implement trajectory planning with OMPL and optimize paths for smoothness and collision avoidance.

## Exercises

See [Module 3 Exercises](./exercises.md) - Exercise 3.3 covers implementing IK solvers for 5-DOF humanoid arm.

## References

- Beeson, P., & Ames, B. (2015). TRAC-IK: An open-source library for improved solving of generic inverse kinematics. *2015 IEEE-RAS International Conference on Humanoid Robots*, 928-935. https://doi.org/10.1109/HUMANOIDS.2015.7363472
- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. (Chapter 3: Differential Kinematics)
- Diankov, R. (2010). Automated construction of robotic manipulation programs. *PhD thesis*, Carnegie Mellon University. (IKFast method)

---

**Word Count**: ~800 words
