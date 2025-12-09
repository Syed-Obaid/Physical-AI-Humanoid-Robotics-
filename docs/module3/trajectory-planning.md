---
id: trajectory-planning
title: "Chapter 3.4: Trajectory Planning"
sidebar_label: "3.4: Trajectory Planning"
sidebar_position: 4
description: Path planning with OMPL, trajectory optimization, and collision avoidance
keywords: [RRT, RRT*, PRM, OMPL, trajectory optimization, CHOMP, collision checking]
---

# Chapter 3.4: Trajectory Planning

## Learning Objectives

By the end of this chapter, you will be able to:

1. Configure OMPL planners (RRT, RRT*, PRM) for robot arms
2. Understand the difference between geometric paths and time-parameterized trajectories
3. Implement collision checking with octomap-based environments
4. Optimize trajectories using CHOMP and TrajOpt
5. Plan bimanual coordinated motions for dual-arm humanoids

## Prerequisites

### Required Knowledge
- Motion planning fundamentals (configuration space, sampling-based algorithms)
- Collision detection concepts
- ROS 2 actions and services

### Previous Chapters
- [Chapter 3.1: Motion Planning Overview](./overview.md)
- [Chapter 3.2: MoveIt Setup](./moveit-setup.md)
- [Chapter 3.3: Inverse Kinematics](./kinematics.md)

## Content

### Geometric Path vs Trajectory

**Path**: Sequence of configurations (joint angles) without timing
```
Path = [θ₀, θ₁, θ₂, ..., θₙ]
```

**Trajectory**: Path + time parameterization (velocities and accelerations)
```
Trajectory = [(θ₀, t₀), (θ₁, t₁), ..., (θₙ, tₙ)]
```

**Planning Pipeline**:
1. **Geometric Planner**: Find collision-free path (OMPL)
2. **Path Smoother**: Reduce waypoints, smooth jerky motions
3. **Time Parameterization**: Add velocity/acceleration profiles respecting joint limits

### OMPL: Open Motion Planning Library

MoveIt integrates **OMPL** (Open Motion Planning Library)—a collection of sampling-based planners.

#### Common OMPL Planners

**RRT (Rapidly-Exploring Random Tree)**:
- Grows tree by random sampling
- Fast (0.1-2 seconds for typical arms)
- Paths are suboptimal and jerky
- **Best for**: Quick solutions when path quality isn't critical

**RRT\* (Optimal RRT)**:
- Rewires tree to improve path cost
- Asymptotically optimal (converges to shortest path)
- Slower than RRT (2-10× longer)
- **Best for**: When smooth, short paths are needed

**RRTConnect**:
- Grows two trees (from start and goal) simultaneously
- Fastest OMPL planner (often 2-3× faster than RRT)
- Not optimal
- **Best for**: Default choice for fast planning

**PRM (Probabilistic Roadmap)**:
- Preprocessing: Build roadmap of valid configurations
- Query: Connect start/goal to roadmap and search
- Good for repeated planning in same environment
- **Best for**: Factory settings where environment is static

**EST (Expansive Space Trees)**:
- Biases exploration toward unexplored regions
- Handles narrow passages better than RRT
- **Best for**: Complex environments with tight corridors

#### Configuring OMPL in MoveIt

Edit `config/ompl_planning.yaml`:

```yaml
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >-
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/FixWorkspaceBounds
    default_planner_request_adapters/FixStartStateBounds
    default_planner_request_adapters/FixStartStateCollision
    default_planner_request_adapters/FixStartStatePathConstraints

start_state_max_bounds_error: 0.1

arm:
  planner_configs:
    - RRTConnect
    - RRT
    - RRTstar
    - PRM

  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0  # 0 = automatic

  RRTstar:
    type: geometric::RRTstar
    range: 0.0
    goal_bias: 0.05            # Probability of sampling goal (5%)
    delay_collision_checking: true

  PRM:
    type: geometric::PRM
    max_nearest_neighbors: 10  # Roadmap connectivity
```

**Key Parameters**:
- **range**: Maximum step size for tree extension (0 = automatic)
- **goal_bias**: Probability of sampling goal state (balances exploration vs exploitation)
- **delay_collision_checking**: Only check collisions for new nodes (faster)

#### Using OMPL Programmatically

```python
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan

def plan_to_joint_target(joint_angles):
    """Plan to target joint configuration using RRTConnect."""
    node = rclpy.create_node('motion_planner')
    client = node.create_client(GetMotionPlan, '/plan_kinematic_path')

    request = GetMotionPlan.Request()
    request.motion_plan_request.group_name = "arm"
    request.motion_plan_request.num_planning_attempts = 5
    request.motion_plan_request.allowed_planning_time = 10.0
    request.motion_plan_request.planner_id = "RRTConnectkConfigDefault"

    # Set goal constraints
    for i, angle in enumerate(joint_angles):
        jc = JointConstraint()
        jc.joint_name = f"joint_{i}"
        jc.position = angle
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        request.motion_plan_request.goal_constraints.append(
            Constraints(joint_constraints=[jc])
        )

    response = client.call(request)
    if response.motion_plan_response.error_code.val == 1:  # SUCCESS
        return response.motion_plan_response.trajectory
    else:
        return None
```

### Collision Checking

#### Fast Collision Library (FCL)

MoveIt uses **FCL** for collision detection between:
- Robot links (self-collision)
- Robot and environment obstacles
- Robot and octomap (3D occupancy grid)

**Performance**: FCL checks ~100,000 collision queries/second using bounding volume hierarchies (BVH).

#### Adding Obstacles to Planning Scene

```python
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def add_box_obstacle(x, y, z, size_x, size_y, size_z):
    """Add box obstacle to planning scene."""
    collision_object = CollisionObject()
    collision_object.header.frame_id = "world"
    collision_object.id = "box_obstacle"

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [size_x, size_y, size_z]

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0

    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD

    # Publish to planning scene
    scene_pub = node.create_publisher(CollisionObject, '/collision_object', 10)
    scene_pub.publish(collision_object)
```

**Primitive Types**:
- `BOX`: Rectangular prism (dimensions: [x, y, z])
- `SPHERE`: Ball (dimensions: [radius])
- `CYLINDER`: Cylinder (dimensions: [height, radius])
- `CONE`: Cone (dimensions: [height, radius])

#### Octomap Integration for Dynamic Environments

**Octomap**: 3D occupancy grid built from depth cameras/lidars.

```yaml
# config/sensors_3d.yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/DepthImageOctomapUpdater
    image_topic: /camera/depth/image_raw
    queue_size: 5
    near_clipping_plane_distance: 0.3
    far_clipping_plane_distance: 5.0
    shadow_threshold: 0.2
    filtered_cloud_topic: /filtered_points
```

**Use Case**: Humanoid navigating cluttered household—octomap updates in real-time as robot discovers new obstacles.

### Path Smoothing and Optimization

OMPL paths are often jerky (many unnecessary waypoints). MoveIt applies post-processing:

#### Shortcutting

Iteratively tries to shortcut path by connecting non-adjacent waypoints:
```
Original: θ₀ → θ₁ → θ₂ → θ₃ → θ₄
Shortcut:  θ₀ -------→ θ₂ -------→ θ₄  (if collision-free)
```

**Algorithm**:
```python
def shortcut_path(path, max_iterations=100):
    for _ in range(max_iterations):
        i, j = random.sample(range(len(path)), 2)
        if i > j: i, j = j, i
        if is_collision_free(path[i], path[j]):
            path = path[:i+1] + path[j:]
    return path
```

#### Bspline Smoothing

Fits smooth B-spline curve through waypoints:
```yaml
# In ompl_planning.yaml
smoothing:
  type: time_parameterization
  velocity_scaling_factor: 0.1  # Max 10% of joint velocity limits
  acceleration_scaling_factor: 0.1
```

### CHOMP: Trajectory Optimization

**CHOMP (Covariant Hamiltonian Optimization)** refines an initial trajectory by gradient descent on cost function.

**Cost Function**:
```
Cost = λ_smooth * Smoothness + λ_obstacle * ObstacleCost
```

- **Smoothness**: Penalizes jerky motion (squared acceleration)
- **ObstacleCost**: Penalizes proximity to obstacles

#### Enabling CHOMP

```bash
sudo apt install ros-humble-moveit-planners-chomp
```

Edit `ompl_planning.yaml`:
```yaml
planning_plugin: chomp_interface/CHOMPPlanner
```

**Workflow**:
1. Use RRT to get initial collision-free path
2. Feed to CHOMP for smoothing and optimization
3. Time parameterization for velocity profiles

**When to Use**:
- Initial path exists (CHOMP doesn't handle disconnected C-space regions)
- Path needs smoothing for energy-efficient execution
- Environment has gentle obstacles (CHOMP struggles with narrow passages)

### Time Parameterization

Converts geometric path to trajectory by computing timestamps for each waypoint.

#### Time-Optimal Trajectory Generation (TOTG)

Finds fastest trajectory respecting velocity/acceleration limits.

**Algorithm**:
1. Compute maximum velocity at each waypoint (considering acceleration limits)
2. Forward pass: Accelerate maximally until velocity limit
3. Backward pass: Decelerate to avoid violating limits at goal

**Configuration**:
```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2  # Allow 20% extra time
  allowed_goal_duration_margin: 0.5        # 0.5s tolerance at goal
```

**Result**: Trajectory executes in minimum time without violating joint limits.

#### Cubic/Quintic Spline Interpolation

Alternative to TOTG—fits polynomial between waypoints:

**Cubic**: Ensures continuous velocity
**Quintic**: Ensures continuous acceleration (smoother)

```python
from scipy.interpolate import CubicSpline

def interpolate_trajectory(waypoints, times):
    cs = CubicSpline(times, waypoints)
    t_dense = np.linspace(times[0], times[-1], 1000)
    trajectory = cs(t_dense)
    return trajectory
```

### Bimanual Coordination for Humanoids

Dual-arm tasks (carrying tray, opening box) require coordinated planning.

#### Approach 1: Compound Planning Group

Define `both_arms` group containing left + right arm joints:
```yaml
# SRDF
<group name="both_arms">
  <chain base_link="torso" tip_link="left_gripper"/>
  <chain base_link="torso" tip_link="right_gripper"/>
</group>
```

Plan in combined 14-DOF space (7 per arm).

**Limitation**: Computationally expensive (14D C-space is large).

#### Approach 2: Sequential Planning

Plan each arm separately, checking for collisions with other arm:
```python
# Plan left arm
left_path = plan_arm("left_arm", left_goal)

# Add left arm trajectory as obstacle for right planning
add_trajectory_to_scene(left_path)

# Plan right arm (will avoid left arm)
right_path = plan_arm("right_arm", right_goal)
```

**Limitation**: Not optimal—simultaneous planning would find better solutions.

#### Approach 3: Multi-Goal Planning

Use MoveIt2's multi-goal planning:
```python
from moveit_msgs.msg import Constraints

request = GetMotionPlan.Request()
request.motion_plan_request.group_name = "both_arms"

# Goal for left end-effector
left_goal = create_pose_goal("left_gripper", x=0.5, y=0.3, z=1.0)
request.motion_plan_request.goal_constraints.append(left_goal)

# Goal for right end-effector
right_goal = create_pose_goal("right_gripper", x=0.5, y=-0.3, z=1.0)
request.motion_plan_request.goal_constraints.append(right_goal)
```

Planner finds joint angles satisfying both constraints simultaneously.

## Summary

### Key Takeaways
- **OMPL planners** find geometric paths: RRTConnect (fast), RRT* (optimal), PRM (repeated queries)
- **Collision checking** uses FCL library (~100k queries/sec) with primitives, meshes, and octomaps
- **Path smoothing** reduces waypoints via shortcutting and B-spline fitting
- **CHOMP** optimizes trajectories for smoothness and obstacle avoidance (requires initial collision-free path)
- **Time parameterization** converts paths to executable trajectories respecting joint limits
- **Bimanual planning** requires coordinated dual-arm motion—use compound groups or sequential planning

### What's Next
Complete the exercises in [Module 3 Exercises](./exercises.md) to implement motion planning for pick-and-place tasks.

## Exercises

See [Module 3 Exercises](./exercises.md) - Exercises 3.4-3.5 cover trajectory planning and capstone pick-and-place project.

## References

- Sucan, I. A., Moll, M., & Kavraki, L. E. (2012). The Open Motion Planning Library. *IEEE Robotics & Automation Magazine*, 19(4), 72-82. https://doi.org/10.1109/MRA.2012.2205651
- Ratliff, N., Zucker, M., Bagnell, J. A., & Srinivasa, S. (2009). CHOMP: Gradient optimization techniques for efficient motion planning. *2009 IEEE ICRA*, 489-494. https://doi.org/10.1109/ROBOT.2009.5152817
- Hauser, K., & Ng-Thow-Hing, V. (2010). Fast smoothing of manipulator trajectories using optimal bounded-acceleration shortcuts. *2010 IEEE ICRA*, 2493-2498.

---

**Word Count**: ~900 words
