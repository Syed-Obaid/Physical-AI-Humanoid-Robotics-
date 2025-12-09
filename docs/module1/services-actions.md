---
id: services-actions
title: "Chapter 1.5: Services and Actions"
sidebar_label: "1.5 Services & Actions"
sidebar_position: 5
description: Learn synchronous services and goal-oriented actions for robot commands
keywords: [ROS 2 services, actions, request-reply, goal-based, feedback]
---

# Chapter 1.5: Services and Actions

## Learning Objectives

By the end of this chapter, you will be able to:

1. Differentiate between topics, services, and actions based on communication patterns
2. Implement a simple ROS 2 service for request-reply interactions
3. Use action servers for long-running tasks with feedback
4. Decide when to use each communication primitive for robot control

## Prerequisites

### Required Knowledge
- Python async programming basics (futures, callbacks)
- Understanding of client-server architecture
- ROS 2 topics and nodes (Chapter 1.4)

### Previous Chapters
- [Chapter 1.4: Nodes and Topics](./nodes-topics.md) - Publisher-subscriber pattern

## Content

### When to Use Services vs Topics

ROS 2 provides three communication patterns, each optimized for different scenarios:

**Topics (Asynchronous Streaming)**:
- **Use for**: Continuous sensor data, robot state updates
- **Example**: `/camera/image_raw` published at 30 FPS
- **Characteristics**: Fire-and-forget, one-to-many, no response guarantee

**Services (Synchronous Request-Reply)**:
- **Use for**: Occasional commands requiring confirmation
- **Example**: `/get_robot_pose` service returns current position
- **Characteristics**: Blocking call, one-to-one, guaranteed response

**Actions (Asynchronous Goal-Oriented)**:
- **Use for**: Long-running operations with progress tracking
- **Example**: `/navigate_to_goal` action reports distance remaining every 0.5s
- **Characteristics**: Non-blocking, cancellable, feedback stream + final result

**Decision flowchart**:
```
Need data continuously? → Use TOPIC
Need immediate response? → Use SERVICE
Task takes >1 second with progress? → Use ACTION
```

### Service Definitions

A ROS 2 service consists of two message types: **Request** and **Response**.

**Example service definition** (`GetJointLimits.srv`):
```
# Request
string joint_name
---
# Response
float64 lower_limit
float64 upper_limit
bool success
string message
```

**Built-in service types** (in `std_srvs` and `example_interfaces` packages):
- `std_srvs/Trigger`: No request data, returns success flag + message
- `std_srvs/SetBool`: Single boolean parameter, returns success
- `example_interfaces/AddTwoInts`: Adds two integers, returns sum

### Service Server Implementation

Create a service that resets a robot joint to zero position:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ResetJointService(Node):
    def __init__(self):
        super().__init__('reset_joint_service')

        # Create service: service name, service type, callback
        self.srv = self.create_service(
            Trigger,
            'reset_joint',
            self.reset_callback
        )
        self.joint_position = 1.57  # Current position (90°)
        self.get_logger().info('Reset Joint Service ready')

    def reset_callback(self, request, response):
        """
        Service callback: receives request, returns response
        """
        # Perform reset operation
        old_position = self.joint_position
        self.joint_position = 0.0

        # Populate response
        response.success = True
        response.message = f'Joint reset from {old_position:.2f} to 0.0 rad'

        self.get_logger().info(f'Service called: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetJointService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Key points**:
- Service callbacks are **blocking**: the server waits for callback to complete before accepting new requests
- Return `response` object from callback
- Services are **synchronous** from client perspective but **asynchronous** from network layer (uses DDS request-reply pattern)

### Service Client Implementation

Call the reset service from another node:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ResetJointClient(Node):
    def __init__(self):
        super().__init__('reset_joint_client')
        self.client = self.create_client(Trigger, 'reset_joint')

        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for reset_joint service...')

    def send_request(self):
        request = Trigger.Request()
        # Trigger service has no request fields

        # Call service asynchronously
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Get result
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'Service response: {result.message}')
            return result.success
        else:
            self.get_logger().error('Service call failed')
            return False

def main(args=None):
    rclpy.init(args=args)
    client = ResetJointClient()
    success = client.send_request()
    client.destroy_node()
    rclpy.shutdown()
```

**Alternative: Command-line service call**:
```bash
ros2 service call /reset_joint std_srvs/srv/Trigger
```

### Actions for Long-Running Tasks

Actions extend the service model with:
1. **Goal**: Task parameters (e.g., target position)
2. **Feedback**: Progress updates during execution
3. **Result**: Final outcome when task completes

**Example action definition** (`NavigateToGoal.action`):
```
# Goal
geometry_msgs/Pose target_pose
---
# Result
bool success
float64 final_distance_error
---
# Feedback
float64 distance_remaining
float64 time_elapsed
```

**Action server** (simplified example):

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from my_interfaces.action import NavigateToGoal
import time

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        """
        Executes navigation task, publishes feedback periodically
        """
        self.get_logger().info(f'Navigating to: {goal_handle.request.target_pose}')

        # Simulate navigation with 10 steps
        feedback_msg = NavigateToGoal.Feedback()

        for i in range(10):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToGoal.Result()

            # Publish feedback
            feedback_msg.distance_remaining = 10.0 - i
            feedback_msg.time_elapsed = i * 1.0
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1.0)  # Simulate 1 second per navigation step

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return final result
        result = NavigateToGoal.Result()
        result.success = True
        result.final_distance_error = 0.05  # 5cm error
        return result

def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()
    rclpy.spin(node)
```

**Action client** can:
- Send goal
- Subscribe to feedback updates
- Cancel goal mid-execution
- Get final result when complete

### Communication Pattern Decision Matrix

| Pattern | Latency | Direction | Use Case | Example |
|---------|---------|-----------|----------|---------|
| **Topic** | Low (&lt;1ms) | One-to-many | Continuous streams | Joint states at 50 Hz |
| **Service** | Medium (~10ms) | One-to-one | Occasional commands | Reset simulation |
| **Action** | High (seconds) | One-to-one with feedback | Long tasks | Navigate 10 meters |

**When to use each**:
- **Topic**: Sensor data, robot state, control commands sent repeatedly
- **Service**: Configuration changes, queries, reset/initialization
- **Action**: Motion planning, object manipulation, autonomous navigation

## Summary

### Key Takeaways
- **Services are synchronous request-reply**: Client blocks until server responds
- **Actions add feedback and cancellation**: Suitable for long-running tasks (>1 second)
- **Service definitions**: Split into Request and Response sections (separated by `---`)
- **Action definitions**: Include Goal, Result, and Feedback
- **CLI tools**: `ros2 service call`, `ros2 action send_goal` for testing
- **Decision criteria**: Latency requirements and task duration determine which pattern to use

### What's Next
In Chapter 1.6, you'll complete 18 graded exercises to demonstrate mastery of all Module 1 concepts.

## Exercises

1. **Exercise 1.10** (⭐ Easy): Call a service from the command line using:
   ```bash
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"
   ```
   Verify the service returns 8 as the sum.

2. **Exercise 1.11** (⭐⭐ Medium): Create a service `/set_joint_position` that takes a joint name (string) and target angle (float64), then returns success (bool) and confirmation message (string). Implement both server and client nodes.

3. **Exercise 1.12** (⭐⭐⭐ Hard): Implement an action server for `/move_arm` that:
   - Accepts 3 target joint angles (goal)
   - Moves joints using cubic trajectory interpolation over 5 seconds
   - Publishes feedback with progress percentage every 0.5s
   - Returns final position error (result)

   **Grading criteria** (15 points):
   - Action accepts valid goal and rejects invalid angles outside joint limits (5 pts)
   - Feedback published at correct rate (5 pts)
   - Technical report includes motion profile plot showing smooth trajectory (5 pts)

## References

- Robot Operating System 2. (2023). *About ROS 2 services*. Retrieved December 7, 2025, from https://docs.ros.org/en/humble/Tutorials/Services/Understanding-ROS2-Services.html
- Robot Operating System 2. (2023). *About ROS 2 actions*. Retrieved December 7, 2025, from https://docs.ros.org/en/humble/Tutorials/Actions/Understanding-ROS2-Actions.html
- Pyo, Y., Cho, H., Jung, R., & Lim, T. (2017). *ROS Robot Programming*. ROBOTIS Co., Ltd.

---

**Word Count**: ~730 words
