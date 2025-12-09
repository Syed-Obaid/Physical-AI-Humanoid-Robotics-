---
id: reinforcement-learning
title: "Chapter 4.4: Reinforcement Learning"
sidebar_label: "4.4 Reinforcement Learning"
sidebar_position: 4
description: Training robot policies with RL in Isaac Sim and deploying to hardware
keywords: [reinforcement learning, PPO, SAC, Isaac Gym, sim-to-real, policy training]
---

# Chapter 4.4: Reinforcement Learning

## Learning Objectives

By the end of this chapter, you will be able to:

1. Understand reinforcement learning fundamentals for robotics
2. Set up Isaac Gym for parallel policy training
3. Implement reward functions for manipulation tasks
4. Train policies using PPO (Proximal Policy Optimization)
5. Apply sim-to-real transfer techniques

## Prerequisites

### Required Knowledge
- PyTorch basics (neural networks, backpropagation)
- Markov Decision Processes (states, actions, rewards)
- Isaac Sim installation (from Module 2)
- Python async programming

### Previous Chapters
- [Chapter 2.1: Digital Twin Overview](../module2/overview.md)
- [Chapter 4.1: Embodied AI Overview](./overview.md)

## Content

### Reinforcement Learning Basics

**Goal**: Learn policy π(action | state) that maximizes cumulative reward.

#### Key Components

**State (s)**: Robot observation (joint angles, object positions, camera images)
```python
state = {
    'joint_positions': [θ1, θ2, ..., θ7],  # 7-DOF arm
    'joint_velocities': [dθ1, dθ2, ..., dθ7],
    'object_position': [x, y, z],
    'goal_position': [x_goal, y_goal, z_goal]
}
```

**Action (a)**: Robot command (joint torques or target positions)
```python
action = [τ1, τ2, ..., τ7]  # Torques for each joint
```

**Reward (r)**: Scalar feedback signal
```python
reward = -distance_to_goal  # Negative distance (minimize)
if gripper_touches_object:
    reward += 10  # Bonus for contact
if object_grasped:
    reward += 100  # Large bonus for success
```

**Policy (π)**: Neural network mapping states to actions
```python
class Policy(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim),
            nn.Tanh()  # Action bounds [-1, 1]
        )

    def forward(self, state):
        return self.net(state)
```

### Isaac Gym for Parallel Training

**Isaac Gym** simulates thousands of environments in parallel on GPU—critical for sample-efficient RL.

#### Installation

```bash
# Install IsaacGym from NVIDIA (requires registration)
# Download from: https://developer.nvidia.com/isaac-gym

# Extract and install
cd isaacgym/python
pip install -e .

# Install RL library
pip install stable-baselines3 rl_games
```

#### Creating a Custom Task

```python
from isaacgym import gymapi, gymtorch
import torch

class ReachTask:
    def __init__(self, num_envs=4096):
        self.gym = gymapi.acquire_gym()
        self.num_envs = num_envs

        # Create simulation
        sim_params = gymapi.SimParams()
        sim_params.dt = 1.0 / 60.0  # 60 Hz
        sim_params.substeps = 2
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

        # Load robot asset
        asset_root = "assets"
        robot_asset = self.gym.load_asset(self.sim, asset_root, "robot_arm.urdf")

        # Create environments
        self.envs = []
        for i in range(num_envs):
            env = self.gym.create_env(self.sim, gymapi.Vec3(-1, -1, 0), gymapi.Vec3(1, 1, 2), 4)
            actor = self.gym.create_actor(env, robot_asset, gymapi.Transform(), "robot", i, 0)
            self.envs.append(env)

        # Prepare for GPU tensor access
        self.gym.prepare_sim(self.sim)

    def reset(self):
        """Reset all environments to initial state."""
        # Randomize goal positions
        self.goal_positions = torch.rand((self.num_envs, 3)) * 0.5  # Random positions in 0.5m cube
        # Reset robot to home position
        # ...
        return self.get_observations()

    def get_observations(self):
        """Get states for all environments."""
        # Read robot state tensors from GPU
        dof_states = self.gym.acquire_dof_state_tensor(self.sim)
        dof_pos = dof_states[:, :, 0]  # Joint positions
        dof_vel = dof_states[:, :, 1]  # Joint velocities

        # Compute end-effector positions via forward kinematics
        ee_positions = self.compute_fk(dof_pos)

        # Concatenate state
        obs = torch.cat([dof_pos, dof_vel, ee_positions, self.goal_positions], dim=-1)
        return obs

    def step(self, actions):
        """Apply actions and step simulation."""
        # Set joint targets (position control)
        self.gym.set_dof_position_target_tensor(self.sim, gymtorch.unwrap_tensor(actions))

        # Step physics
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Compute rewards
        obs = self.get_observations()
        ee_positions = self.compute_fk(obs[:, :7])
        distances = torch.norm(ee_positions - self.goal_positions, dim=-1)
        rewards = -distances  # Negative distance

        # Check if reached goal (within 0.02m)
        dones = (distances < 0.02)
        rewards[dones] += 100  # Bonus for success

        return obs, rewards, dones, {}
```

**Performance**: 4096 environments × 60 fps = 245,760 samples/second on single RTX 4090.

### Designing Reward Functions

**Principle**: Reward should be **dense** (provide signal at every step) and **shaped** (guide toward solution).

#### Sparse vs Dense Rewards

**Sparse Reward** (Hard):
```python
reward = 1.0 if task_complete else 0.0
```
**Problem**: Robot rarely stumbles upon success → slow learning

**Dense Reward** (Better):
```python
# Reaching task
distance = torch.norm(ee_pos - goal_pos)
reward = -distance  # Continuous signal

# Additional shaping
if distance < 0.05:
    reward += 5  # Close to goal
if distance < 0.02:
    reward += 20  # Reached goal
```

#### Example: Pick-and-Place Reward

```python
def compute_reward(state):
    gripper_pos = state['gripper_position']
    object_pos = state['object_position']
    goal_pos = state['goal_position']

    # Phase 1: Reach object
    dist_to_object = torch.norm(gripper_pos - object_pos, dim=-1)
    reward = -dist_to_object

    # Phase 2: Grasp object (detect contact)
    grasped = state['gripper_force'] > 5.0  # N
    reward += 10 * grasped

    # Phase 3: Lift object
    if grasped:
        object_height = object_pos[:, 2]  # Z coordinate
        reward += object_height * 5  # Encourage lifting

    # Phase 4: Move to goal
    if grasped and object_height > 0.3:
        dist_to_goal = torch.norm(object_pos - goal_pos, dim=-1)
        reward -= dist_to_goal * 2

    # Success bonus
    if dist_to_goal < 0.05 and grasped:
        reward += 100

    return reward
```

**Design Tips**:
- Start with simple reward (just distance)
- Add bonuses incrementally
- Visualize learning curves to debug reward shaping
- Avoid conflicting terms (e.g., "minimize time" + "maximize safety" may cause thrashing)

### Training with PPO

**PPO (Proximal Policy Optimization)**: Most popular RL algorithm for robotics (used by OpenAI, Google, Tesla).

#### Implementation with Stable-Baselines3

```python
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

# Wrap Isaac Gym task as gym environment
class IsaacGymEnv:
    def __init__(self):
        self.task = ReachTask(num_envs=4096)

    def reset(self):
        return self.task.reset().cpu().numpy()

    def step(self, actions):
        obs, reward, done, info = self.task.step(torch.from_numpy(actions))
        return obs.cpu().numpy(), reward.cpu().numpy(), done.cpu().numpy(), info

# Create vectorized environment
env = DummyVecEnv([lambda: IsaacGymEnv()])

# Initialize PPO agent
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    n_steps=2048,        # Steps per update
    batch_size=512,      # Minibatch size
    learning_rate=3e-4,
    gamma=0.99,          # Discount factor
    tensorboard_log="./logs/"
)

# Train for 1M steps
model.learn(total_timesteps=1_000_000)

# Save policy
model.save("reach_policy")
```

**Hyperparameter Notes**:
- **learning_rate**: 3e-4 is good default; decrease if training unstable
- **gamma**: 0.99 for long-horizon tasks, 0.95 for short tasks
- **n_steps**: Larger = more stable but slower updates

#### Monitoring Training

```bash
# Start TensorBoard
tensorboard --logdir ./logs/

# Open browser: http://localhost:6006
```

**Key Metrics**:
- **ep_rew_mean**: Average episode reward (should increase)
- **ep_len_mean**: Average episode length
- **loss/value_loss**: Critic loss (should decrease)
- **approx_kl**: KL divergence (if >0.05, learning rate too high)

### Sim-to-Real Transfer

**Challenge**: Policies trained in perfect simulation fail on real robots due to:
- Unmodeled dynamics (motor backlash, cable flex)
- Sensor noise (IMU drift, camera blur)
- Time delays (actuator latency)

#### Domain Randomization

Randomize simulation parameters during training:

```python
# Randomize robot mass
mass_scale = torch.rand(num_envs) * 0.4 + 0.8  # 0.8-1.2×
for i, env in enumerate(envs):
    for link in robot_links:
        original_mass = link.mass
        link.mass = original_mass * mass_scale[i]

# Randomize friction
friction_coef = torch.rand(num_envs) * 0.6 + 0.3  # 0.3-0.9
for i, env in enumerate(envs):
    ground.friction = friction_coef[i]

# Add sensor noise
joint_pos_noisy = joint_pos + torch.randn_like(joint_pos) * 0.01  # ±0.01 rad
```

**Effect**: Policy learns to be robust to parameter variations → transfers better to real hardware.

#### System Identification

Measure real robot parameters:

```python
# Measure actual link masses (weigh links before assembly)
measured_masses = [0.95, 1.12, 0.88, ...]  # kg

# Update URDF
for i, link in enumerate(robot_urdf.links):
    link.inertial.mass = measured_masses[i]
```

**Tip**: Prioritize accuracy of distal links (gripper, wrist)—errors propagate less than base links.

#### Residual Learning

Train RL policy to **correct** model-based controller:

```python
# Model-based controller (PD control)
tau_pd = Kp * (q_desired - q) + Kd * (dq_desired - dq)

# RL correction
tau_correction = policy(state)

# Final command
tau_total = tau_pd + tau_correction
```

**Advantage**: Guarantees baseline safety (PD controller won't cause wild motions), RL only learns residuals.

### Deploying Policy to ROS 2

#### Export Trained Policy

```python
# Save policy as ONNX (cross-platform format)
import torch.onnx

dummy_input = torch.randn(1, state_dim)
torch.onnx.export(model.policy.actor, dummy_input, "policy.onnx")
```

#### Inference Node

```python
import onnxruntime as ort
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class PolicyInferenceNode(Node):
    def __init__(self):
        super().__init__('policy_inference')

        # Load ONNX model
        self.session = ort.InferenceSession("policy.onnx")

        # Subscribe to robot state
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10
        )

        # Publish actions
        self.action_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )

    def state_callback(self, msg):
        # Prepare state
        state = np.array(msg.position + msg.velocity)  # Concatenate

        # Run inference
        action = self.session.run(None, {'input': state[None, :]})[0][0]

        # Publish
        action_msg = Float64MultiArray()
        action_msg.data = action.tolist()
        self.action_pub.publish(action_msg)
```

**Latency**: ONNX Runtime achieves 1-5ms inference on CPU → suitable for 100Hz control loop.

### Practical Example: Object Reaching

```python
# Complete training script
def train_reaching_policy():
    # 1. Create task
    task = ReachTask(num_envs=8192)

    # 2. Domain randomization
    task.randomize_masses()
    task.randomize_friction()

    # 3. Train with PPO
    model = PPO("MlpPolicy", task, learning_rate=3e-4)
    model.learn(total_timesteps=5_000_000)

    # 4. Evaluate in sim
    success_rate = evaluate_policy(model, task, n_episodes=100)
    print(f"Sim success rate: {success_rate:.2%}")

    # 5. Export and deploy
    torch.onnx.export(model.policy.actor, dummy_input, "reach_policy.onnx")
    print("Policy saved to reach_policy.onnx")

    # 6. Test on real robot
    # Deploy via ROS 2 node (see above)
```

## Summary

### Key Takeaways
- **RL basics**: States, actions, rewards, policy networks
- **Isaac Gym**: GPU-parallelized simulation (245k+ samples/sec on RTX 4090)
- **Reward design**: Dense rewards with shaped bonuses outperform sparse rewards
- **PPO**: Industry-standard RL algorithm (stable, sample-efficient)
- **Sim-to-real transfer**: Domain randomization, system ID, residual learning
- **Deployment**: Export to ONNX, run inference at 1-5ms latency in ROS 2

### What's Next
Complete the capstone exercises in [Module 4 Exercises](./exercises.md) to build a full VLA pipeline with vision, language, and RL-trained policies.

## Exercises

See [Module 4 Exercises](./exercises.md) - Exercise 4.3 covers RL policy training and Exercise 4.5 integrates all components for household tasks.

## References

- Schulman, J., et al. (2017). Proximal Policy Optimization Algorithms. *arXiv:1707.06347*. https://arxiv.org/abs/1707.06347
- Tobin, J., et al. (2017). Domain randomization for transferring deep neural networks from simulation to the real world. *IROS 2017*. https://arxiv.org/abs/1703.06907
- Makoviychuk, V., et al. (2021). Isaac Gym: High performance GPU-based physics simulation for robot learning. *arXiv:2108.10470*. https://arxiv.org/abs/2108.10470

---

**Word Count**: ~900 words
