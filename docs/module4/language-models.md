---
id: language-models
title: "Chapter 4.3: Language Model Integration"
sidebar_label: "4.3 Language Models"
sidebar_position: 3
description: Integrating LLMs for natural language command parsing and task planning
keywords: [LLM, GPT-4, Claude, natural language processing, task decomposition, ROS 2]
---

# Chapter 4.3: Language Model Integration

## Learning Objectives

By the end of this chapter, you will be able to:

1. Integrate OpenAI API or local LLMs with ROS 2
2. Parse natural language commands into structured robot actions
3. Implement task decomposition with chain-of-thought prompting
4. Handle ambiguity and ask clarification questions
5. Build a language-driven action server

## Prerequisites

### Required Knowledge
- ROS 2 services and actions
- Python async programming (asyncio)
- Basic natural language processing concepts
- JSON data structures

### Previous Chapters
- [Chapter 4.1: Embodied AI Overview](./overview.md)
- [Chapter 4.2: Vision Integration](./vision-integration.md)
- [Chapter 1.5: Services and Actions](../module1/services-actions.md)

## Content

### Why Language Models for Robotics?

Traditional robot interfaces require:
- Predefined commands ("move_to_pose x y z")
- Programming knowledge (Python/C++)
- Exact syntax

**Language Models** enable natural interaction:
```
User: "Can you grab the cup on the table and put it in the sink?"
Robot: [Decomposes task] → [Detects objects] → [Plans motion] → [Executes]
```

**Key Capabilities**:
1. **Command Parsing**: "Pick up the red cube" → `{action: 'pick', object: 'cube', color: 'red'}`
2. **Task Decomposition**: Complex goals → sequence of atomic actions
3. **Spatial Reasoning**: "Left of the mug" → relative position calculation
4. **Error Recovery**: Suggest alternatives when task fails

### LLM Options for Robotics

#### Cloud-Based APIs

**OpenAI GPT-4**
- **Pros**: State-of-the-art reasoning, multimodal (vision + text)
- **Cons**: API costs ($0.01-0.03 per 1K tokens), latency (200-500ms), requires internet
- **Use Case**: Research, prototyping, complex task planning

**Anthropic Claude 3.5**
- **Pros**: Strong reasoning, long context (200K tokens), lower latency
- **Cons**: API costs, internet dependency
- **Use Case**: Multi-step planning, document understanding

**Google Gemini 1.5**
- **Pros**: Fast, affordable, multimodal
- **Cons**: Less capable than GPT-4 on complex reasoning
- **Use Case**: Real-time command parsing

#### Local Open-Source Models

**LLaMA 3 (8B / 70B parameters)**
- **Pros**: Free, runs locally, no internet needed
- **Cons**: Requires GPU (RTX 3090+ for 70B), slower than APIs
- **Use Case**: On-robot deployment, privacy-sensitive applications

**Mistral 7B**
- **Pros**: Compact (7B params), fast inference (100 tokens/sec on GPU)
- **Cons**: Less capable than larger models
- **Use Case**: Embedded systems (Jetson AGX Orin)

**Phi-3 Mini (3.8B parameters)**
- **Pros**: Runs on CPU, very fast
- **Cons**: Limited reasoning for complex tasks
- **Use Case**: Simple command parsing on resource-constrained hardware

### Integrating OpenAI API with ROS 2

#### Installation

```bash
pip install openai pydantic
```

#### Command Parser Service

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
from openai import OpenAI
import json

class LLMCommandParser(Node):
    def __init__(self):
        super().__init__('llm_command_parser')

        # OpenAI client
        self.client = OpenAI(api_key="YOUR_API_KEY")

        # ROS 2 service
        self.service = self.create_service(
            Trigger,
            '/parse_command',
            self.parse_callback
        )

        # System prompt
        self.system_prompt = """
        You are a command parser for a humanoid robot. Convert natural language
        commands into structured JSON with these fields:
        - action: pick | place | navigate | look
        - object: target object name
        - location: target location
        - modifiers: {color, size, orientation}

        Example:
        Input: "Pick up the red cube on the table"
        Output: {"action": "pick", "object": "cube", "modifiers": {"color": "red", "location": "table"}}
        """

        self.get_logger().info('LLM Command Parser ready')

    def parse_callback(self, request, response):
        """Parse natural language command."""
        user_command = request.request  # Assuming command in request field

        try:
            # Call GPT-4
            completion = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_command}
                ],
                temperature=0.0,  # Deterministic output
                max_tokens=200
            )

            # Extract response
            parsed = completion.choices[0].message.content
            response.success = True
            response.message = parsed

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

def main():
    rclpy.init()
    node = LLMCommandParser()
    rclpy.spin(node)
    rclpy.shutdown()
```

#### Testing the Service

```bash
# Terminal 1: Run parser node
ros2 run my_package llm_command_parser

# Terminal 2: Call service
ros2 service call /parse_command example_interfaces/srv/Trigger "{request: 'Pick up the red cube'}"
```

**Expected Output**:
```json
{
  "success": true,
  "message": "{\"action\": \"pick\", \"object\": \"cube\", \"modifiers\": {\"color\": \"red\"}}"
}
```

### Task Decomposition with Chain-of-Thought

**Problem**: "Clean up the living room" is too abstract for direct execution.

**Solution**: Use LLM to break down into atomic actions.

#### Prompting Strategy

```python
decomposition_prompt = """
You are a task planner for a humanoid robot. Given a high-level goal,
decompose it into a sequence of atomic actions. Available actions:
- navigate(location)
- detect_objects(object_type)
- pick(object)
- place(object, location)

Output JSON list of actions with parameters.

Example:
Goal: "Put the cup in the dishwasher"
Output: [
  {"action": "detect_objects", "params": {"object_type": "cup"}},
  {"action": "navigate", "params": {"location": "cup_location"}},
  {"action": "pick", "params": {"object": "cup"}},
  {"action": "navigate", "params": {"location": "dishwasher"}},
  {"action": "place", "params": {"object": "cup", "location": "dishwasher"}}
]
"""

def decompose_task(goal):
    response = client.chat.completions.create(
        model="gpt-4-turbo",
        messages=[
            {"role": "system", "content": decomposition_prompt},
            {"role": "user", "content": f"Goal: {goal}"}
        ]
    )
    plan = json.loads(response.choices[0].message.content)
    return plan
```

**Usage in ROS 2 Action**:
```python
from action_msgs.msg import GoalStatus
from my_interfaces.action import ExecuteTask

class TaskExecutor(Node):
    def __init__(self):
        super().__init__('task_executor')
        self._action_server = ActionServer(
            self,
            ExecuteTask,
            'execute_task',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        # Decompose high-level goal
        plan = decompose_task(goal_handle.request.task_description)

        # Execute each step
        for step in plan:
            action = step['action']
            params = step['params']

            if action == 'navigate':
                self.navigate(params['location'])
            elif action == 'pick':
                self.pick_object(params['object'])
            elif action == 'place':
                self.place_object(params['object'], params['location'])

            # Publish feedback
            feedback = ExecuteTask.Feedback()
            feedback.current_step = action
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        return ExecuteTask.Result(success=True)
```

### Handling Spatial References

**Challenge**: "Put the apple to the left of the mug"—what does "left" mean?

#### Spatial Reasoning with LLM

```python
spatial_prompt = """
You are a spatial reasoning module. Given object positions and a spatial relation,
compute the target position.

Available relations: left_of, right_of, behind, in_front_of, above, below

Input format:
- reference_object: {name, position: {x, y, z}}
- relation: "left_of" | "right_of" | ...
- offset: distance in meters

Output: {x, y, z} of target position

Example:
reference_object: {"name": "mug", "position": {"x": 0.5, "y": 0.0, "z": 1.0}}
relation: "left_of"
offset: 0.15

Output: {"x": 0.5, "y": 0.15, "z": 1.0}  (0.15m to the left)
"""

def compute_spatial_position(reference_obj, relation, offset=0.15):
    query = f"""
    reference_object: {json.dumps(reference_obj)}
    relation: {relation}
    offset: {offset}
    """

    response = client.chat.completions.create(
        model="gpt-4-turbo",
        messages=[
            {"role": "system", "content": spatial_prompt},
            {"role": "user", "content": query}
        ]
    )

    target_pos = json.loads(response.choices[0].message.content)
    return target_pos
```

### Running Local LLMs with Ollama

**Ollama** simplifies local LLM deployment.

#### Installation

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull model
ollama pull llama3:8b
```

#### ROS 2 Node with Ollama

```python
import requests

class LocalLLMNode(Node):
    def __init__(self):
        super().__init__('local_llm_node')
        self.ollama_url = "http://localhost:11434/api/generate"

    def query_llm(self, prompt):
        payload = {
            "model": "llama3:8b",
            "prompt": prompt,
            "stream": False
        }

        response = requests.post(self.ollama_url, json=payload)
        return response.json()['response']
```

**Advantage**: No API costs, works offline, privacy-preserving.

**Limitation**: Slower inference (2-3 seconds on CPU vs 200ms for GPT-4 API).

### Handling Ambiguity

**Problem**: "Pick up the box"—which box if there are multiple?

#### Clarification Dialog

```python
def handle_ambiguous_command(command, detected_objects):
    """
    If command is ambiguous, ask for clarification.
    """
    # Check for ambiguity
    if command == "pick up the box" and len(detected_objects) > 1:
        # Generate clarification question
        clarification_prompt = f"""
        The user said: "{command}"
        Detected objects: {detected_objects}

        Generate a clarification question for the user.
        """

        response = client.chat.completions.create(
            model="gpt-4-turbo",
            messages=[{"role": "user", "content": clarification_prompt}]
        )

        question = response.choices[0].message.content
        # Publish question to /robot/speech
        self.speech_pub.publish(String(data=question))

        # Wait for user response
        # ...
```

**Example Output**: "I see 3 boxes—a red one, a blue one, and a large cardboard box. Which one would you like me to pick up?"

### Safety and Validation

**Problem**: LLMs can hallucinate invalid actions.

#### Validation Layer

```python
def validate_action(action):
    """
    Verify LLM output before execution.
    """
    valid_actions = ['pick', 'place', 'navigate', 'look']

    if action['action'] not in valid_actions:
        return False, f"Invalid action: {action['action']}"

    if action['action'] == 'navigate':
        # Check if location exists
        if not is_valid_location(action['params']['location']):
            return False, f"Unknown location: {action['params']['location']}"

    if action['action'] == 'pick':
        # Check if object exists in scene
        if not object_detected(action['params']['object']):
            return False, f"Object not found: {action['params']['object']}"

    return True, "Valid"
```

**Critical**: Always validate LLM outputs before sending commands to hardware.

### Practical Example: Voice-Commanded Manipulation

```python
from speech_recognition import Recognizer, Microphone

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.recognizer = Recognizer()
        self.llm_client = OpenAI(api_key="YOUR_API_KEY")

    def listen_and_execute(self):
        """
        1. Listen for voice command
        2. Parse with LLM
        3. Execute action
        """
        with Microphone() as source:
            print("Listening...")
            audio = self.recognizer.listen(source)

        # Speech to text
        command = self.recognizer.recognize_google(audio)
        print(f"You said: {command}")

        # Parse with LLM
        parsed = self.parse_command(command)

        # Validate
        valid, msg = validate_action(parsed)
        if not valid:
            print(f"Invalid: {msg}")
            return

        # Execute
        if parsed['action'] == 'pick':
            self.pick_object(parsed['params']['object'])
        # ... other actions
```

## Summary

### Key Takeaways
- **LLM integration** enables natural language robot control via APIs (GPT-4, Claude) or local models (LLaMA, Mistral)
- **Command parsing** converts "Pick up the red cube" → structured JSON `{action, object, modifiers}`
- **Task decomposition** breaks complex goals into sequences of atomic actions
- **Spatial reasoning** handles relative positions ("left of the mug")
- **Ollama** simplifies local LLM deployment (no API costs, works offline)
- **Validation layer** prevents LLM hallucinations from reaching hardware
- **Voice integration** combines speech recognition + LLM parsing for hands-free control

### What's Next
In Chapter 4.4, you'll train reinforcement learning policies in Isaac Sim for low-level control.

## Exercises

See [Module 4 Exercises](./exercises.md) - Exercise 4.2 covers LLM command parsing and Exercise 4.4 integrates vision-language-action pipeline.

## References

- OpenAI. (2023). GPT-4 Technical Report. *arXiv:2303.08774*. https://arxiv.org/abs/2303.08774
- Anthropic. (2024). Claude 3.5 model card. Retrieved from https://www.anthropic.com/claude
- Huang, W., et al. (2023). Language models as zero-shot planners. *CoRL 2022*. https://arxiv.org/abs/2201.07207

---

**Word Count**: ~850 words
