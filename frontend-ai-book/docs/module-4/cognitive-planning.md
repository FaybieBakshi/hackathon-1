---
sidebar_position: 3
title: "Cognitive Planning: LLMs for Command Decomposition to ROS 2 Action Sequences"
---

# Cognitive Planning: LLMs for Command Decomposition to ROS 2 Action Sequences

## Overview

This chapter introduces you to using Large Language Models (LLMs) to decompose complex natural language commands like "Clean the room" into sequences of executable ROS 2 actions that the humanoid robot can perform in the simulation. You'll learn how to implement prompt engineering, task decomposition, and ROS 2 sequence generation.

## Learning Objectives

By the end of this chapter, you will be able to:

1. Implement Large Language Model integration for natural language processing
2. Design prompt engineering techniques for robotics command interpretation
3. Create task decomposition algorithms that break complex commands into action sequences
4. Generate executable ROS 2 action sequences from natural language input
5. Validate and execute action sequences in simulation environments

## Prerequisites

- Completion of Module 1: The Robotic Nervous System (ROS 2)
- Completion of Module 2: The Digital Twin (Gazebo & Unity)
- Completion of Module 3: The AI-Robot Brain (NVIDIA Isaac)
- Completion of Chapter 1: Voice-to-Action processing
- Understanding of ROS 2 action servers and execution
- Basic Python programming knowledge for robotics

## Introduction to Cognitive Planning for Robotics

Cognitive planning bridges the gap between high-level natural language commands and low-level robot actions. The process involves:

1. **Natural Language Understanding**: Interpreting complex commands like "Clean the room"
2. **Task Decomposition**: Breaking commands into smaller, executable subtasks
3. **Action Sequencing**: Ordering actions logically to achieve the goal
4. **ROS 2 Action Generation**: Converting subtasks into executable ROS 2 action calls
5. **Execution Planning**: Validating action sequences for feasibility and safety

### Key Benefits of Cognitive Planning

1. **High-Level Commands**: Allows users to issue complex tasks in natural language
2. **Intelligent Decomposition**: Automatically breaks tasks into manageable steps
3. **Context Awareness**: Considers environment and robot capabilities
4. **Adaptive Planning**: Adjusts plans based on execution feedback

## Large Language Model Integration

### LLM Selection and Setup

For robotics cognitive planning, you'll typically use models like:

- **OpenAI GPT-4**: Excellent for reasoning and planning
- **Anthropic Claude**: Good for structured output generation
- **Open Source Models**: Like Llama 2 for local deployment

```python
import openai
import json
from typing import List, Dict, Any

class CognitivePlanningNode:
    def __init__(self):
        # Initialize LLM client
        self.client = openai.OpenAI(api_key="YOUR_API_KEY_HERE")

        # Define the robot's action capabilities
        self.robot_capabilities = {
            "navigation": ["move_to", "navigate_to", "go_to", "approach"],
            "manipulation": ["pick_up", "grasp", "lift", "place", "drop", "move_object"],
            "perception": ["detect", "find", "locate", "scan", "look_for"],
            "communication": ["speak", "say", "announce", "report"]
        }

        # Define common task templates
        self.task_templates = {
            "clean_room": [
                "detect objects_to_clean",
                "navigate_to object_location",
                "pick_up object",
                "navigate_to disposal_area",
                "place object in_bin"
            ],
            "prepare_drink": [
                "navigate_to kitchen",
                "detect ingredients",
                "pick_up container",
                "navigate_to preparation_area",
                "perform preparation_steps"
            ]
        }

    def decompose_command(self, command: str) -> List[Dict[str, Any]]:
        """
        Decompose a natural language command into a sequence of ROS 2 actions
        """
        # Create a structured prompt for the LLM
        prompt = self.create_planning_prompt(command)

        try:
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[
                    {
                        "role": "system",
                        "content": "You are a cognitive planning assistant for a humanoid robot. "
                                  "Decompose complex commands into sequences of executable actions. "
                                  "Each action should be specific, executable, and in the format: "
                                  "{'action': 'action_name', 'parameters': {'param_name': 'value'}}. "
                                  "Return only a JSON array of actions."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.3,
                max_tokens=1000
            )

            # Parse the response
            actions = json.loads(response.choices[0].message.content)
            return actions

        except Exception as e:
            print(f"Error decomposing command: {e}")
            return self.fallback_decomposition(command)

    def create_planning_prompt(self, command: str) -> str:
        """
        Create a structured prompt for cognitive planning
        """
        prompt = f"""
Decompose the following command into a sequence of executable actions for a humanoid robot:

Command: "{command}"

Consider the following capabilities of the robot:
- Navigation: move_to, navigate_to, go_to, approach
- Manipulation: pick_up, grasp, lift, place, drop, move_object
- Perception: detect, find, locate, scan, look_for
- Communication: speak, say, announce, report

Context: The robot operates in a simulated environment with known objects and locations.

Return a JSON array of actions in the format:
[
    {{
        "action": "action_name",
        "parameters": {{
            "param_name": "value",
            "target_object": "object_name",
            "location": "location_name"
        }}
    }}
]

Provide the most logical sequence of actions to complete the task.
"""
        return prompt

    def fallback_decomposition(self, command: str) -> List[Dict[str, Any]]:
        """
        Fallback method if LLM fails
        """
        # Simple keyword-based decomposition
        command_lower = command.lower()

        if "clean" in command_lower or "tidy" in command_lower:
            return [
                {"action": "detect", "parameters": {"target_object": "objects_to_clean"}},
                {"action": "navigate_to", "parameters": {"location": "first_object"}},
                {"action": "pick_up", "parameters": {"target_object": "object"}},
                {"action": "navigate_to", "parameters": {"location": "disposal_area"}},
                {"action": "place", "parameters": {"target_object": "object", "location": "bin"}}
            ]
        elif "move" in command_lower or "go" in command_lower:
            return [
                {"action": "navigate_to", "parameters": {"location": "target_destination"}}
            ]
        else:
            return [
                {"action": "speak", "parameters": {"text": f"I don't understand the command: {command}"}}
            ]
```

### Prompt Engineering for Robotics

Effective prompt engineering is crucial for reliable cognitive planning:

```python
class PromptEngineering:
    def __init__(self):
        self.system_prompt = """You are a cognitive planning assistant for a humanoid robot.
Your role is to decompose high-level natural language commands into sequences of
specific, executable ROS 2 actions. Each action must be feasible for the robot
and logically sequenced to achieve the user's goal."""

    def create_context_aware_prompt(self, command: str, environment_context: Dict[str, Any]) -> str:
        """
        Create a prompt that includes environmental context
        """
        context_str = json.dumps(environment_context, indent=2)

        prompt = f"""
{self.system_prompt}

Environmental Context:
{context_str}

User Command: "{command}"

Available Robot Actions:
- Navigation: move_to, navigate_to, go_to, approach (parameters: location, target)
- Manipulation: pick_up, grasp, lift, place, drop, move_object (parameters: object, location)
- Perception: detect, find, locate, scan, look_for (parameters: target_object, area)
- Communication: speak, say, announce, report (parameters: text, recipient)

Requirements:
1. Actions must be specific and executable
2. Consider environmental constraints
3. Maintain logical sequence
4. Include error handling where appropriate

Output Format:
Return ONLY a JSON array of action objects:
[
    {{
        "action": "action_name",
        "parameters": {{"param1": "value1", "param2": "value2"}},
        "description": "Brief explanation of the action"
    }}
]
"""
        return prompt

    def validate_action_sequence(self, actions: List[Dict[str, Any]], robot_capabilities: Dict[str, Any]) -> bool:
        """
        Validate that the action sequence is executable by the robot
        """
        valid_actions = set()
        for category, actions_list in robot_capabilities.items():
            valid_actions.update(actions_list)

        for action in actions:
            if action.get("action") not in valid_actions:
                return False
        return True
```

## Task Decomposition Techniques

### Hierarchical Task Decomposition

Complex tasks are broken down hierarchically:

```python
class TaskDecomposer:
    def __init__(self):
        self.decomposition_rules = {
            "clean_room": {
                "high_level": ["survey_area", "identify_objects", "plan_cleanup_sequence"],
                "mid_level": ["navigate_to_object", "pickup_object", "dispose_object"],
                "low_level": ["move_arm", "grip_object", "navigate", "release_object"]
            },
            "set_table": {
                "high_level": ["identify_table", "locate_items", "plan_placement"],
                "mid_level": ["pickup_item", "navigate_to_table", "place_item"],
                "low_level": ["approach_item", "grasp", "transport", "position"]
            }
        }

    def decompose_task(self, task: str, depth: str = "mid_level") -> List[Dict[str, Any]]:
        """
        Decompose a task to specified depth level
        """
        if task in self.decomposition_rules:
            if depth in self.decomposition_rules[task]:
                subtasks = self.decomposition_rules[task][depth]
                return [{"action": action, "parameters": {}} for action in subtasks]

        # Fallback: use LLM for unknown tasks
        return self.llm_decompose_task(task)

    def llm_decompose_task(self, task: str) -> List[Dict[str, Any]]:
        """
        Use LLM to decompose unfamiliar tasks
        """
        # Implementation would call the LLM as shown in previous examples
        pass
```

### Sequential vs Parallel Task Planning

Consider which tasks can be executed in parallel:

```python
class TaskScheduler:
    def __init__(self):
        self.dependency_graph = {
            "navigate_to_object": ["detect_object"],
            "pickup_object": ["navigate_to_object"],
            "dispose_object": ["pickup_object", "navigate_to_disposal"],
            "navigate_to_disposal": ["detect_disposal_location"]
        }

    def create_execution_plan(self, action_sequence: List[Dict[str, Any]]) -> List[List[Dict[str, Any]]]:
        """
        Create an execution plan with parallelizable actions grouped together
        """
        execution_plan = []
        remaining_actions = action_sequence.copy()

        while remaining_actions:
            ready_actions = []

            for action in remaining_actions[:]:  # Use slice to avoid modification during iteration
                dependencies = self.dependency_graph.get(action["action"], [])
                all_deps_met = all(self.is_dependency_met(dep) for dep in dependencies)

                if all_deps_met:
                    ready_actions.append(action)
                    remaining_actions.remove(action)

            if ready_actions:
                execution_plan.append(ready_actions)
            else:
                # No progress possible - there might be circular dependencies
                print("Warning: Circular dependencies detected in action sequence")
                break

        return execution_plan

    def is_dependency_met(self, dependency: str) -> bool:
        """
        Check if a dependency has been met (simplified)
        """
        # In a real implementation, this would check execution state
        return True  # Simplified for example
```

## ROS 2 Sequence Generation

### Action Sequence Validation

Validate action sequences before execution:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

class ActionSequenceValidator(Node):
    def __init__(self):
        super().__init__('action_sequence_validator')

        # Publishers for different action types
        self.nav_publisher = self.create_publisher(String, '/navigation_commands', 10)
        self.manip_publisher = self.create_publisher(String, '/manipulation_commands', 10)
        self.percept_publisher = self.create_publisher(String, '/perception_commands', 10)

    def validate_sequence(self, action_sequence: List[Dict[str, Any]]) -> bool:
        """
        Validate an action sequence for feasibility
        """
        for action in action_sequence:
            action_type = action["action"]
            parameters = action.get("parameters", {})

            if not self.validate_action(action_type, parameters):
                return False

        return True

    def validate_action(self, action_type: str, parameters: Dict[str, Any]) -> bool:
        """
        Validate individual action
        """
        # Check if action type is supported
        supported_actions = [
            "navigate_to", "move_to", "go_to", "approach",
            "pick_up", "grasp", "lift", "place", "drop", "move_object",
            "detect", "find", "locate", "scan", "look_for",
            "speak", "say", "announce", "report"
        ]

        if action_type not in supported_actions:
            self.get_logger().warn(f"Unsupported action: {action_type}")
            return False

        # Validate parameters based on action type
        required_params = self.get_required_parameters(action_type)
        for param in required_params:
            if param not in parameters:
                self.get_logger().warn(f"Missing required parameter '{param}' for action '{action_type}'")
                return False

        return True

    def get_required_parameters(self, action_type: str) -> List[str]:
        """
        Get required parameters for an action type
        """
        param_map = {
            "navigate_to": ["location"],
            "move_to": ["location"],
            "go_to": ["location"],
            "approach": ["target"],
            "pick_up": ["target_object"],
            "grasp": ["target_object"],
            "place": ["target_object", "location"],
            "detect": ["target_object"],
            "speak": ["text"]
        }
        return param_map.get(action_type, [])
```

### Sequence Execution Engine

Execute validated action sequences:

```python
class SequenceExecutionEngine:
    def __init__(self):
        self.current_sequence = []
        self.current_step = 0
        self.execution_status = "idle"

    def execute_sequence(self, action_sequence: List[Dict[str, Any]]) -> bool:
        """
        Execute a sequence of actions
        """
        if not self.validate_sequence(action_sequence):
            print("Action sequence validation failed")
            return False

        self.current_sequence = action_sequence
        self.current_step = 0
        self.execution_status = "executing"

        success = True
        for i, action in enumerate(action_sequence):
            print(f"Executing action {i+1}/{len(action_sequence)}: {action['action']}")

            action_success = self.execute_single_action(action)
            if not action_success:
                print(f"Action failed: {action['action']}")
                success = False
                break

            self.current_step = i + 1

        self.execution_status = "completed" if success else "failed"
        return success

    def execute_single_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute a single action (placeholder implementation)
        """
        action_type = action["action"]
        parameters = action.get("parameters", {})

        # In a real implementation, this would call the appropriate ROS service/action
        print(f"Executing {action_type} with parameters: {parameters}")

        # Simulate action execution
        import time
        time.sleep(0.5)  # Simulate execution time

        # Return success/failure based on action execution
        return True  # Simplified for example
```

## Practical Exercises

### Exercise 1: Basic Command Decomposition

1. Set up LLM API access (OpenAI, Anthropic, etc.)
2. Create a simple command decomposition function
3. Test with commands like "Clean the room" and "Set the table"
4. Verify that the LLM generates appropriate action sequences

### Exercise 2: ROS 2 Action Sequence Generation

1. Implement the action sequence validation system
2. Create ROS action clients for different robot capabilities
3. Test the "natural language → executable action sequence" pipeline
4. Validate that sequences execute correctly in simulation

### Exercise 3: Advanced Task Planning

1. Implement hierarchical task decomposition
2. Add environmental context awareness to prompts
3. Test complex multi-step commands
4. Implement error handling and recovery behaviors

## Performance Optimization

### LLM Response Optimization

- **Caching**: Cache responses for common commands to reduce API calls
- **Prompt Templates**: Use optimized prompt templates for consistent responses
- **Response Parsing**: Implement robust JSON parsing for LLM outputs
- **Error Handling**: Graceful fallback when LLM fails

### Task Planning Efficiency

- **Parallel Execution**: Identify actions that can run in parallel
- **Dependency Resolution**: Optimize action sequencing for efficiency
- **Validation**: Validate sequences before execution to avoid failures
- **Monitoring**: Track execution progress and adjust plans as needed

## Troubleshooting Common Issues

### LLM Integration Issues

- **API Key Problems**: Verify LLM API keys are correctly configured
- **Rate Limits**: Implement proper rate limiting and retry logic
- **Response Format**: Ensure LLM responses match expected JSON format
- **Context Limits**: Manage token usage for complex planning tasks

### Action Sequence Issues

- **Invalid Actions**: Verify all actions are supported by the robot
- **Missing Parameters**: Ensure all required parameters are provided
- **Execution Failures**: Implement proper error handling and recovery
- **Sequence Validation**: Validate sequences before execution

## Summary

In this chapter, you've learned how to implement cognitive planning using Large Language Models to decompose complex natural language commands into executable ROS 2 action sequences. You've explored prompt engineering techniques, task decomposition methods, and sequence validation approaches.

The cognitive planning system enables robots to understand high-level commands and automatically generate appropriate action sequences, bridging the gap between natural language and robot execution.

## Next Steps

In the next chapter, you'll integrate voice processing and cognitive planning to create a complete Vision-Language-Action system that demonstrates full autonomous humanoid behavior in simulation environments.