---
sidebar_position: 1
title: "Complete VLA Workflow Tutorial"
description: "Step-by-step guide to implementing the complete Vision-Language-Action pipeline"
---

# Complete VLA Workflow Tutorial

## Overview

This tutorial provides a comprehensive step-by-step guide to implementing the complete Vision-Language-Action (VLA) pipeline. You'll learn how to integrate voice interfaces, cognitive planning with LLMs, and autonomous execution in a unified robotic system.

## Prerequisites

Before starting this tutorial, you should have:
- Completed Modules 1-3 of this course
- Basic understanding of ROS 2 concepts
- Access to OpenAI API (for Whisper and GPT models)
- A simulation environment (Gazebo, Isaac Sim, or similar)
- Python development environment with ROS 2

## Complete VLA System Architecture

The complete VLA system consists of three main components working together:

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│   Voice Input   │───▶│ Cognitive        │───▶│ Autonomous       │
│   (Chapter 1)   │    │ Planning (Ch 2)  │    │ Execution (Ch 3) │
│                 │    │                  │    │                  │
│ • Speech        │    │ • LLM-based      │    │ • Task execution │
│   recognition   │    │   planning       │    │ • Behavior trees │
│ • Intent        │    │ • Task           │    │ • Monitoring     │
│   classification│    │   decomposition  │    │ • Safety systems │
└─────────────────┘    └──────────────────┘    └──────────────────┘
```

## Step 1: Setting Up the Voice Interface (Chapter 1)

### 1.1 Install Required Dependencies

```bash
# Create a new workspace for the VLA system
mkdir -p ~/vla_ws/src
cd ~/vla_ws

# Create virtual environment
python -m venv vla_env
source vla_env/bin/activate

# Install required packages
pip install openai pyaudio wave numpy rclpy
```

### 1.2 Create the Voice Command Processor

Create `voice_processor.py`:

```python
import openai
import pyaudio
import wave
import numpy as np
import os
import threading
import time
import rclpy
from std_msgs.msg import String

class VoiceCommandProcessor:
    def __init__(self, api_key):
        openai.api_key = api_key

        # Audio configuration
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.silence_threshold = 500
        self.max_silence = 30

    def capture_audio(self):
        """Capture audio from microphone until silence is detected"""
        p = pyaudio.PyAudio()

        stream = p.open(format=self.format,
                       channels=self.channels,
                       rate=self.rate,
                       input=True,
                       frames_per_buffer=self.chunk)

        print("Listening... Speak your command now.")
        frames = []
        silence_count = 0

        while True:
            data = stream.read(self.chunk)
            frames.append(data)

            # Check for silence to stop recording
            audio_data = np.frombuffer(data, dtype=np.int16)
            if np.abs(audio_data).mean() < self.silence_threshold:
                silence_count += 1
                if silence_count > self.max_silence:
                    break
            else:
                silence_count = 0

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save to temporary file
        temp_file = "temp_command.wav"
        wf = wave.open(temp_file, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return temp_file

    def transcribe_audio(self, audio_file):
        """Transcribe audio using OpenAI Whisper API"""
        try:
            with open(audio_file, "rb") as audio:
                transcript = openai.Audio.transcribe("whisper-1", audio)
            return transcript.text
        except Exception as e:
            print(f"Transcription error: {e}")
            return None

    def process_voice_command(self):
        """Complete process: capture, transcribe, return command"""
        audio_file = self.capture_audio()
        command = self.transcribe_audio(audio_file)

        # Clean up temp file
        if os.path.exists(audio_file):
            os.remove(audio_file)

        return command
```

### 1.3 Test Voice Command Processing

Create `test_voice.py`:

```python
import os
from voice_processor import VoiceCommandProcessor

def main():
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Please set OPENAI_API_KEY environment variable")
        return

    processor = VoiceCommandProcessor(api_key)

    print("Voice command processor ready. Say a command...")
    command = processor.process_voice_command()

    if command:
        print(f"Recognized command: {command}")
    else:
        print("Could not understand command")

if __name__ == "__main__":
    main()
```

## Step 2: Implementing Cognitive Planning (Chapter 2)

### 2.1 Create the Intent Classifier

Create `intent_classifier.py`:

```python
import openai
import json
import os

class LLMIntentClassifier:
    def __init__(self, api_key):
        self.client = openai.OpenAI(api_key=api_key)

    def classify_intent(self, text):
        """
        Use LLM to classify intent and extract parameters
        """
        prompt = f"""
        You are a robot command interpreter. Analyze the following user command and extract:
        1. The primary intent (one of: navigate_to, pick_up, place, find, report, wait, unknown)
        2. Any relevant parameters (like location, object name, duration, etc.)

        Command: "{text}"

        Respond in JSON format:
        {{
            "success": true,
            "structured_goal": {{
                "intent": "...",
                "object_type": "...",
                "location": "...",
                "duration": "...",
                "parameters": {{}}
            }},
            "confidence": 0.0-1.0
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            result_text = response.choices[0].message.content.strip()
            json_start = result_text.find('{')
            json_end = result_text.rfind('}') + 1

            if json_start != -1 and json_end != 0:
                json_str = result_text[json_start:json_end]
                return json.loads(json_str)
            else:
                return {
                    "success": False,
                    "error": "Could not parse LLM response",
                    "structured_goal": {}
                }

        except Exception as e:
            print(f"LLM classification error: {e}")
            return {
                "success": False,
                "error": str(e),
                "structured_goal": {}
            }
```

### 2.2 Create the Task Planner

Create `task_planner.py`:

```python
import openai
import json

class LLMBasedPlanner:
    def __init__(self, api_key):
        self.client = openai.OpenAI(api_key=api_key)

    def generate_plan(self, goal):
        """
        Generate a detailed plan from a structured goal
        """
        prompt = f"""
        You are a robotic task planner. Generate a detailed execution plan for the following goal:

        Goal: {json.dumps(goal, indent=2)}

        Consider:
        1. Navigation requirements
        2. Object detection needs
        3. Manipulation requirements
        4. Safety constraints
        5. Environmental conditions

        Generate a plan with specific, executable steps.

        Respond in JSON format:
        {{
            "success": true,
            "plan": [
                {{
                    "step_id": "...",
                    "description": "...",
                    "action": "...",
                    "parameters": {{
                        "location": [x, y, theta],
                        "object_type": "...",
                        "gripper_width": 0.0
                    }},
                    "expected_outcomes": ["..."]
                }}
            ],
            "estimated_duration": 0.0
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.2
            )

            result_text = response.choices[0].message.content.strip()
            json_start = result_text.find('{')
            json_end = result_text.rfind('}') + 1

            if json_start != -1 and json_end != 0:
                json_str = result_text[json_start:json_end]
                return json.loads(json_str)
            else:
                return {
                    "success": False,
                    "error": "Could not parse plan generation response"
                }

        except Exception as e:
            print(f"Plan generation error: {e}")
            return {
                "success": False,
                "error": str(e)
            }
```

## Step 3: Autonomous Execution System (Chapter 3)

### 3.1 Create the Behavior Executor

Create `behavior_executor.py`:

```python
import time
import random

class BehaviorExecutor:
    def __init__(self):
        self.robot_state = {
            "position": [0.0, 0.0, 0.0],
            "holding_object": None,
            "battery_level": 100.0
        }

    def execute_plan(self, plan):
        """
        Execute a sequence of planned behaviors
        """
        results = []

        for step in plan.get("plan", []):
            print(f"Executing: {step['description']}")
            result = self.execute_step(step)
            results.append({
                "step": step,
                "result": result,
                "timestamp": time.time()
            })

            if not result["success"]:
                print(f"Step failed: {result.get('error', 'Unknown error')}")
                return {
                    "success": False,
                    "results": results,
                    "error_step": step
                }

        return {
            "success": True,
            "results": results
        }

    def execute_step(self, step):
        """
        Execute a single step
        """
        action = step.get("action", "").lower()
        params = step.get("parameters", {})

        if action == "navigate_to":
            return self.execute_navigation(params)
        elif action == "pick_up":
            return self.execute_pickup(params)
        elif action == "place":
            return self.execute_placement(params)
        elif action == "detect_object":
            return self.execute_detection(params)
        else:
            return {
                "success": False,
                "error": f"Unknown action: {action}"
            }

    def execute_navigation(self, params):
        """Execute navigation action"""
        target = params.get("location", [0, 0, 0])
        print(f"  Navigating to {target}")

        # Simulate navigation time
        time.sleep(2.0)

        # Simulate occasional failures
        if random.random() < 0.1:  # 10% failure rate
            return {
                "success": False,
                "error": "Navigation failed - path blocked"
            }

        # Update robot state
        self.robot_state["position"] = target
        return {
            "success": True,
            "message": f"Navigated to {target}",
            "final_position": target
        }

    def execute_pickup(self, params):
        """Execute pickup action"""
        obj_type = params.get("object_type", "unknown")
        print(f"  Picking up {obj_type}")

        time.sleep(1.5)

        if random.random() < 0.15:  # 15% failure rate
            return {
                "success": False,
                "error": f"Failed to pick up {obj_type}"
            }

        self.robot_state["holding_object"] = obj_type
        return {
            "success": True,
            "message": f"Successfully picked up {obj_type}",
            "object_held": obj_type
        }

    def execute_placement(self, params):
        """Execute placement action"""
        location = params.get("location", "unknown")
        obj = self.robot_state["holding_object"]

        if not obj:
            return {
                "success": False,
                "error": "Not holding any object to place"
            }

        print(f"  Placing {obj} at {location}")
        time.sleep(1.0)

        if random.random() < 0.05:  # 5% failure rate
            return {
                "success": False,
                "error": f"Failed to place {obj} - dropped"
            }

        self.robot_state["holding_object"] = None
        return {
            "success": True,
            "message": f"Successfully placed {obj} at {location}"
        }

    def execute_detection(self, params):
        """Execute object detection"""
        obj_type = params.get("object_type", "unknown")
        print(f"  Detecting {obj_type}")

        time.sleep(0.8)

        if random.random() < 0.2:  # 20% failure rate
            return {
                "success": False,
                "error": f"Could not find {obj_type}"
            }

        return {
            "success": True,
            "message": f"Detected {obj_type}",
            "object_found": True,
            "object_info": {"type": obj_type, "location": [1, 1, 1]}
        }
```

## Step 4: Complete VLA Orchestrator

### 4.1 Create the Main Orchestrator

Create `vla_orchestrator.py`:

```python
import os
import time
from voice_processor import VoiceCommandProcessor
from intent_classifier import LLMIntentClassifier
from task_planner import LLMBasedPlanner
from behavior_executor import BehaviorExecutor

class VLAOrchestrator:
    def __init__(self):
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")

        self.voice_processor = VoiceCommandProcessor(api_key)
        self.intent_classifier = LLMIntentClassifier(api_key)
        self.task_planner = LLMBasedPlanner(api_key)
        self.behavior_executor = BehaviorExecutor()

    def process_voice_command(self):
        """
        Complete VLA pipeline: Voice -> Intent -> Plan -> Execute
        """
        print("Starting VLA pipeline...")

        # Step 1: Capture and transcribe voice command
        print("1. Capturing voice command...")
        command_text = self.voice_processor.process_voice_command()

        if not command_text:
            return {
                "success": False,
                "error": "Could not capture voice command"
            }

        print(f"   Command captured: {command_text}")

        # Step 2: Classify intent
        print("2. Classifying intent...")
        intent_result = self.intent_classifier.classify_intent(command_text)

        if not intent_result["success"]:
            return {
                "success": False,
                "error": f"Intent classification failed: {intent_result.get('error')}"
            }

        print(f"   Intent classified: {intent_result['structured_goal']['intent']}")

        # Step 3: Generate task plan
        print("3. Generating task plan...")
        plan = self.task_planner.generate_plan(intent_result["structured_goal"])

        if not plan["success"]:
            return {
                "success": False,
                "error": f"Plan generation failed: {plan.get('error')}"
            }

        print(f"   Plan generated with {len(plan.get('plan', []))} steps")

        # Step 4: Execute the plan
        print("4. Executing plan...")
        execution_result = self.behavior_executor.execute_plan(plan)

        return {
            "success": execution_result["success"],
            "command_text": command_text,
            "intent_result": intent_result,
            "plan": plan,
            "execution_result": execution_result
        }

def main():
    """
    Main function to run the complete VLA system
    """
    try:
        orchestrator = VLAOrchestrator()

        print("Complete VLA System Ready!")
        print("Say a command for the robot to execute...")
        print("Example commands: 'Go to the kitchen', 'Find the red cup', 'Pick up the object'")

        result = orchestrator.process_voice_command()

        if result["success"]:
            print("\n✅ SUCCESS: Task completed!")
            print(f"Final robot state: {orchestrator.behavior_executor.robot_state}")
        else:
            print(f"\n❌ FAILED: {result.get('error', 'Unknown error')}")

    except ValueError as e:
        print(f"Configuration error: {e}")
        print("Make sure OPENAI_API_KEY environment variable is set")
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
```

## Step 5: Running the Complete System

### 5.1 Set Up Environment Variables

```bash
# Set your OpenAI API key
export OPENAI_API_KEY=your_openai_api_key_here

# Activate the virtual environment
source ~/vla_ws/vla_env/bin/activate
```

### 5.2 Run the Complete System

```bash
# Run the complete VLA system
python vla_orchestrator.py
```

## Step 6: Testing Different Scenarios

### 6.1 Simple Navigation

Try saying: "Go to the kitchen"

The system will:
1. Capture and transcribe your voice command
2. Classify the intent as navigation
3. Generate a plan to navigate to the kitchen
4. Execute the navigation plan

### 6.2 Object Manipulation

Try saying: "Find the red cup and pick it up"

The system will:
1. Process the complex command
2. Classify intent as object detection and manipulation
3. Generate a plan with detection and pickup steps
4. Execute the plan in sequence

### 6.3 Complex Multi-Step Task

Try saying: "Go to the kitchen, find the red cup, pick it up, and bring it to me"

The system will:
1. Break down the complex command
2. Generate a multi-step plan
3. Execute each step sequentially
4. Handle any failures with recovery

## Troubleshooting Common Issues

### Audio Capture Issues
- Ensure your microphone is working and has proper permissions
- Check that pyaudio is properly installed
- Verify audio format compatibility

### API Connection Issues
- Verify your OpenAI API key is correct
- Check your internet connection
- Ensure you have sufficient API quota

### Planning Failures
- The LLM might not understand complex commands
- Try simpler, more specific commands
- Ensure proper context is provided

### Execution Failures
- Simulated failures are included to test robustness
- The system includes recovery mechanisms
- Monitor the robot state for debugging

## Performance Optimization Tips

1. **Caching**: Cache frequently used plans for common commands
2. **Parallel Processing**: Process multiple aspects of commands in parallel
3. **Timeout Management**: Set appropriate timeouts for each component
4. **Error Recovery**: Implement robust error handling and recovery
5. **Resource Management**: Monitor and optimize resource usage

## Next Steps

After completing this tutorial, you should:

1. **Extend the System**: Add new capabilities like gesture recognition
2. **Improve Robustness**: Enhance error handling and recovery
3. **Add Learning**: Implement learning from execution results
4. **Deploy to Hardware**: Test on physical robots
5. **Expand Vocabulary**: Train on more diverse commands

## Summary

This tutorial has walked you through implementing a complete Vision-Language-Action system:

- **Vision**: Through object detection and environment perception
- **Language**: Using LLMs for intent classification and task planning
- **Action**: Executing complex robotic behaviors in simulation

You now have a foundation for building sophisticated autonomous humanoid systems that can understand natural language commands and execute them in real-world environments.