---
sidebar_position: 5
title: "Practical Exercises: Voice Commands"
description: "Hands-on exercises to implement voice command processing for robotic systems"
---

# Practical Exercises: Voice Commands

## Exercise 1: Basic Voice Command System

### Objective
Create a simple voice command system that can recognize and execute basic movement commands.

### Prerequisites
- OpenAI API key
- ROS 2 environment with a simulated robot
- Python development environment

### Steps

1. **Set up the environment**
   ```bash
   # Create a new directory for the exercise
   mkdir ~/vla_exercises/voice_commands
   cd ~/vla_exercises/voice_commands

   # Create virtual environment
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate

   # Install required packages
   pip install openai pyaudio wave rclpy
   ```

2. **Create the basic voice command processor**
   ```python
   # voice_command_processor.py
   import openai
   import pyaudio
   import wave
   import numpy as np
   import os
   import threading
   import time

   class SimpleVoiceCommandProcessor:
       def __init__(self):
           openai.api_key = os.getenv("OPENAI_API_KEY")

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

       def process_command(self):
           """Complete process: capture, transcribe, return command"""
           audio_file = self.capture_audio()
           command = self.transcribe_audio(audio_file)

           # Clean up temp file
           if os.path.exists(audio_file):
               os.remove(audio_file)

           return command

   # Example usage
   if __name__ == "__main__":
       processor = SimpleVoiceCommandProcessor()
       command = processor.process_command()
       print(f"Recognized command: {command}")
   ```

3. **Create intent classification**
   ```python
   # intent_classifier.py
   import re
   import openai
   import json
   import os

   class SimpleIntentClassifier:
       def __init__(self):
           self.command_patterns = {
               "move_forward": [r"move forward", r"go forward", r"forward"],
               "move_backward": [r"move backward", r"go backward", r"back"],
               "turn_left": [r"turn left", r"go left", r"left"],
               "turn_right": [r"turn right", r"go right", r"right"],
               "stop": [r"stop", r"halt", r"pause"]
           }

       def classify_rule_based(self, text):
           """Classify intent using rule-based patterns"""
           text_lower = text.lower()

           for intent, patterns in self.command_patterns.items():
               for pattern in patterns:
                   if re.search(pattern, text_lower):
                       return {
                           "intent": intent,
                           "confidence": 0.9,
                           "parameters": {}
                       }

           return {
               "intent": "unknown",
               "confidence": 0.0,
               "parameters": {}
           }

       def classify_llm(self, text):
           """Classify intent using LLM"""
           client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

           prompt = f"""
           Classify this robot command:
           "{text}"

           Available intents: move_forward, move_backward, turn_left, turn_right, stop, unknown

           Respond in JSON format:
           {{
               "intent": "...",
               "confidence": 0.0-1.0,
               "parameters": {{}}
           }}
           """

           try:
               response = client.chat.completions.create(
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
           except Exception as e:
               print(f"LLM classification error: {e}")

           return {"intent": "unknown", "confidence": 0.0, "parameters": {}}
   ```

4. **Create a simple ROS 2 action client**
   ```python
   # robot_action_client.py
   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node

   class SimpleRobotClient(Node):
       def __init__(self):
           super().__init__('simple_robot_client')
           # In a real implementation, you would connect to actual robot actions
           self.get_logger().info('Simple robot client initialized')

       def execute_command(self, intent_result):
           """Execute robot command based on intent"""
           intent = intent_result.get('intent', 'unknown')
           print(f"Executing command: {intent}")

           # In a real system, this would trigger actual robot actions
           if intent == 'move_forward':
               print("Robot moving forward...")
           elif intent == 'move_backward':
               print("Robot moving backward...")
           elif intent == 'turn_left':
               print("Robot turning left...")
           elif intent == 'turn_right':
               print("Robot turning right...")
           elif intent == 'stop':
               print("Robot stopping...")
           else:
               print("Unknown command")
   ```

5. **Combine all components**
   ```python
   # main_voice_system.py
   import os
   import sys
   from voice_command_processor import SimpleVoiceCommandProcessor
   from intent_classifier import SimpleIntentClassifier
   import rclpy

   def main():
       # Check for API key
       if not os.getenv("OPENAI_API_KEY"):
           print("Please set OPENAI_API_KEY environment variable")
           return

       # Initialize components
       processor = SimpleVoiceCommandProcessor()
       classifier = SimpleIntentClassifier()

       print("Voice command system ready. Say 'quit' to exit.")

       while True:
           try:
               print("\nWaiting for voice command...")
               command_text = processor.process_command()

               if not command_text:
                   print("Could not understand command, try again.")
                   continue

               print(f"Heard: {command_text}")

               if "quit" in command_text.lower():
                   print("Exiting...")
                   break

               # Classify intent
               intent_result = classifier.classify_rule_based(command_text)
               print(f"Classified intent: {intent_result}")

               # In a real system, you would execute the command here
               print("Command processed successfully!")

           except KeyboardInterrupt:
               print("\nExiting...")
               break
           except Exception as e:
               print(f"Error: {e}")
               continue

   if __name__ == "__main__":
       main()
   ```

6. **Run the exercise**
   ```bash
   # Set your OpenAI API key
   export OPENAI_API_KEY=your_api_key_here

   # Run the voice command system
   python main_voice_system.py
   ```

## Exercise 2: Advanced Voice Command with ROS 2 Integration

### Objective
Extend the basic system to integrate with a real ROS 2 robot simulation.

### Steps

1. **Launch a ROS 2 robot simulation**
   ```bash
   # Launch turtlebot3 simulation (or your preferred robot)
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

2. **Create a custom action for voice commands**
   ```bash
   # Create action file
   mkdir -p ~/voice_robot_ws/src/voice_robot_msgs/action
   ```

   ```yaml
   # ~/voice_robot_ws/src/voice_robot_msgs/action/VoiceCommand.action
   # Goal
   string command
   float64[] parameters

   # Result
   bool success
   string message

   # Feedback
   string status
   float64 progress
   ```

3. **Build the custom action**
   ```bash
   cd ~/voice_robot_ws
   colcon build --packages-select voice_robot_msgs
   source install/setup.bash
   ```

4. **Create the voice command action server**
   ```python
   # voice_command_server.py
   import rclpy
   from rclpy.action import ActionServer, CancelResponse
   from rclpy.node import Node
   from voice_robot_msgs.action import VoiceCommand
   import threading
   import time

   class VoiceCommandActionServer(Node):
       def __init__(self):
           super().__init__('voice_command_action_server')
           self._action_server = ActionServer(
               self,
               VoiceCommand,
               'voice_command',
               self.execute_callback,
               cancel_callback=self.cancel_callback)

           self.get_logger().info('Voice command action server started')

       def cancel_callback(self, goal_handle):
           """Accept or reject a client request to cancel an action"""
           self.get_logger().info('Received cancel request')
           return CancelResponse.ACCEPT

       def execute_callback(self, goal_handle):
           """Execute the voice command goal"""
           self.get_logger().info(f'Executing goal: {goal_handle.request.command}')

           feedback_msg = VoiceCommand.Feedback()
           feedback_msg.status = 'Processing command'
           feedback_msg.progress = 0.0

           command = goal_handle.request.command

           # Simulate command execution
           for i in range(10):
               if goal_handle.is_cancel_requested:
                   goal_handle.canceled()
                   self.get_logger().info('Goal canceled')
                   return VoiceCommand.Result(success=False, message='Goal canceled')

               # Update feedback
               feedback_msg.progress = float(i + 1) / 10.0
               feedback_msg.status = f'Executing: {command} ({int(feedback_msg.progress * 100)}%)'
               goal_handle.publish_feedback(feedback_msg)

               time.sleep(0.5)  # Simulate work

           # Complete successfully
           goal_handle.succeed()
           result = VoiceCommand.Result()
           result.success = True
           result.message = f'Command "{command}" executed successfully'
           self.get_logger().info(f'Command completed: {result.message}')

           return result

   def main(args=None):
       rclpy.init(args=args)
       action_server = VoiceCommandActionServer()

       try:
           rclpy.spin(action_server)
       except KeyboardInterrupt:
           pass
       finally:
           action_server.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

5. **Create an action client to send voice commands**
   ```python
   # voice_command_client.py
   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node
   from voice_robot_msgs.action import VoiceCommand

   class VoiceCommandActionClient(Node):
       def __init__(self):
           super().__init__('voice_command_action_client')
           self._action_client = ActionClient(
               self,
               VoiceCommand,
               'voice_command')

       def send_goal(self, command):
           goal_msg = VoiceCommand.Goal()
           goal_msg.command = command

           self._action_client.wait_for_server()
           self._send_goal_future = self._action_client.send_goal_async(
               goal_msg,
               feedback_callback=self.feedback_callback)

           self._send_goal_future.add_done_callback(self.goal_response_callback)

       def goal_response_callback(self, future):
           goal_handle = future.result()
           if not goal_handle.accepted:
               self.get_logger().info('Goal rejected :(')
               return

           self.get_logger().info('Goal accepted :)')
           self._get_result_future = goal_handle.get_result_async()
           self._get_result_future.add_done_callback(self.get_result_callback)

       def feedback_callback(self, feedback_msg):
           self.get_logger().info(
               f'Feedback: {feedback_msg.status} - {feedback_msg.progress:.2f}')

       def get_result_callback(self, future):
           result = future.result().result
           self.get_logger().info(f'Result: {result.message}')

   def main(args=None):
       rclpy.init(args=args)
       action_client = VoiceCommandActionClient()

       # Send a test command
       action_client.send_goal('move forward slowly')

       # Spin to process callbacks
       rclpy.spin(action_client)

   if __name__ == '__main__':
       main()
   ```

6. **Test the complete system**
   ```bash
   # Terminal 1: Start the action server
   ros2 run voice_robot voice_command_server.py

   # Terminal 2: Send test commands
   ros2 run voice_robot voice_command_client.py
   ```

## Exercise 3: Voice Command Validation and Safety

### Objective
Implement validation and safety checks for voice commands to prevent unsafe robot behavior.

### Steps

1. **Create a validation system**
   ```python
   # command_validator.py
   class CommandValidator:
       def __init__(self):
           self.safe_locations = ['kitchen', 'living room', 'bedroom', 'office']
           self.safe_distances = {'min': 0.1, 'max': 5.0}  # meters
           self.safe_angles = {'min': -180, 'max': 180}    # degrees

       def validate_navigation(self, location):
           """Validate navigation command"""
           if location not in self.safe_locations:
               return False, f"Unsafe location: {location}. Valid locations: {self.safe_locations}"
           return True, "Valid navigation command"

       def validate_movement(self, distance):
           """Validate movement command"""
           if distance < self.safe_distances['min'] or distance > self.safe_distances['max']:
               return False, f"Unsafe distance: {distance}m. Valid range: {self.safe_distances['min']}-{self.safe_distances['max']}m"
           return True, "Valid movement command"

       def validate_command(self, intent_result):
           """Validate the entire intent result"""
           intent = intent_result.get('intent', 'unknown')
           params = intent_result.get('parameters', {})

           if intent == 'navigate_to' and 'location' in params:
               return self.validate_navigation(params['location'])
           elif intent in ['move_forward', 'move_backward'] and 'distance' in params:
               return self.validate_movement(params['distance'])

           return True, "Command validated successfully"
   ```

2. **Integrate validation with your system**
   ```python
   # Add validation to your main system
   def process_voice_command_with_validation(self, text):
       # Classify intent
       intent_result = self.classifier.classify_rule_based(text)

       # Validate command
       validator = CommandValidator()
       is_valid, message = validator.validate_command(intent_result)

       if not is_valid:
           print(f"Command validation failed: {message}")
           return False

       # Execute if valid
       return self.execute_command(intent_result)
   ```

## Solutions and Hints

### Exercise 1 Solution Highlights
- Audio capture uses PyAudio to record until silence is detected
- Whisper API transcribes the audio to text
- Simple regex patterns match common commands
- The system provides feedback about recognized commands

### Exercise 2 Solution Highlights
- Custom ROS 2 action for voice commands
- Action server processes commands with feedback
- Action client sends commands from voice processing
- Integration with existing robot simulation

### Exercise 3 Solution Highlights
- Validation layer prevents unsafe commands
- Safe locations, distances, and angles defined
- Validation occurs before command execution
- Clear error messages for invalid commands

## Assessment Questions

1. What are the main components of a voice-to-action pipeline?
2. Why are ROS 2 actions preferred over services for voice command execution?
3. How would you handle ambiguous voice commands?
4. What safety considerations are important when executing voice commands on a robot?

## Extension Ideas

1. Add speech synthesis to provide verbal feedback to users
2. Implement voice command history and undo functionality
3. Add multi-language support for international users
4. Create a web interface for monitoring voice command status
5. Implement voice command macros or sequences

## Summary

These exercises provide hands-on experience with:
- Audio capture and processing
- Speech recognition with OpenAI Whisper
- Intent classification using both rule-based and LLM approaches
- ROS 2 action integration
- Safety validation for robot commands

Complete these exercises to gain practical experience with voice-to-action interfaces for robotics.