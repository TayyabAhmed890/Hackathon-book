---
sidebar_position: 4
title: "ROS 2 Action Triggering"
description: "Triggering ROS 2 actions from voice commands and intent classification"
---

# ROS 2 Action Triggering

## Introduction to ROS 2 Actions

ROS 2 Actions are a communication pattern that allows for long-running tasks with feedback and status updates. Unlike simple services that provide immediate responses, actions are ideal for robotic behaviors that take time to complete, such as navigation, manipulation, or complex movements. In the context of voice-to-action interfaces, ROS 2 actions provide the perfect mechanism to execute commands that may take seconds or minutes to complete.

## Why Use Actions for Voice Commands?

Actions are particularly well-suited for voice command execution because they:

- **Provide Feedback**: Users can receive status updates during long-running operations
- **Support Preemption**: Allow users to cancel or interrupt ongoing actions
- **Handle Failures**: Provide detailed error information when operations fail
- **Enable Progress Tracking**: Give users visibility into the execution progress

## ROS 2 Action Structure

A typical ROS 2 action consists of three message types:

1. **Goal**: Defines the desired action and its parameters
2. **Feedback**: Provides intermediate status during execution
3. **Result**: Contains the final outcome of the action

```python
# Example action definition (typically in action files like Move.action)
# Goal:
float64 target_x
float64 target_y
float64 target_theta

# Feedback:
float64 current_x
float64 current_y
float64 current_theta
string status

# Result:
bool success
string message
```

## Creating Action Clients for Voice Commands

To trigger ROS 2 actions from voice commands, you need to create action clients:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import your action message (example: navigation)
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class VoiceActionClient(Node):
    def __init__(self):
        super().__init__('voice_action_client')

        # Create action clients for different types of actions
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # You might also have manipulation actions
        # self.manip_client = ActionClient(
        #     self,
        #     ManipulationAction,
        #     'manipulation_action'
        # )

        # Publisher for status updates
        self.status_pub = self.create_publisher(String, 'voice_command_status', 10)

    def send_navigation_goal(self, x, y, theta):
        """
        Send a navigation goal to the robot based on voice command
        """
        # Wait for the action server to be available
        self.nav_client.wait_for_server()

        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta  # Simplified orientation

        # Send the goal asynchronously
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

    def goal_response_callback(self, future):
        """
        Handle the response from the action server
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.publish_status('navigation_goal_rejected')
            return

        self.get_logger().info('Goal accepted :)')
        self.publish_status('navigation_goal_accepted')

        # Request result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the action server
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current position: x={feedback.current_pose.pose.position.x}, y={feedback.current_pose.pose.position.y}')
        self.publish_status(f'navigating_to_destination,progress:{feedback.distance_remaining}')

    def get_result_callback(self, future):
        """
        Handle the final result from the action server
        """
        result = future.result().result
        status = self.nav_client.get_result()

        if status.result.success:
            self.get_logger().info('Navigation completed successfully!')
            self.publish_status('navigation_completed_successfully')
        else:
            self.get_logger().info('Navigation failed!')
            self.publish_status('navigation_failed')

    def publish_status(self, message):
        """
        Publish status updates for voice command system
        """
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
```

## Mapping Intents to Actions

Connect your intent classification system with ROS 2 action clients:

```python
class VoiceToActionMapper:
    def __init__(self, action_client):
        self.action_client = action_client
        self.action_mapping = {
            'navigate_to': self.execute_navigation,
            'move_forward': self.execute_move_forward,
            'turn_left': self.execute_turn,
            'turn_right': self.execute_turn,
            'pick_up': self.execute_manipulation,
            'place': self.execute_manipulation
        }

    def execute_voice_command(self, intent_result):
        """
        Execute the appropriate ROS 2 action based on classified intent
        """
        intent = intent_result['intent']
        params = intent_result['parameters']

        if intent in self.action_mapping:
            try:
                return self.action_mapping[intent](params)
            except Exception as e:
                self.action_client.get_logger().error(f'Error executing {intent}: {e}')
                return False
        else:
            self.action_client.get_logger().error(f'Unknown intent: {intent}')
            return False

    def execute_navigation(self, params):
        """
        Execute navigation action based on parameters
        """
        location = params.get('location', 'unknown')

        # Map location names to coordinates (this would come from a map)
        location_map = {
            'kitchen': (2.0, 1.0, 0.0),
            'bedroom': (5.0, 3.0, 1.57),
            'living_room': (0.0, 0.0, 0.0),
            'office': (-2.0, 4.0, -1.57)
        }

        if location in location_map:
            x, y, theta = location_map[location]
            self.action_client.get_logger().info(f'Navigating to {location} at ({x}, {y}, {theta})')
            return self.action_client.send_navigation_goal(x, y, theta)
        else:
            self.action_client.get_logger().error(f'Unknown location: {location}')
            return False

    def execute_move_forward(self, params):
        """
        Execute forward movement action
        """
        distance = float(params.get('distance', 1.0))  # Default 1 meter
        self.action_client.get_logger().info(f'Moving forward {distance} meters')

        # This would use a custom action for simple movement
        # Implementation depends on your robot's capabilities
        pass

    def execute_turn(self, params):
        """
        Execute turning action
        """
        intent = params.get('intent', 'turn_left')
        angle = float(params.get('angle', 90))  # Default 90 degrees

        if intent == 'turn_right':
            angle = -angle  # Negative for right turn

        self.action_client.get_logger().info(f'Turning {angle} degrees')
        # Implementation would use appropriate turning action
        pass

    def execute_manipulation(self, params):
        """
        Execute manipulation action (pick up, place, etc.)
        """
        intent = params.get('intent', 'pick_up')
        obj = params.get('object', 'unknown')

        self.action_client.get_logger().info(f'{intent} object: {obj}')
        # Implementation would use manipulation actions
        pass
```

## Handling Action Feedback and Status

Provide meaningful feedback to users during action execution:

```python
class ActionFeedbackHandler:
    def __init__(self, speech_publisher):
        self.speech_publisher = speech_publisher
        self.last_feedback_time = 0

    def handle_navigation_feedback(self, feedback_msg):
        """
        Process navigation feedback and provide user updates
        """
        current_time = time.time()

        # Avoid spamming feedback - only update every 5 seconds
        if current_time - self.last_feedback_time < 5:
            return

        self.last_feedback_time = current_time

        feedback = feedback_msg.feedback
        distance_remaining = getattr(feedback, 'distance_remaining', None)

        if distance_remaining:
            message = f"Still {distance_remaining:.1f} meters to go"
            self.speech_publisher.speak(message)

    def handle_manipulation_feedback(self, feedback_msg):
        """
        Process manipulation feedback
        """
        feedback = feedback_msg.feedback
        status = getattr(feedback, 'status', 'unknown')

        if status == 'approaching_object':
            self.speak("Approaching the object")
        elif status == 'grasping':
            self.speak("Grasping the object")
        elif status == 'lifting':
            self.speak("Lifting the object")
        # Add more status messages as needed
```

## Error Handling and Recovery

Implement robust error handling for action execution:

```python
class ActionErrorHandler:
    def __init__(self, action_client):
        self.action_client = action_client
        self.recovery_attempts = {}

    def handle_action_failure(self, intent, result, error_code=None):
        """
        Handle action failures and attempt recovery
        """
        failure_key = f"{intent}_{error_code}" if error_code else intent

        if failure_key not in self.recovery_attempts:
            self.recovery_attempts[failure_key] = 0

        self.recovery_attempts[failure_key] += 1
        attempts = self.recovery_attempts[failure_key]

        if attempts <= 3:  # Try up to 3 times
            self.action_client.get_logger().info(f'Attempting recovery for {intent}, attempt {attempts}')
            return self.attempt_recovery(intent, result)
        else:
            self.action_client.get_logger().error(f'Max recovery attempts reached for {intent}')
            self.speak_error_message(intent, result)
            return False

    def attempt_recovery(self, intent, result):
        """
        Attempt to recover from action failure
        """
        if intent == 'navigate_to':
            # For navigation failures, try an alternative route or closer point
            return self.recovery_navigation(intent, result)
        elif intent == 'pick_up':
            # For manipulation failures, try different grasp approach
            return self.recovery_manipulation(intent, result)
        else:
            # For other failures, ask user for alternative
            self.ask_user_alternative(intent, result)
            return False

    def recovery_navigation(self, intent, result):
        """
        Recovery strategy for navigation failures
        """
        # This might involve:
        # - Trying a different navigation planner
        # - Using a safer path
        # - Moving to a nearby position instead of exact target
        pass

    def speak_error_message(self, intent, result):
        """
        Provide verbal feedback about the failure
        """
        error_messages = {
            'navigate_to': "I couldn't reach that location. The path might be blocked.",
            'pick_up': "I couldn't pick up the object. It might be out of reach or too heavy.",
            'move_forward': "I couldn't move forward. There might be an obstacle in the way."
        }

        message = error_messages.get(intent, f"The {intent} action failed. Please try again.")
        # Publish to speech system
        pass
```

## Integration Example: Complete Voice Command System

Here's how all components work together:

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import queue

class VoiceCommandSystem:
    def __init__(self):
        rclpy.init()
        self.node = VoiceActionClient()

        # Initialize components
        self.intent_classifier = LLMIntentClassifier(api_key=os.getenv("OPENAI_API_KEY"))
        self.intent_validator = IntentValidator()
        self.action_mapper = VoiceToActionMapper(self.node)
        self.feedback_handler = ActionFeedbackHandler(None)  # Speech publisher
        self.error_handler = ActionErrorHandler(self.node)

        # Command queue for processing
        self.command_queue = queue.Queue()
        self.running = True

    def process_voice_command(self, text):
        """
        Complete pipeline: text -> intent -> action -> execution
        """
        try:
            # 1. Classify intent
            intent_result = self.intent_classifier.classify_intent(text)

            # 2. Validate intent
            validation_result = self.intent_validator.validate_intent(intent_result)

            if not validation_result["valid"]:
                # Handle validation failure
                self.node.get_logger().warn(f'Invalid intent: {validation_result["reason"]}')
                return False

            # 3. Execute action
            execution_result = self.action_mapper.execute_voice_command(intent_result)

            return execution_result

        except Exception as e:
            self.node.get_logger().error(f'Error processing voice command: {e}')
            return False

    def start_listening(self):
        """
        Start the voice command processing loop
        """
        while self.running:
            try:
                # Wait for commands in the queue
                command_text = self.command_queue.get(timeout=1.0)

                if command_text:
                    self.process_voice_command(command_text)

            except queue.Empty:
                continue  # Keep listening
            except KeyboardInterrupt:
                self.running = False
                break

    def shutdown(self):
        """
        Clean shutdown of the system
        """
        self.running = False
        rclpy.shutdown()

# Example usage
def main():
    system = VoiceCommandSystem()

    # Simulate receiving voice commands
    commands = [
        "Go to the kitchen",
        "Turn left",
        "Move forward 2 meters"
    ]

    for cmd in commands:
        system.process_voice_command(cmd)
        time.sleep(2)  # Wait between commands

    system.shutdown()

if __name__ == '__main__':
    main()
```

## Testing Action Integration

Create tests to verify action triggering works correctly:

```python
import unittest
from unittest.mock import Mock, MagicMock

class TestVoiceActionIntegration(unittest.TestCase):
    def setUp(self):
        self.mock_action_client = Mock()
        self.action_mapper = VoiceToActionMapper(self.mock_action_client)

    def test_navigation_intent_mapping(self):
        """
        Test that navigation intents correctly map to navigation actions
        """
        params = {'location': 'kitchen'}
        result = self.action_mapper.execute_navigation(params)

        # Verify the action client was called with correct parameters
        self.mock_action_client.send_navigation_goal.assert_called()

    def test_invalid_location_handling(self):
        """
        Test handling of invalid locations
        """
        params = {'location': 'invalid_location'}
        result = self.action_mapper.execute_navigation(params)

        # Should handle gracefully without crashing
        self.assertIsNotNone(result)

if __name__ == '__main__':
    unittest.main()
```

## Practical Exercise

Implement ROS 2 action triggering for your robot:

1. Create action clients for your robot's main capabilities
2. Integrate with your intent classification system
3. Implement feedback handling for user communication
4. Add error handling and recovery mechanisms
5. Test with various voice commands to ensure reliable execution

## Summary

In this section, we've covered:
- The structure and benefits of ROS 2 actions for voice command execution
- How to create action clients for different robot capabilities
- Mapping classified intents to appropriate ROS 2 actions
- Handling feedback and status updates during action execution
- Error handling and recovery strategies
- Complete integration example showing all components working together

This completes Chapter 1 on Voice-to-Action Interfaces. You now have the foundation to build natural voice interaction systems for your robots.