---
sidebar_position: 3
title: "Speech-to-Intent Processing"
description: "Converting recognized speech into actionable intents for robotic systems"
---

# Speech-to-Intent Processing

## Introduction

Once speech has been converted to text using a system like OpenAI Whisper, the next critical step is to determine the user's intent. Speech-to-intent processing involves understanding what the user wants the robot to do based on the transcribed text. This is where natural language understanding meets robotic action planning.

## The Intent Classification Challenge

Converting speech to intent is more complex than simple keyword matching because:

- Users may express the same intent in many different ways
- Natural language is ambiguous and context-dependent
- Spoken commands may be incomplete or imprecise
- The robot needs to understand both the action and its parameters

## Approaches to Intent Classification

### Rule-Based Approach

A simple approach uses predefined rules and patterns:

```python
import re

class RuleBasedIntentClassifier:
    def __init__(self):
        self.intent_patterns = {
            "move_forward": [
                r"move forward",
                r"go forward",
                r"move ahead",
                r"forward",
                r"straight"
            ],
            "turn_left": [
                r"turn left",
                r"go left",
                r"turn anticlockwise",
                r"rotate left"
            ],
            "turn_right": [
                r"turn right",
                r"go right",
                r"turn clockwise",
                r"rotate right"
            ],
            "stop": [
                r"stop",
                r"halt",
                r"pause",
                r"freeze"
            ],
            "navigate_to": [
                r"go to (the )?(?P<location>\w+)",
                r"move to (the )?(?P<location>\w+)",
                r"navigate to (the )?(?P<location>\w+)"
            ]
        }

    def classify_intent(self, text):
        """
        Classify intent based on predefined patterns
        """
        text_lower = text.lower()

        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    # Extract any parameters
                    params = match.groupdict()
                    return {
                        "intent": intent,
                        "confidence": 0.9,  # High confidence for rule match
                        "parameters": params
                    }

        return {
            "intent": "unknown",
            "confidence": 0.0,
            "parameters": {}
        }
```

### LLM-Based Approach

For more sophisticated intent classification, we can use large language models:

```python
import openai
import json

class LLMIntentClassifier:
    def __init__(self, api_key):
        self.client = openai.OpenAI(api_key=api_key)

    def classify_intent(self, text):
        """
        Use LLM to classify intent and extract parameters
        """
        prompt = f"""
        You are a robot command interpreter. Analyze the following user command and extract:
        1. The primary intent (one of: move_forward, move_backward, turn_left, turn_right, stop, pick_up, place, navigate_to, unknown)
        2. Any relevant parameters (like location, object name, distance, etc.)

        Command: "{text}"

        Respond in JSON format:
        {{
            "intent": "...",
            "confidence": 0.0-1.0,
            "parameters": {{
                "location": "...",
                "object": "...",
                "distance": "...",
                "other": "..."
            }}
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            # Parse the JSON response
            result_text = response.choices[0].message.content.strip()

            # Extract JSON from response (in case it includes extra text)
            json_start = result_text.find('{')
            json_end = result_text.rfind('}') + 1

            if json_start != -1 and json_end != 0:
                json_str = result_text[json_start:json_end]
                result = json.loads(json_str)
                return result
            else:
                return {
                    "intent": "unknown",
                    "confidence": 0.0,
                    "parameters": {}
                }

        except Exception as e:
            print(f"LLM classification error: {e}")
            return {
                "intent": "unknown",
                "confidence": 0.0,
                "parameters": {}
            }
```

## Handling Ambiguity

Real-world voice commands often contain ambiguity that needs to be resolved:

```python
class IntentDisambiguator:
    def __init__(self):
        self.context = {}  # Store conversation context

    def resolve_ambiguity(self, intent_result, context=None):
        """
        Resolve ambiguous intents based on context
        """
        intent = intent_result["intent"]
        params = intent_result["parameters"]

        if intent == "navigate_to" and "location" not in params:
            # Ask for clarification if location is missing
            return {
                "intent": "request_clarification",
                "message": "Where would you like me to go?",
                "original_intent": intent
            }

        if intent == "pick_up" and "object" not in params:
            # Use context or ask for clarification
            if context and "last_mentioned_object" in context:
                params["object"] = context["last_mentioned_object"]
                return intent_result
            else:
                return {
                    "intent": "request_clarification",
                    "message": "Which object would you like me to pick up?",
                    "original_intent": intent
                }

        return intent_result

    def handle_clarification_response(self, user_response, original_intent):
        """
        Process user's clarification response
        """
        # Process the clarification and return updated intent
        # This would involve re-processing with the additional information
        pass
```

## Intent Confidence and Validation

Not all classified intents should be executed immediately:

```python
class IntentValidator:
    def __init__(self):
        self.min_confidence = 0.7
        self.known_locations = ["kitchen", "bedroom", "living room", "office", "dining room"]
        self.known_objects = ["cup", "book", "ball", "box", "chair"]

    def validate_intent(self, intent_result):
        """
        Validate the intent before execution
        """
        intent = intent_result["intent"]
        confidence = intent_result["confidence"]
        params = intent_result["parameters"]

        # Check confidence threshold
        if confidence < self.min_confidence:
            return {
                "valid": False,
                "reason": f"Low confidence ({confidence:.2f} < {self.min_confidence})",
                "suggested_action": "request_confirmation"
            }

        # Validate specific parameters
        if intent == "navigate_to" and "location" in params:
            location = params["location"]
            if location not in self.known_locations:
                return {
                    "valid": False,
                    "reason": f"Unknown location: {location}",
                    "suggested_action": "request_confirmation"
                }

        if intent == "pick_up" and "object" in params:
            obj = params["object"]
            if obj not in self.known_objects:
                return {
                    "valid": False,
                    "reason": f"Unknown object: {obj}",
                    "suggested_action": "request_confirmation"
                }

        return {
            "valid": True,
            "reason": "Intent validated successfully",
            "suggested_action": "execute"
        }
```

## Integration with Robot Control

Once an intent is classified and validated, it needs to be mapped to robot actions:

```python
class IntentToActionMapper:
    def __init__(self, robot_controller):
        self.robot = robot_controller

    def execute_intent(self, intent_result):
        """
        Map intent to robot action and execute
        """
        intent = intent_result["intent"]
        params = intent_result["parameters"]

        if intent == "move_forward":
            distance = params.get("distance", 1.0)  # Default 1 meter
            return self.robot.move_forward(distance)

        elif intent == "turn_left":
            angle = params.get("angle", 90)  # Default 90 degrees
            return self.robot.turn(angle)

        elif intent == "turn_right":
            angle = params.get("angle", 90)  # Default 90 degrees
            return self.robot.turn(-angle)  # Negative for right turn

        elif intent == "stop":
            return self.robot.stop()

        elif intent == "navigate_to":
            location = params.get("location")
            if location:
                return self.robot.navigate_to(location)
            else:
                return False  # Missing required parameter

        elif intent == "pick_up":
            obj = params.get("object")
            if obj:
                return self.robot.pick_up_object(obj)
            else:
                return False  # Missing required parameter

        else:
            print(f"Unknown intent: {intent}")
            return False
```

## Error Handling and Recovery

Implement comprehensive error handling for robust operation:

```python
class RobustIntentProcessor:
    def __init__(self, classifier, validator, mapper):
        self.classifier = classifier
        self.validator = validator
        self.mapper = mapper

    def process_voice_command(self, text):
        """
        Complete pipeline: classify -> validate -> execute
        """
        try:
            # Step 1: Classify intent
            intent_result = self.classifier.classify_intent(text)

            # Step 2: Validate intent
            validation_result = self.validator.validate_intent(intent_result)

            if not validation_result["valid"]:
                # Handle invalid intent
                if validation_result["suggested_action"] == "request_confirmation":
                    return self.handle_uncertainty(intent_result, validation_result)
                else:
                    return False

            # Step 3: Execute intent
            execution_result = self.mapper.execute_intent(intent_result)

            return execution_result

        except Exception as e:
            print(f"Error processing voice command: {e}")
            return False

    def handle_uncertainty(self, intent_result, validation_result):
        """
        Handle uncertain or invalid intents
        """
        reason = validation_result["reason"]
        print(f"Uncertainty detected: {reason}")

        # Could implement various strategies:
        # 1. Ask for clarification
        # 2. Use default behavior
        # 3. Provide options to user
        # 4. Cancel operation

        return False  # For now, just return failure
```

## Practical Exercise

Implement an intent classifier for your robot:

1. Create a rule-based classifier for basic commands
2. Integrate with an LLM for more complex intent understanding
3. Add validation to ensure safe execution
4. Map intents to actual robot actions
5. Test with various voice commands and edge cases

## Summary

In this section, we've covered:
- Different approaches to intent classification (rule-based vs. LLM-based)
- Techniques for handling ambiguous commands
- Validation strategies to ensure safe execution
- Mapping intents to robot actions
- Error handling and recovery mechanisms

Next, we'll explore how to trigger ROS 2 actions based on these classified intents.