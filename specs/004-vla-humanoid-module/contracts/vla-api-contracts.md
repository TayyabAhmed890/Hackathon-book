# VLA Module API Contracts

## Overview
This document defines the API contracts for the Vision-Language-Action (VLA) module services.

## Voice Interface API

### POST /voice/transcribe
Transcribes audio to text using OpenAI Whisper

**Request:**
- Content-Type: multipart/form-data
- Body: audio file (audio.wav, audio.mp3, etc.)

**Response (200):**
```json
{
  "transcript": "string",
  "confidence": "float",
  "language": "string",
  "duration": "float"
}
```

### POST /voice/intent-classify
Classifies the intent of a transcribed command

**Request:**
- Content-Type: application/json
- Body:
```json
{
  "transcript": "string",
  "context": "string (optional)"
}
```

**Response (200):**
```json
{
  "intent": {
    "action_type": "string",
    "target_object": "string",
    "parameters": "object",
    "confidence": "float"
  }
}
```

## Task Planning API

### POST /planning/generate
Generates a task plan from a natural language request

**Request:**
- Content-Type: application/json
- Body:
```json
{
  "request": "string",
  "context": "object (optional)"
}
```

**Response (200):**
```json
{
  "task_plan": {
    "id": "string",
    "user_request": "string",
    "plan_steps": [
      {
        "id": "string",
        "action": "string",
        "parameters": "object",
        "dependencies": ["string"],
        "timeout": "float",
        "success_criteria": ["string"]
      }
    ],
    "estimated_duration": "float"
  }
}
```

### POST /planning/execute
Executes a generated task plan

**Request:**
- Content-Type: application/json
- Body:
```json
{
  "task_plan_id": "string",
  "plan": "object (task plan structure)"
}
```

**Response (200):**
```json
{
  "execution_id": "string",
  "status": "string",
  "start_time": "datetime"
}
```

## ROS 2 Interface API

### POST /ros2/action/trigger
Triggers a ROS 2 action

**Request:**
- Content-Type: application/json
- Body:
```json
{
  "action_name": "string",
  "goal": "object",
  "timeout": "float (optional)"
}
```

**Response (200):**
```json
{
  "request_id": "string",
  "status": "string",
  "execution_time": "float"
}
```

### GET /ros2/action/status/{request_id}
Gets the status of a ROS 2 action

**Response (200):**
```json
{
  "request_id": "string",
  "status": "string",
  "result_data": "object",
  "execution_time": "float"
}
```

## Perception API

### GET /perception/environment
Gets the current perception data from the environment

**Response (200):**
```json
{
  "timestamp": "datetime",
  "objects_detected": [
    {
      "id": "string",
      "object_type": "string",
      "position": {
        "x": "float",
        "y": "float",
        "z": "float"
      },
      "orientation": {
        "x": "float",
        "y": "float",
        "z": "float",
        "w": "float"
      },
      "confidence": "float",
      "properties": "object"
    }
  ]
}
```

### GET /perception/simulation-state
Gets the current simulation state

**Response (200):**
```json
{
  "timestamp": "datetime",
  "robot_pose": {
    "position": {
      "x": "float",
      "y": "float",
      "z": "float"
    },
    "orientation": {
      "x": "float",
      "y": "float",
      "z": "float",
      "w": "float"
    }
  },
  "robot_status": "string",
  "environment_objects": ["SimulatedObject"],
  "active_tasks": ["string"]
}
```

## Student Interaction API

### POST /session/start
Starts a new student session

**Request:**
- Content-Type: application/json
- Body:
```json
{
  "student_id": "string"
}
```

**Response (200):**
```json
{
  "session_id": "string",
  "start_time": "datetime"
}
```

### POST /session/command
Submits a command in the context of a session

**Request:**
- Content-Type: application/json
- Body:
```json
{
  "session_id": "string",
  "command_type": "string (voice, text, etc.)",
  "command_data": "object"
}
```

**Response (200):**
```json
{
  "command_id": "string",
  "processing_status": "string"
}
```

### GET /session/{session_id}/performance
Gets performance metrics for a session

**Response (200):**
```json
{
  "session_id": "string",
  "performance_metrics": {
    "command_accuracy": "float",
    "task_completion_rate": "float",
    "average_response_time": "float",
    "error_recovery_rate": "float",
    "learning_progress": "object"
  }
}