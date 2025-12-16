# Quickstart Guide: Vision-Language-Action (VLA) Module

## Overview
This guide will help you set up and start working with the Vision-Language-Action module. This module demonstrates how large language models, perception, and robotics converge to produce intelligent humanoid behavior.

## Prerequisites
- Python 3.8+ installed
- ROS 2 Humble Hawksbill (or newer) installed and sourced
- Node.js 16+ and npm installed for Docusaurus
- OpenAI API key for Whisper and LLM services
- NVIDIA Isaac Sim (optional, for full simulation)

## Environment Setup

### 1. Clone and Initialize the Repository
```bash
git clone [repository-url]
cd [repository-name]
git checkout 004-vla-humanoid-module
```

### 2. Set up Python Environment
```bash
cd [project-root]
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Install ROS 2 Dependencies
```bash
# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# Install additional ROS 2 packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-moveit ros-humble-moveit-ros ros-humble-rosbridge-suite
```

### 4. Configure API Keys
Create a `.env` file in the project root:
```env
OPENAI_API_KEY=your_openai_api_key_here
ROS_DOMAIN_ID=42  # Optional: set a specific ROS domain
```

## Running the VLA System

### 1. Start the ROS 2 System
```bash
# Terminal 1: Start the robot simulation
source /opt/ros/humble/setup.bash
cd [project-root]
ros2 launch robot_bringup robot.launch.py

# Terminal 2: Start the VLA services
source venv/bin/activate
cd [project-root]
python -m vla_services.main
```

### 2. Run the Docusaurus Documentation Site
```bash
cd frontend_book
npm install
npm run start
```

## Key Components

### Voice Interface
The voice interface handles speech recognition and intent classification:

```python
from vla_services.voice_interface import VoiceInterface

# Initialize the voice interface
voice_interface = VoiceInterface(
    whisper_model="base",  # or "small", "medium", "large"
    api_key=os.getenv("OPENAI_API_KEY")
)

# Process a voice command
result = voice_interface.process_audio("audio_file.wav")
print(f"Transcript: {result.transcript}")
print(f"Intent: {result.intent_classification}")
```

### LLM Task Planner
The task planner generates action sequences from natural language:

```python
from vla_services.task_planner import TaskPlanner

# Initialize the planner
planner = TaskPlanner(api_key=os.getenv("OPENAI_API_KEY"))

# Generate a task plan
user_request = "Go to the kitchen and bring me the red cup"
task_plan = planner.generate_plan(user_request)

print(f"Task plan: {task_plan.plan_steps}")
```

### ROS 2 Action Interface
The action interface executes robot behaviors:

```python
from vla_services.ros_interface import ROSInterface

# Initialize the ROS interface
ros_interface = ROSInterface()

# Execute a navigation action
nav_result = ros_interface.execute_navigation(
    goal_x=1.0,
    goal_y=2.0,
    goal_theta=0.0
)

print(f"Navigation result: {nav_result.status}")
```

## Content Structure

### Chapter 1: Voice-to-Action Interfaces
Located in `frontend_book/docs/module-4-vla/chapter-1-voice-to-action/`

- `index.md` - Introduction to voice interfaces
- `whisper-integration.md` - Integrating OpenAI Whisper
- `speech-to-intent.md` - Converting speech to actionable intents
- `ros2-actions.md` - Triggering ROS 2 actions from voice commands

### Chapter 2: Cognitive Planning with LLMs
Located in `frontend_book/docs/module-4-vla/chapter-2-cognitive-planning/`

- `index.md` - Introduction to LLM-based planning
- `llm-task-planning.md` - How LLMs generate task plans
- `natural-language-understanding.md` - Understanding complex commands
- `ros2-behaviors.md` - Mapping plans to ROS 2 behaviors

### Chapter 3: Capstone - The Autonomous Humanoid
Located in `frontend_book/docs/module-4-vla/chapter-3-capstone/`

- `index.md` - Introduction to the capstone system
- `end-to-end-integration.md` - Bringing all components together
- `simulation-environment.md` - Working with the simulation
- `autonomous-execution.md` - Running autonomous tasks

## Development Workflow

### 1. Adding New Voice Commands
1. Define the new command pattern in the intent classification system
2. Create the corresponding ROS 2 action handler
3. Update the documentation in Chapter 1

### 2. Extending Task Planning
1. Add examples to the LLM prompt for the new task type
2. Implement the action mapping in the plan executor
3. Update the documentation in Chapter 2

### 3. Enhancing the Simulation
1. Add new simulation scenarios in Isaac Sim
2. Create corresponding ROS 2 interfaces
3. Document the new capabilities in Chapter 3

## Testing

### Unit Tests
```bash
# Run all unit tests
python -m pytest tests/unit/

# Run voice interface tests
python -m pytest tests/unit/test_voice_interface.py

# Run task planner tests
python -m pytest tests/unit/test_task_planner.py
```

### Integration Tests
```bash
# Run integration tests
python -m pytest tests/integration/
```

## Troubleshooting

### Common Issues
1. **ROS 2 Connection Issues**: Ensure ROS_DOMAIN_ID is consistent across all terminals
2. **API Rate Limits**: Add rate limiting to OpenAI API calls
3. **Audio Input Problems**: Check microphone permissions and audio format compatibility

### Debugging Tips
- Enable debug logging: `export VLA_DEBUG=1`
- Check ROS 2 topics: `ros2 topic list`
- Monitor system performance: `ros2 run rqt_plot rqt_plot`

## Next Steps

1. Complete Chapter 1 exercises: Basic voice commands
2. Complete Chapter 2 exercises: Complex task planning
3. Complete Chapter 3 capstone: Full autonomous operation
4. Explore advanced topics in the tutorials section