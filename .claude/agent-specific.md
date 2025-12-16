# AI-Driven Technical Book Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-16

## Active Technologies

- Markdown
- Docusaurus static site generator
- React components
- OpenAI Whisper API
- OpenAI GPT models
- ROS 2 Humble Hawksbill
- NVIDIA Isaac Sim
- Python 3.8+
- Node.js 16+

## Project Structure

```text
frontend_book/
├── docs/
│   ├── module-4-vla/
│   │   ├── chapter-1-voice-to-action/
│   │   │   ├── index.md
│   │   │   ├── whisper-integration.md
│   │   │   ├── speech-to-intent.md
│   │   │   └── ros2-actions.md
│   │   ├── chapter-2-cognitive-planning/
│   │   │   ├── index.md
│   │   │   ├── llm-task-planning.md
│   │   │   ├── natural-language-understanding.md
│   │   │   └── ros2-behaviors.md
│   │   └── chapter-3-capstone/
│   │       ├── index.md
│   │       ├── end-to-end-integration.md
│   │       ├── simulation-environment.md
│   │       └── autonomous-execution.md
│   └── tutorials/
│       └── vla-workflow.md
├── src/
│   ├── components/
│   │   └── vla-diagram/
│   └── pages/
└── docusaurus.config.js
specs/004-vla-humanoid-module/
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
├── contracts/
│   └── vla-api-contracts.md
└── tasks.md
```

## Commands

### Docusaurus Commands
```bash
# Start development server
cd frontend_book
npm run start

# Build static site
npm run build

# Deploy to GitHub Pages
npm run deploy
```

### ROS 2 Commands
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Start robot simulation
ros2 launch robot_bringup robot.launch.py

# List available topics
ros2 topic list
```

### Python Commands
```bash
# Activate virtual environment
source venv/bin/activate

# Run VLA services
python -m vla_services.main

# Run tests
python -m pytest tests/
```

## Code Style

### Markdown Style
- Use proper heading hierarchy (#, ##, ###)
- Include alt text for all images
- Use proper code fencing with language specification
- Follow Docusaurus documentation best practices

### Python Style
- Follow PEP 8 guidelines
- Use type hints for all function signatures
- Include docstrings for all public functions
- Use meaningful variable names

## Recent Changes

### Module 4: Vision-Language-Action (VLA)
- Added documentation for voice-to-action interfaces using OpenAI Whisper
- Implemented LLM-based cognitive planning for robotic tasks
- Created capstone autonomous humanoid system with simulation

### Module 3: NVIDIA Isaac AI Integration
- Integrated Isaac Sim for robotics simulation
- Connected perception systems with AI models
- Implemented navigation and manipulation in simulation

### Module 2: Digital Twin Simulation
- Created digital twin architecture
- Implemented real-time synchronization between physical and virtual systems
- Added visualization and monitoring capabilities

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->