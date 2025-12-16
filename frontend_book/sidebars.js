/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/intro',
        'module-1/chapter-1-ros2-fundamentals',
        'module-1/chapter-2-ros2-communication',
        'module-1/chapter-3-python-agents',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/intro',
        'module-2/chapter-1-physics-simulation-gazebo',
        'module-2/chapter-2-unity-environments',
        'module-2/chapter-3-sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/intro',
        'module-3/chapter-1-isaac-sim',
        'module-3/chapter-2-isaac-ros-perception',
        'module-3/chapter-3-navigation-with-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/index',
        {
          type: 'category',
          label: 'Chapter 1: Voice-to-Action Interfaces',
          items: [
            'module-4-vla/chapter-1-voice-to-action/index',
            'module-4-vla/chapter-1-voice-to-action/whisper-integration',
            'module-4-vla/chapter-1-voice-to-action/speech-to-intent',
            'module-4-vla/chapter-1-voice-to-action/ros2-actions',
            'module-4-vla/chapter-1-voice-to-action/practical-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Cognitive Planning with LLMs',
          items: [
            'module-4-vla/chapter-2-cognitive-planning/index',
            'module-4-vla/chapter-2-cognitive-planning/llm-task-planning',
            'module-4-vla/chapter-2-cognitive-planning/natural-language-understanding',
            'module-4-vla/chapter-2-cognitive-planning/ros2-behaviors',
            'module-4-vla/chapter-2-cognitive-planning/practical-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Capstone - The Autonomous Humanoid',
          items: [
            'module-4-vla/chapter-3-capstone/index',
            'module-4-vla/chapter-3-capstone/end-to-end-integration',
            'module-4-vla/chapter-3-capstone/simulation-environment',
            'module-4-vla/chapter-3-capstone/autonomous-execution',
            'module-4-vla/chapter-3-capstone/practical-exercises',
          ],
        },
        'module-4-vla/assessment',
      ],
    },
  ],
};

module.exports = sidebars;