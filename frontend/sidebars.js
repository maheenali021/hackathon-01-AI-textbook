// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      link: {type: 'doc', id: 'intro'},
      collapsed: false, // Keep the main category open by default
      items: [
        {
          type: 'category',
          label: 'Introduction',
          collapsed: false, // Keep intro open
          items: [
            'intro/physical-ai',
            'intro/humanoid-robotics',
          ],
        },
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          collapsed: false, // Keep module 1 open
          items: [
            'module1/what-is-robotic-nervous-system',
            'module1/setup-ros2-humble',
            'module1/ros2-architecture-basics',
            'module1/publishers-subscribers',
            'module1/services-actions-control',
            'module1/bridging-python-agents',
            'module1/understanding-urdf',
            'module1/module1-milestone',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          collapsed: false, // Keep module 2 open
          items: [
            'module2/intro-to-digital-twins',
            'module2/gazebo-basics',
            'module2/unity-visualization',
            'module2/sensor-simulation',
            'module2/module2-milestone',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          collapsed: false, // Keep module 3 open
          items: [
            'module3/intro-to-nvidia-isaac',
            'module3/isaac-ros',
            'module3/nav2-path-planning',
            'module3/module3-milestone',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          collapsed: false, // Keep module 4 open
          items: [
            'module4/intro-to-vla',
            'module4/voice-to-action',
            'module4/cognitive-planning',
            'module4/multi-modal-integration',
            'module4/module4-milestone',
          ],
        },
        {
          type: 'category',
          label: 'Capstone: Autonomous Humanoid Robot',
          collapsed: false, // Keep capstone open
          items: [
            'capstone/capstone-overview',
            'capstone/simulation-testing',
            'capstone/best-practices',
          ],
        },
      ],
    },
  ],

};

export default sidebars;