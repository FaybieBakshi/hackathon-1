// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/chapter-1-ros-core',
        'module-1/chapter-2-python-bridge',
        'module-1/chapter-3-urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/index',
        'module-2/chapter-1-gazebo-fundamentals/index',
        'module-2/chapter-1-gazebo-fundamentals/physics-concepts',
        'module-2/chapter-1-gazebo-fundamentals/simulation-env',
        'module-2/chapter-1-gazebo-fundamentals/practical-exercises',
        'module-2/chapter-2-unity-rendering/index',
        'module-2/chapter-2-unity-rendering/visual-rendering',
        'module-2/chapter-2-unity-rendering/unity-integration',
        'module-2/chapter-2-unity-rendering/interaction-design',
        'module-2/chapter-3-sensor-simulation/index',
        'module-2/chapter-3-sensor-simulation/lidar-simulation',
        'module-2/chapter-3-sensor-simulation/depth-camera-sim',
        'module-2/chapter-3-sensor-simulation/imu-simulation',
        'module-2/chapter-3-sensor-simulation/sensor-fusion',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3/index',
        'module-3/isaac-sim-synthetic-data',
        'module-3/isaac-ros-vslam',
        'module-3/nav2-bipedal-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/index',
        'module-4/voice-to-action',
        'module-4/cognitive-planning',
        'module-4/capstone-project',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
