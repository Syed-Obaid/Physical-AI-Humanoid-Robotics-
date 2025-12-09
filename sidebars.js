// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    'guide',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Foundation',
      items: [
        'module1/overview',
        'module1/installation',
        'module1/nodes-topics',
        'module1/urdf-basics',
        'module1/services-actions',
        'module1/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module2/overview',
        'module2/gazebo-setup',
        'module2/sensor-simulation',
        'module2/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Motion Planning',
      items: [
        'module3/overview',
        'module3/moveit-setup',
        'module3/kinematics',
        'module3/trajectory-planning',
        'module3/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Embodied AI',
      items: [
        'module4/overview',
        'module4/vision-integration',
        'module4/language-models',
        'module4/reinforcement-learning',
        'module4/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Advanced Topics',
      items: [
        'module5/exercises',
      ],
    },
    'glossary',
    'notation',
    'references',
  ],
};

module.exports = sidebars;