import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: '🤖⚙️ ROS 2 Fundamentals',
    description: (
      <>
        Learn the fundamentals of Robot Operating System 2, the nervous system of modern robotics.
        Understand nodes, topics, services, and actions that connect all robot components.
      </>
    ),
  },
  {
    title: '🧿🧬 Digital Twins & Simulation ',
    description: (
      <>
        Master Gazebo and Unity simulation environments to create digital twins of your robots.
        Test and validate your systems in safe, virtual environments before real-world deployment.
      </>
    ),
  },
  {
    title: '🧠👁️ AI-Powered Perception ',
    description: (
      <>
        Explore NVIDIA Isaac for hardware-accelerated perception systems.
        Implement visual SLAM, object detection, and sensor fusion for intelligent robots.
      </>
    ),
  },
  {
    title: '👁️📝 Vision-Language-Action ',
    description: (
      <>
        Build intelligent robots that understand natural language commands.
        Integrate perception, reasoning, and action for human-like robot interaction.
      </>
    ),
  },
  {
    title: ' ⭐ Capstone Project ⭐',
    description: (
      <>
        Integrate all concepts in a comprehensive humanoid robot project.
        Combine ROS 2, simulation, AI perception, and VLA for a complete system.
      </>
    ),
  },
  {
    title: '🏭🚀 Industry Ready ',
    description: (
      <>
        Learn best practices for deploying robotics systems in real-world applications.
        Understand safety, reliability, and maintainability requirements for production systems.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}