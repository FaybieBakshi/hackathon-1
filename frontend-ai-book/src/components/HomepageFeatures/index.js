import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
       Learn how ROS 2 works as the nervous system of humnoid robots,enabling communication,control and real-time execution.
      </>
    ),
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        In this module, you'll explore the world of digital twins for robotics, focusing on physics simulation and sensor modeling for humanoid robots.
      </>
    ),
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
         In this module, you'll explore advanced perception and navigation using NVIDIA Isaac for humanoid robots. 
      </>
    ),
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
       In this module, you'll explore the integration of Vision-Language-Action systems with Large Language Models for voice-controlled humanoid robots. This module builds upon the foundations laid in Modules 1-3 ROS 2 basics, simulation concepts
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
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
