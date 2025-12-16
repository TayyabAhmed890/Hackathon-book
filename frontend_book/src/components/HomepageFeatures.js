import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Learn ROS 2 Fundamentals',
    description: (
      <>
        Master the fundamentals of ROS 2, the middleware that connects AI agents
        to humanoid robot control systems.
      </>
    ),
  },
  {
    title: 'Connect AI to Robotics',
    description: (
      <>
        Bridge the gap between AI algorithms and robotic hardware using Python
        and ROS 2 communication patterns.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    description: (
      <>
        Explore how AI agents interact with humanoid robots through practical
        examples and hands-on exercises.
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