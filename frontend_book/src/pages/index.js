import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1 className="hero__title">{siteConfig.title}</h1>
            {/* <p className="hero__subtitle">{siteConfig.tagline}</p> */}
            <p className={styles.heroDescription}>
              Master the intersection of artificial intelligence and humanoid robotics.
              Learn how modern AI escapes the screen and comes alive through robots, simulation, and intelligent action
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Start Reading
              </Link>
              <Link
                className="button button--secondary button--lg margin-left--md"
                to="/docs/module-1/chapter-1-ros2-fundamentals">
                Begin Course
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics - Module 1: The Robotic Nervous System (ROS 2)">
      <HomepageHeader />
    </Layout>
  );
}