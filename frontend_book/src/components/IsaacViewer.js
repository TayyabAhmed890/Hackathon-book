import React from 'react';
import styles from './IsaacViewer.module.css';

// A component for displaying Isaac-related content and visualizations
// This could be extended to show embedded simulations, videos, or interactive elements
const IsaacViewer = ({ title, description, children, type = 'general' }) => {
  const getTypeClass = () => {
    switch(type) {
      case 'simulation':
        return styles.simulationType;
      case 'perception':
        return styles.perceptionType;
      case 'navigation':
        return styles.navigationType;
      default:
        return styles.generalType;
    }
  };

  return (
    <div className={`${styles.isaacViewer} ${getTypeClass()}`}>
      <div className={styles.header}>
        <h3>{title}</h3>
      </div>
      <div className={styles.content}>
        <p>{description}</p>
        <div className={styles.visualizationContainer}>
          {children}
        </div>
        <div className={styles.isaacInfo}>
          <p className={styles.infoText}>
            This visualization demonstrates NVIDIA Isaac technology for {type} applications.
          </p>
        </div>
      </div>
    </div>
  );
};

export default IsaacViewer;