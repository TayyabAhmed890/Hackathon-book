import React from 'react';
import styles from './SimulationViewer.module.css';

// A placeholder component for displaying simulation content
// This could be extended to show embedded simulations, videos, or interactive elements
const SimulationViewer = ({ title, description, children }) => {
  return (
    <div className={styles.simulationViewer}>
      <div className={styles.header}>
        <h3>{title}</h3>
      </div>
      <div className={styles.content}>
        <p>{description}</p>
        <div className={styles.simulationContainer}>
          {children}
        </div>
      </div>
    </div>
  );
};

export default SimulationViewer;