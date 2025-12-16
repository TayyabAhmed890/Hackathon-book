import React from 'react';

const VLADiagram = ({ title = "Vision-Language-Action System" }) => {
  return (
    <div style={{ padding: '20px', border: '1px solid #ccc', borderRadius: '8px', margin: '20px 0' }}>
      <h3 style={{ textAlign: 'center', color: '#2c3e50' }}>{title}</h3>
      <div style={{ display: 'flex', justifyContent: 'space-around', alignItems: 'center', flexWrap: 'wrap' }}>
        <div style={{ textAlign: 'center', margin: '10px' }}>
          <div style={{
            backgroundColor: '#3498db',
            color: 'white',
            padding: '15px',
            borderRadius: '8px',
            minWidth: '120px',
            boxShadow: '0 4px 8px rgba(0,0,0,0.1)'
          }}>
            <strong>Vision</strong>
            <p style={{ margin: '5px 0', fontSize: '0.9em' }}>Perception & Sensing</p>
          </div>
        </div>
        <div style={{ fontSize: '24px', margin: '0 10px' }}>&#8594;</div>
        <div style={{ textAlign: 'center', margin: '10px' }}>
          <div style={{
            backgroundColor: '#e74c3c',
            color: 'white',
            padding: '15px',
            borderRadius: '8px',
            minWidth: '120px',
            boxShadow: '0 4px 8px rgba(0,0,0,0.1)'
          }}>
            <strong>Language</strong>
            <p style={{ margin: '5px 0', fontSize: '0.9em' }}>LLM Processing</p>
          </div>
        </div>
        <div style={{ fontSize: '24px', margin: '0 10px' }}>&#8594;</div>
        <div style={{ textAlign: 'center', margin: '10px' }}>
          <div style={{
            backgroundColor: '#2ecc71',
            color: 'white',
            padding: '15px',
            borderRadius: '8px',
            minWidth: '120px',
            boxShadow: '0 4px 8px rgba(0,0,0,0.1)'
          }}>
            <strong>Action</strong>
            <p style={{ margin: '5px 0', fontSize: '0.9em' }}>Robot Control</p>
          </div>
        </div>
      </div>
      <p style={{ textAlign: 'center', marginTop: '15px', fontStyle: 'italic' }}>
        The VLA pipeline: Vision processes environment → Language interprets commands → Action executes robot behaviors
      </p>
    </div>
  );
};

export default VLADiagram;