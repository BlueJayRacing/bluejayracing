import React from 'react';
import { useThree, useFrame } from '@react-three/fiber';

// Overlay component with interactive sliders
export const CameraOverlay = ({ position, onPositionChange }) => {
  const handleSliderChange = (index, value) => {
    const newPosition = [...position];
    newPosition[index] = parseFloat(value);
    onPositionChange(newPosition);
  };

  return (
    <div
      style={{
        position: "absolute",
        top: "150px",
        left: "10px",
        backgroundColor: "rgba(255, 255, 255, 0.8)",
        padding: "8px",
        borderRadius: "4px",
        zIndex: 999,
        fontFamily: "monospace"
      }}
    >
      <div style={{ marginBottom: "8px" }}>
        <label style={{ display: "block", marginBottom: "4px" }}>
          X: {position[0].toFixed(3)}
        </label>
        <input
          type="range"
          min={-20}
          max={20}
          step={0.1}
          value={position[0]}
          onChange={(e) => handleSliderChange(0, e.target.value)}
          style={{ width: "200px" }}
        />
      </div>

      <div style={{ marginBottom: "8px" }}>
        <label style={{ display: "block", marginBottom: "4px" }}>
          Y: {position[1].toFixed(3)}
        </label>
        <input
          type="range"
          min={-20}
          max={20}
          step={0.1}
          value={position[1]}
          onChange={(e) => handleSliderChange(1, e.target.value)}
          style={{ width: "200px" }}
        />
      </div>

      <div style={{ marginBottom: "8px" }}>
        <label style={{ display: "block", marginBottom: "4px" }}>
          Z: {position[2].toFixed(3)}
        </label>
        <input
          type="range"
          min={-20}
          max={20}
          step={0.1}
          value={position[2]}
          onChange={(e) => handleSliderChange(2, e.target.value)}
          style={{ width: "200px" }}
        />
      </div>
    </div>
  );
};