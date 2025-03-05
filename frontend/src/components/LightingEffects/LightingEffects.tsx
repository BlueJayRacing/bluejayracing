import React from 'react';

const LightingEffects: React.FC = () => {
  return (
    <>
      {/* Ambient Light */}
      <ambientLight intensity={0.5} />
      {/* Directional Light */}
      <directionalLight position={[10, 10, 5]} intensity={1} />
      {/* Point Light for Glow Effects */}
      <pointLight position={[0, 5, 0]} intensity={1} />
      {/* Additional lights or effects as needed */}
    </>
  );
};

export default LightingEffects;
