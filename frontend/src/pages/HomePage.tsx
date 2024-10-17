// HomePage.tsx
import React, { useState } from 'react';
import { Canvas } from '@react-three/fiber';
import CarModel from '../components/CarModel/CarModel';
import SuspensionModel from '../components/SuspensionModel/SuspensionModel';
import SuspensionControls from '../components/SuspensionControls/SuspensionControls';
import CameraControls from '../components/CameraControls/CameraControls';
import LightingEffects from '../components/LightingEffects/LightingEffects';
import useSuspensionData from '../hooks/useSuspensionData';

const HomePage: React.FC = () => {
  const [shockExtension, setShockExtension] = useState(0);
  const [showSuspension, setShowSuspension] = useState(true);

  useSuspensionData(); // Start polling for suspension data

  return (
    <div className="flex h-screen" style={{ width: "100vw", height: "100vh" }}>
      <div className="w-1/4 p-4 overflow-y-auto">
        <SuspensionControls
          shockExtension={shockExtension}
          setShockExtension={setShockExtension}
          showSuspension={showSuspension}
          setShowSuspension={setShowSuspension}
        />
      </div>
      <div className="w-3/4 h-full relative" style={{ width: "100%", height: "100%" }}>
        <Canvas
          className="absolute top-0 left-0 w-full h-full"
          camera={{ position: [0, 5, 10], fov: 60 }}
        >
          <CameraControls />
          <LightingEffects />
          <CarModel showSuspension={!showSuspension} />
          {showSuspension && <SuspensionModel shockExtension={shockExtension} />}
        </Canvas>
      </div>
    </div>
  );
};

export default HomePage;