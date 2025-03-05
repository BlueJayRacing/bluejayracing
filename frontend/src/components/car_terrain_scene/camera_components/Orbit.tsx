// File: src/components/car_terrain_scene/camera_components/Orbit.tsx

import React, { useEffect } from 'react';
import { useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';

interface OrbitProps {
  horizontalPosition: number;
  verticalPosition: number;
  target?: [number, number, number];
  minDistance?: number;
  maxDistance?: number;
  enabled?: boolean;
}

const Orbit: React.FC<OrbitProps> = ({
  horizontalPosition,
  verticalPosition,
  target = [35, -10, 0],
  minDistance = 10,
  maxDistance = 200,
  enabled = true,
}) => {
  const { camera, gl } = useThree();

  useEffect(() => {
    // Update camera settings when props change
    camera.near = 0.1;
    camera.far = 1000;
    camera.updateProjectionMatrix();
  }, [camera]);

  return (
    <OrbitControls
      enablePan={enabled}
      enableZoom={enabled}
      enableRotate={enabled}
      maxPolarAngle={verticalPosition}
      minPolarAngle={verticalPosition}
      minAzimuthAngle={horizontalPosition}
      maxAzimuthAngle={horizontalPosition}
      args={[camera, gl.domElement]}
      target={target}
      minDistance={minDistance}
      maxDistance={maxDistance}
      // Ensure smooth damping for better UX
      dampingFactor={0.05}
      enableDamping={true}
      // Add resistance at the poles
      maxPolarAngle={Math.PI - 0.2}
      minPolarAngle={0.2}
    />
  );
};

export default React.memo(Orbit);