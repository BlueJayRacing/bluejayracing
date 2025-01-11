// Orbit.tsx
import React, {FC} from 'react';
import {CameraPositionProps, CameraControlProps} from 'src/components/CameraControls/CameraControls';
import { OrbitControls } from '@react-three/drei';
import { useThree } from '@react-three/fiber';

export interface OrbitControlProps extends CameraControlProps {
  setIsGoingToFront: (value: boolean) => void;
  setIsGoingToBack: (value: boolean) => void;
  setIsGoingToLeft: (value: boolean) => void;
  setIsGoingToRight: (value: boolean) => void;
}

export const Orbit: FC<CameraPositionProps> = ({horizontalPosition, verticalPosition}) => {
  const { camera, gl } = useThree();
  
  return (
    <OrbitControls
      autoRotate={false}
      maxPolarAngle={verticalPosition}
      minPolarAngle={verticalPosition}
      minAzimuthAngle={horizontalPosition}
      maxAzimuthAngle={horizontalPosition}
      args={[camera, gl.domElement]}
      target={[35, -10, 0]}
      zoom0={0.5}
    />
  );
};