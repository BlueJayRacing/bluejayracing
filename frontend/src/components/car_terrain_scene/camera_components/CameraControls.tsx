import React, { useEffect, useState, useRef } from 'react';
import { useThree, useFrame } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import { Vector3, Quaternion } from 'three';

interface CameraSetPoint {
  position: [number, number, number];
  target: [number, number, number];
}

interface CameraConfig {
  setPoints: CameraSetPoint[];
}

interface CameraControlsProps {
  cameraConfigRef: React.RefObject<CameraConfig>;
  transitionDuration?: number; // Duration in seconds
  enableInteraction?: boolean;
}

const CameraControls: React.FC<CameraControlsProps> = ({
  cameraConfigRef,
  transitionDuration = 1.0,
  enableInteraction = true,
}) => {
  const { camera, scene } = useThree();
  const orbitControlsRef = useRef();
  const [isTransitioning, setIsTransitioning] = useState(false);
  const [currentSetPoint, setCurrentSetPoint] = useState<number>(0);
  
  // Animation state
  const startPos = useRef(new Vector3());
  const endPos = useRef(new Vector3());
  const startTarget = useRef(new Vector3());
  const endTarget = useRef(new Vector3());
  const animationProgress = useRef(0);

  const transitionToSetPoint = (index: number) => {
    if (!cameraConfigRef.current?.setPoints?.[index]) return;
    
    const targetSetPoint = cameraConfigRef.current.setPoints[index];
    startPos.current.copy(camera.position);
    endPos.current.set(...targetSetPoint.position);
    
    startTarget.current.copy(orbitControlsRef.current.target);
    endTarget.current.set(...targetSetPoint.target);
    
    animationProgress.current = 0;
    setIsTransitioning(true);
    setCurrentSetPoint(index);
  };

  // Smooth easing function
  const easeInOutCubic = (t: number): number => {
    return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
  };

  useFrame((state, delta) => {
    if (isTransitioning) {
      animationProgress.current += delta / transitionDuration;
      
      if (animationProgress.current >= 1.0) {
        setIsTransitioning(false);
        animationProgress.current = 1.0;
      }

      const t = easeInOutCubic(animationProgress.current);
      
      // Interpolate position
      camera.position.lerpVectors(startPos.current, endPos.current, t);
      
      // Interpolate target
      orbitControlsRef.current.target.lerpVectors(startTarget.current, endTarget.current, t);
    }
  });

  // Method to expose camera control to parent components
  React.useImperativeHandle(cameraConfigRef, () => ({
    transitionToSetPoint,
    getCurrentSetPoint: () => currentSetPoint,
    isTransitioning: () => isTransitioning
  }));

  return (
    <>
      <OrbitControls
        ref={orbitControlsRef}
        enablePan={enableInteraction}
        enableZoom={enableInteraction}
        enableRotate={enableInteraction}
        enabled={!isTransitioning}
        makeDefault
      />
    </>
  );
};

export default CameraControls;