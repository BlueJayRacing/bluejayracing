// File: src/pages/HomePage.tsx

import React, { useState, useRef, useEffect } from 'react';
import CarTerrainScene from '../components/car_terrain_scene/CarTerrainScene';
import { CarState, DEFAULT_CAR_STATE } from '../components/car_terrain_scene/ct_configs/carConfig';
import { TerrainConfig, DEFAULT_TERRAIN_CONFIG } from '../components/car_terrain_scene/ct_configs/terrainConfig';
import { CameraConfig, DEFAULT_CAMERA_CONFIG } from '../components/car_terrain_scene/ct_configs/cameraConfig';
import { eulerToQuaternion } from '../components/car_terrain_scene/ct_utils/transformationUtils';

const HomePage: React.FC = () => {
  const [carState, setCarState] = useState<CarState>(DEFAULT_CAR_STATE);
  const [terrainConfig] = useState<TerrainConfig>(DEFAULT_TERRAIN_CONFIG);
  const [isAnimating, setIsAnimating] = useState(false);
  const [radius, setRadius] = useState(5); // Circle radius in meters
  const [speed, setSpeed] = useState(1); // Animation speed multiplier
  const lastTimeRef = useRef<number>(0);
  const animationRef = useRef<number>();
  
  const cameraConfigRef = useRef<CameraConfig>({
    ...DEFAULT_CAMERA_CONFIG,
    transitionToSetPoint: () => {},
    getCurrentSetPoint: () => 0,
    isTransitioning: () => false
  });

  const modelPaths = {
    car: '/models/vehicle.glb',
    terrain: '/models/penn2_rec.glb'
  };

  // Animation loop
  useEffect(() => {
    if (!isAnimating) {
      lastTimeRef.current = 0;
      return;
    }

    const animate = (currentTime: number) => {
      if (!lastTimeRef.current) {
        lastTimeRef.current = currentTime;
      }

      const deltaTime = (currentTime - lastTimeRef.current) / 1000; // Convert to seconds
      const elapsedTime = currentTime / 1000;
      
      // Calculate circular path
      const angularSpeed = speed * 0.5; // Reduced for more natural movement
      const angle = elapsedTime * angularSpeed;
      const x = Math.cos(angle) * radius;
      const z = Math.sin(angle) * radius;
      
      // Calculate steering angle - point towards circle center
      const steeringAngle = Math.atan2(-x, -z) * (180 / Math.PI);
      
      // Calculate wheel speeds based on position in circle
      const baseSpeed = 30 * speed;
      const innerWheelMultiplier = 0.8; // Inner wheel moves slower in turns
      const outerWheelMultiplier = 1.2; // Outer wheel moves faster in turns
      const steeringRatio = Math.abs(steeringAngle) / 45; // Normalize to max steering angle
      
      // Calculate wheel speeds with differential effect
      const frontLeftSpeed = baseSpeed * (steeringAngle > 0 ? innerWheelMultiplier : outerWheelMultiplier) * (1 - steeringRatio * 0.2);
      const frontRightSpeed = baseSpeed * (steeringAngle > 0 ? outerWheelMultiplier : innerWheelMultiplier) * (1 - steeringRatio * 0.2);
      
      // Calculate suspension movement
      const suspensionFreq = 2 * speed;
      const baseAmplitude = 3; // Base amplitude in inches
      const phaseOffset = Math.PI / 2; // Quarter cycle offset between corners
      
      // Add position-dependent suspension compression
      const bankingAngle = Math.atan2(speed * speed, radius) * 0.5; // Simple banking calculation
      const bankingOffset = Math.sin(bankingAngle) * 2; // Convert banking to suspension offset
      
      setCarState(prev => ({
        ...prev,
        framePose: {
          position: [x, 0.15, z],
          rotation: eulerToQuaternion(
            bankingAngle, // Roll due to banking
            -angle, // Yaw to follow circle
            0 // No pitch
          )
        },
        steeringAngleInDegrees: steeringAngle,
        wheelSpeedsInRPM: {
          frontLeft: frontLeftSpeed,
          frontRight: frontRightSpeed,
          lockedRear: baseSpeed
        },
        suspensionLengthsInInches: {
          frontLeft: Math.sin(elapsedTime * suspensionFreq) * baseAmplitude + 
                    (steeringAngle > 0 ? bankingOffset : -bankingOffset),
          frontRight: Math.sin(elapsedTime * suspensionFreq + phaseOffset) * baseAmplitude + 
                     (steeringAngle > 0 ? -bankingOffset : bankingOffset),
          rearLeft: Math.sin(elapsedTime * suspensionFreq + phaseOffset * 2) * baseAmplitude + 
                   (steeringAngle > 0 ? bankingOffset : -bankingOffset),
          rearRight: Math.sin(elapsedTime * suspensionFreq + phaseOffset * 3) * baseAmplitude + 
                    (steeringAngle > 0 ? -bankingOffset : bankingOffset)
        }
      }));

      lastTimeRef.current = currentTime;
      animationRef.current = requestAnimationFrame(animate);
    };

    animationRef.current = requestAnimationFrame(animate);

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [isAnimating, radius, speed]);

  // Manual control handlers
  const handleManualControl = (key: string, value: number) => {
    setCarState(prev => {
      const newState = { ...prev };
      const keys = key.split('.');
      let target = newState as any;
      
      for (let i = 0; i < keys.length - 1; i++) {
        target = target[keys[i]];
      }
      
      target[keys[keys.length - 1]] = value;
      return newState;
    });
  };

  return (
    <div style={{ 
      position: 'fixed',
      top: 0,
      left: 0,
      width: '100vw',
      height: '100vh',
      margin: 0,
      padding: 0,
      overflow: 'hidden'
    }}>
      <div className="absolute top-5 left-5 z-10 bg-black/70 p-5 rounded-lg text-white">
        <h3 className="text-xl font-bold mb-4">Animation Controls</h3>
        <div className="space-y-4">
          <button 
            onClick={() => setIsAnimating(!isAnimating)}
            className={`px-4 py-2 rounded ${
              isAnimating ? 'bg-red-500' : 'bg-green-500'
            } hover:opacity-90 transition-opacity`}
          >
            {isAnimating ? 'Stop' : 'Start'} Animation
          </button>
          
          <div className="space-y-2">
            <label className="block">
              Circle Radius: {radius}m
              <input 
                type="range" 
                min="1" 
                max="10" 
                value={radius} 
                onChange={(e) => setRadius(Number(e.target.value))}
                className="w-full"
              />
            </label>
          </div>

          <div className="space-y-2">
            <label className="block">
              Speed: {speed.toFixed(1)}x
              <input 
                type="range" 
                min="0.1" 
                max="2" 
                step="0.1" 
                value={speed} 
                onChange={(e) => setSpeed(Number(e.target.value))}
                className="w-full"
              />
            </label>
          </div>
        </div>

        <h3 className="text-xl font-bold mt-6 mb-4">Manual Controls</h3>
        <div className="space-y-4">
          <label className="block">
            Steering Angle
            <input 
              type="range" 
              min="-45" 
              max="45" 
              value={carState.steeringAngleInDegrees} 
              onChange={(e) => handleManualControl('steeringAngleInDegrees', Number(e.target.value))}
              disabled={isAnimating}
              className="w-full"
            />
          </label>
        </div>
      </div>

      <CarTerrainScene
        carState={carState}
        terrainConfig={terrainConfig}
        cameraConfigRef={cameraConfigRef}
        modelPaths={modelPaths}
        style={{
          width: '100%',
          height: '100%'
        }}
      />
    </div>
  );
};

export default HomePage;