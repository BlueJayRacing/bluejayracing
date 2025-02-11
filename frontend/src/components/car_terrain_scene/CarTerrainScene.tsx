// File: src/components/car_terrain_scene/CarTerrainScene.tsx

import React, { useMemo, Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { useGLTF } from '@react-three/drei';
import * as THREE from 'three';
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import { parseCarGLTF, parseTerrainGLTF } from './ct_utils/gltfParser';

import { CarState } from './ct_configs/carConfig';
import { TerrainConfig } from './ct_configs/terrainConfig';
import { CameraConfig } from './ct_configs/cameraConfig';

import CameraControls from './camera_components/CameraControls';
import SuspensionModel from './car_components/suspension/SuspensionModel';
import FrameModel from './car_components/frame/FrameModel';
import DrivetrainModel from './car_components/drivetrain/DrivetrainModel';
import TerrainModel from './terrain_components/TerrainModel';

// Configure DRACO loader
const dracoLoader = new DRACOLoader();
dracoLoader.setDecoderPath('https://www.gstatic.com/draco/versioned/decoders/1.5.6/');
dracoLoader.preload();

// Configure GLTF loader
const gltfLoader = new GLTFLoader();
gltfLoader.setDRACOLoader(dracoLoader);

export interface CarTerrainSceneProps {
  carState: CarState;
  terrainConfig: TerrainConfig;
  cameraConfigRef: React.RefObject<CameraConfig>;
  modelPaths?: {
    car: string;
    terrain: string;
  };
  className?: string;
}

const DEFAULT_MODEL_PATHS = {
  car: '/models/vehicle.glb',
  terrain: '/models/terrain.glb'
};

const CarTerrainScene: React.FC<CarTerrainSceneProps> = ({
  carState,
  terrainConfig,
  cameraConfigRef,
  modelPaths = DEFAULT_MODEL_PATHS,
  className = ''
}) => {
  // Load car model with DRACO compression
  const { scene: carScene } = useGLTF(modelPaths.car, true, true, (loader) => {
    if (loader instanceof GLTFLoader) {
      loader.setDRACOLoader(dracoLoader);
    }
  });
  
  // Parse car and terrain components using gltfParser utility
  const parsedCarComponents = useMemo(() => {
    return parseCarGLTF(carScene);
  }, [carScene]);


  // Scene lighting setup
  const Lights = () => (
    <>
      <ambientLight intensity={0.5} />
      <directionalLight 
        position={[10, 10, 5]} 
        intensity={1}
        castShadow
        shadow-mapSize={[2048, 2048]}
      />
      <pointLight position={[0, 5, 0]} intensity={0.5} />
    </>
  );

  return (
    <div style={{
      width: '100%',
      height: '100%',
      position: 'relative',
      overflow: 'hidden'
    }}>
      <Canvas
        shadows
        camera={{ position: [5, 5, 5], fov: 60 }}
        style={{
          width: '100%',
          height: '100%',
          display: 'block'
        }}
        gl={{ 
          antialias: true,
          powerPreference: "high-performance"
        }}
      >
        <Suspense fallback={null}>
          <Lights />
          
          <CameraControls 
            cameraConfigRef={cameraConfigRef}
            enableInteraction={true}
            transitionDuration={1.0}
          />

          <group name="car">
            <SuspensionModel
              suspensionLengthsInInches={carState.suspensionLengthsInInches}
              framePose={carState.framePose}
              parsedComponents={parsedCarComponents.suspension}
            />
            <FrameModel
              framePose={carState.framePose}
              parsedComponents={parsedCarComponents.frame}
            />
            <DrivetrainModel
              framePose={carState.framePose}
              wheelSpeedsInRPM={carState.wheelSpeedsInRPM}
              steeringAngleInDegrees={carState.steeringAngleInDegrees}
              suspensionLengthsInInches={carState.suspensionLengthsInInches}
              parsedComponents={parsedCarComponents.drivetrain}
            />
          </group>

          <TerrainModel 
            terrainConfig={terrainConfig}
            modelPath={modelPaths.terrain}
          />
        </Suspense>
      </Canvas>
    </div>
  );
};

// // Preload models
// useGLTF.preload(DEFAULT_MODEL_PATHS.car, true, true, (loader) => {
//   if (loader instanceof GLTFLoader) {
//     loader.setDRACOLoader(dracoLoader);
//   }
// });

// useGLTF.preload(DEFAULT_MODEL_PATHS.terrain, true, true, (loader) => {
//   if (loader instanceof GLTFLoader) {
//     loader.setDRACOLoader(dracoLoader);
//   }
// });

export default CarTerrainScene;