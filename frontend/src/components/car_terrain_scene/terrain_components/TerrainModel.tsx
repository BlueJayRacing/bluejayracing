// File: src/components/car_terrain_scene/terrain_components/TerrainModel.tsx

import React, { useRef, useMemo } from 'react';
import { useLoader } from '@react-three/fiber';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import * as THREE from 'three';
import { TerrainConfig } from '../ct_configs/terrainConfig';

interface TerrainModelProps {
  terrainConfig: TerrainConfig;
  modelPath?: string;
}

const TerrainModel: React.FC<TerrainModelProps> = ({ 
  terrainConfig,
  // modelPath = '/models/penn2.glb'
  modelPath = '/models/mich_end_start.glb'
}) => {
  const groupRef = useRef<THREE.Group>(null);
  
  // Load and process the terrain model
  const result = useLoader(GLTFLoader, modelPath);
  
  // Process geometry with scale factor
  const processedGeometry = useMemo(() => {
    if (!result.scene.children[0]?.geometry) return null;
    
    const geometry = result.scene.children[0].geometry.clone();
    const scale = 0.02; // Base scale factor
    
    const positionAttr = geometry.attributes.position;
    for (let i = 0; i < positionAttr.count; i++) {
      positionAttr.setXYZ(
        i,
        positionAttr.getX(i) * scale,
        positionAttr.getY(i) * scale,
        positionAttr.getZ(i) * scale
      );
    }
    
    return geometry;
  }, [result]);

  // Apply terrain configuration transforms
  const terrainTransform = useMemo(() => {
    const matrix = new THREE.Matrix4();
    
    // Apply scale
    const scaleVector = new THREE.Vector3(
      terrainConfig.scale,
      terrainConfig.scale,
      terrainConfig.scale
    );
    
    // Apply rotation from quaternion
    const quaternion = new THREE.Quaternion(...terrainConfig.angle);
    
    // Apply position
    const position = new THREE.Vector3(...terrainConfig.offset);
    
    // Combine transforms
    matrix.compose(position, quaternion, scaleVector);
    
    return matrix;
  }, [terrainConfig]);

  if (!processedGeometry) return null;

  return (
    <group ref={groupRef} matrix={terrainTransform}>
      <primitive object={result.scene} geometry={processedGeometry} scale={ 0.02}/>
      
      {/* Track point visualization */}
      {terrainConfig.trackPoints.map((point, index) => (
        <mesh 
          key={`track-point-${index}`}
          position={new THREE.Vector3(...point)}
        >
          <sphereGeometry args={[0.01, 16, 16]} />
          <meshBasicMaterial color="red" />
        </mesh>
      ))}
    </group>
  );
};

export default TerrainModel;