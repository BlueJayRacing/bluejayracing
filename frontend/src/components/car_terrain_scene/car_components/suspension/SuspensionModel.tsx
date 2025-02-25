// File: src/components/car_terrain_scene/car_components/suspension/SuspensionModel.tsx

import React, { useRef, useEffect } from 'react';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber';
import { computeFrameTransformMatrix } from '../frame/FrameDataManipulation';
import { CarConfig } from '../../ct_configs/carConfig';
import { ParsedMesh } from '../../ct_utils/gltfParser';

interface SuspensionModelProps {
  suspensionLengthsInInches: {
    frontLeft: number;
    frontRight: number;
    rearLeft: number;
    rearRight: number;
  };
  framePose: {
    position: [number, number, number];
    rotation: [number, number, number, number];
  };
  parsedComponents: {
    frontLeft: ParsedMesh[];
    frontRight: ParsedMesh[];
    rearLeft: ParsedMesh[];
    rearRight: ParsedMesh[];
  };
}

const SuspensionModel: React.FC<SuspensionModelProps> = ({
  suspensionLengthsInInches,
  framePose,
  parsedComponents
}) => {
  const groupRef = useRef<THREE.Group>();
  const cornerGroupRefs = useRef<{ [key: string]: THREE.Group }>({});

  // Apply initial scale
  useEffect(() => {
    Object.entries(parsedComponents).forEach(([corner, meshes]) => {
      meshes.forEach(meshData => {
        const mesh = meshData.mesh;
        const uniformScale = CarConfig.scale.frame;
        mesh.scale.setScalar(uniformScale);
      });
    });
  }, [parsedComponents]);

  // Handle frame transform
  useEffect(() => {
    if (!groupRef.current) return;

    const transformMatrix = computeFrameTransformMatrix({
      position: framePose.position,
      rotation: framePose.rotation,
      scale: [1, 1, 1]
    });

    groupRef.current.matrix.copy(transformMatrix);
    groupRef.current.matrixAutoUpdate = false;
  }, [framePose]);

  // Handle suspension length changes
  useEffect(() => {
    Object.entries(suspensionLengthsInInches).forEach(([corner, length]) => {
      const cornerGroup = cornerGroupRefs.current[corner];
      if (!cornerGroup) return;

      // Convert inches to meters for the position offset
      const metersOffset = length * 0.0254; // 1 inch = 0.0254 meters
      
      // Get the base mount point for this corner
      const mountPoint = CarConfig.suspension.mountPoints[corner];
      if (!mountPoint) return;

      // Create position vector with vertical offset
      const position = new THREE.Vector3(
        0,
        0 + metersOffset, // Add the suspension travel
        0
      );

      // Apply the position to the corner group
      cornerGroup.position.copy(position);
    });
  }, [suspensionLengthsInInches]);

  return (
    <group ref={groupRef}>
      {Object.entries(parsedComponents).map(([corner, meshes]) => (
        <group 
          key={`suspension-group-${corner}`}
          ref={el => {
            if (el) cornerGroupRefs.current[corner] = el;
          }}
        >
          {meshes.map((meshData, meshIndex) => (
            <primitive
              key={`suspension-mesh-${corner}-${meshIndex}`}
              object={meshData.mesh}
            />
          ))}
        </group>
      ))}
    </group>
  );
};

export default SuspensionModel;