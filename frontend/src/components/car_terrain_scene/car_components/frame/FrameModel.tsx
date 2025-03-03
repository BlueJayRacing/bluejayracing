// File: src/components/car_terrain_scene/car_components/frame/FrameModel.tsx

import React, { useRef, useEffect } from 'react';
import * as THREE from 'three';
import { computeFrameTransformMatrix } from './FrameDataManipulation';
import { ParsedMesh } from '../../ct_utils/gltfParser';
import { CarConfig } from '../../ct_configs/carConfig';

interface FrameModelProps {
  framePose: {
    position: [number, number, number];
    rotation: [number, number, number, number];
  };
  parsedComponents: ParsedMesh[];
}

const FrameModel: React.FC<FrameModelProps> = ({ 
  framePose, 
  parsedComponents,
}) => {
  const group = useRef<THREE.Group>();
  // Apply scale to all meshes
  useEffect(() => {
    parsedComponents.forEach(meshData => {
      const mesh = meshData.mesh;
      const uniformScale = CarConfig.scale.frame;
      mesh.scale.setScalar(uniformScale);
      // Update materials
      const material = mesh.material as THREE.MeshStandardMaterial;
      if (material?.map) {
        material.needsUpdate = true;
      }
    });
  }, [parsedComponents]);

  // Update frame transform when pose changes
  useEffect(() => {
    if (!group.current) return;

    const transformMatrix = computeFrameTransformMatrix({
      position: framePose.position,
      rotation: framePose.rotation,
      scale: [1,1,1]
    });

    group.current.matrix.copy(transformMatrix);
    group.current.matrixAutoUpdate = false;
    group.current.matrixWorldNeedsUpdate = true;
  }, [framePose]);

  return (
    <group ref={group}>
      {parsedComponents.map((meshData, index) => (
        <primitive 
          key={`frame-${meshData.mesh.name}-${index}`} 
          object={meshData.mesh}
        />
      ))}
    </group>
  );
};

export default React.memo(FrameModel);