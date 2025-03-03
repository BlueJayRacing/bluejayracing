import React, { useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { computeFrameTransformMatrix } from '../frame/FrameDataManipulation';
import { CarConfig } from '../../ct_configs/carConfig';
import { ParsedMesh } from '../../ct_utils/gltfParser';

interface DrivetrainModelProps {
  wheelSpeedsInRPM: {
    frontLeft: number;
    frontRight: number;
    lockedRear: number;
  };
  steeringAngleInDegrees: number;
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

const DrivetrainModel: React.FC<DrivetrainModelProps> = ({
  wheelSpeedsInRPM,
  steeringAngleInDegrees,
  framePose,
  parsedComponents
}) => {
  const groupRef = useRef<THREE.Group>();
  const wheelGroupRefs = useRef<{ [key: string]: THREE.Group }>({});
  const wheelRotations = useRef<{ [key: string]: number }>({
    frontLeft: 0,
    frontRight: 0,
    rearLeft: 0,
    rearRight: 0
  });

  // Initialize wheels
  useEffect(() => {
    Object.entries(parsedComponents).forEach(([corner, meshes]) => {
      meshes.forEach(meshData => {
        const mesh = meshData.mesh;
        mesh.scale.setScalar(CarConfig.scale.frame);
        mesh.position.set(
          -.5*meshData.originalPosition[0],
          -.5*meshData.originalPosition[1],
          -.5*meshData.originalPosition[2]
        )
        
        // Use original position from parsed data as center of rotation
        if (wheelGroupRefs.current[corner]) {
          const group = wheelGroupRefs.current[corner];
          group.position.set(
            1.5*meshData.originalPosition[0],
            1.5*meshData.originalPosition[1],
            1.5*meshData.originalPosition[2]
          );
          // group.position.set(
          //   0,
          //   0,
          //   0
          // );
        }
      });
    });
  }, [parsedComponents]);

  // Handle frame transform
  useEffect(() => {
    if (!groupRef.current) return;

    const frameMatrix = computeFrameTransformMatrix({
      position: framePose.position,
      rotation: framePose.rotation,
      scale: [1, 1, 1]
    });

    groupRef.current.matrix.copy(frameMatrix);
    groupRef.current.matrixAutoUpdate = false;
  }, [framePose]);

  // Handle wheel rotation and steering
  useFrame((state, deltaTime) => {
    Object.entries(wheelGroupRefs.current).forEach(([corner, group]) => {
      if (!group) return;

      // Get wheel speed
      const wheelSpeed = corner.includes('rear') ? wheelSpeedsInRPM.lockedRear :
                        corner === 'frontLeft' ? wheelSpeedsInRPM.frontLeft :
                        wheelSpeedsInRPM.frontRight;

      // Update accumulated rotation
      wheelRotations.current[corner] += (wheelSpeed * Math.PI * 2 * deltaTime) / 60;

      // Create rotation matrix combining steering and rolling
      const euler = new THREE.Euler(0, 0, 0, 'YXZ');
      
      // Apply steering for front wheels
      if (corner.includes('front')) {
        euler.y = (steeringAngleInDegrees * Math.PI) / 180;
      }
      
      // Apply rolling rotation
      euler.x = wheelRotations.current[corner];

      // Create transformation matrix
      const matrix = new THREE.Matrix4();
      matrix.makeRotationFromEuler(euler);
      
      // Set position from group's position (original wheel position)
      matrix.setPosition(group.position);

      // Apply transform
      group.matrix.copy(matrix);
      group.matrixAutoUpdate = false;
    });
  });

  return (
    <group ref={groupRef}>
      {Object.entries(parsedComponents).map(([corner, meshes]) => (
        <group 
          key={`wheel-group-${corner}`}
          ref={el => {
            if (el) wheelGroupRefs.current[corner] = el;
          }}
          matrixAutoUpdate={false}
        >
          {meshes.map((meshData, meshIndex) => (
            <primitive
              key={`wheel-mesh-${corner}-${meshIndex}`}
              object={meshData.mesh}
            />
          ))}
        </group>
      ))}
    </group>
  );
};

export default DrivetrainModel;