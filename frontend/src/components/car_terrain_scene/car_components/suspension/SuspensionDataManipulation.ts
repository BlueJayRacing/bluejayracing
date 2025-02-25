// File: src/components/car_terrain_scene/car_components/suspension/SuspensionDataManipulation.ts

import * as THREE from 'three';

interface SuspensionLengths {
  frontLeft: number;
  frontRight: number;
  rearLeft: number;
  rearRight: number;
}

interface FramePose {
  position: [number, number, number];
  rotation: [number, number, number, number];
}

interface SuspensionConfig {
  scale: number;
  frontThreshold: number;
  baseHeight: number;
  maxExtension: number;
  minExtension: number;
  mountPoints: {
    frontLeft: [number, number, number];
    frontRight: [number, number, number];
    rearLeft: [number, number, number];
    rearRight: [number, number, number];
  };
}

interface TransformResult {
  position: THREE.Vector3;
  rotation: {
    x: number;
    y: number;
    z: number;
  };
}

interface SuspensionTransforms {
  frontLeft: TransformResult;
  frontRight: TransformResult;
  rearLeft: TransformResult;
  rearRight: TransformResult;
}

/**
 * Converts suspension length from inches to meters
 */
const inchesToMeters = (inches: number): number => inches * 0.0254;

/**
 * Calculates the vertical offset based on suspension length
 */
const calculateVerticalOffset = (
  lengthInches: number,
  config: SuspensionConfig
): number => {
  const lengthMeters = inchesToMeters(lengthInches);
  const clampedLength = Math.max(
    config.minExtension,
    Math.min(config.maxExtension, lengthMeters)
  );
  return clampedLength - config.baseHeight;
};

/**
 * Applies frame pose transformation to a position
 */
const applyFrameTransform = (
  position: THREE.Vector3,
  framePose: FramePose
): THREE.Vector3 => {
  const framePosition = new THREE.Vector3(...framePose.position);
  const frameQuaternion = new THREE.Quaternion(...framePose.rotation);
  
  // Create a matrix from the frame's pose
  const frameMatrix = new THREE.Matrix4()
    .compose(
      framePosition,
      frameQuaternion,
      new THREE.Vector3(1, 1, 1)
    );
  
  // Apply the transformation
  return position.applyMatrix4(frameMatrix);
};

/**
 * Computes suspension transforms based on lengths and frame pose
 */
export const computeSuspensionTransforms = ({
  lengths,
  framePose,
  config
}: {
  lengths: SuspensionLengths;
  framePose: FramePose;
  config: SuspensionConfig;
}): SuspensionTransforms => {
  const computeTransform = (
    length: number,
    mountPoint: [number, number, number]
  ): TransformResult => {
    const verticalOffset = calculateVerticalOffset(length, config);
    const basePosition = new THREE.Vector3(...mountPoint);
    
    // Apply vertical offset
    basePosition.y += verticalOffset;
    
    // Apply frame transformation
    const transformedPosition = applyFrameTransform(basePosition, framePose);
    
    // Calculate rotation based on the frame pose and vertical offset
    const frameQuaternion = new THREE.Quaternion(...framePose.rotation);
    const euler = new THREE.Euler().setFromQuaternion(frameQuaternion);
    
    return {
      position: transformedPosition,
      rotation: {
        x: euler.x,
        y: euler.y,
        z: euler.z
      }
    };
  };

  return {
    frontLeft: computeTransform(lengths.frontLeft, config.mountPoints.frontLeft),
    frontRight: computeTransform(lengths.frontRight, config.mountPoints.frontRight),
    rearLeft: computeTransform(lengths.rearLeft, config.mountPoints.rearLeft),
    rearRight: computeTransform(lengths.rearRight, config.mountPoints.rearRight)
  };
};