// File: src/components/car_terrain_scene/car_components/frame/FrameDataManipulation.ts

import { Vector3, Quaternion, Matrix4, Euler } from 'three';

/**
 * Interface for frame transformation parameters
 */
interface FrameTransformParams {
  position: [number, number, number];
  rotation: [number, number, number, number]; // Quaternion
  scale?: [number, number, number];
}

/**
 * Computes the final frame transformation matrix combining position, rotation, and scale
 */
export function computeFrameTransformMatrix({ position, rotation, scale = [1, 1, 1] }: FrameTransformParams): Matrix4 {
  const matrix = new Matrix4();
  const pos = new Vector3(...position);
  const quat = new Quaternion(...rotation);
  const scaleVec = new Vector3(...scale);
  
  return matrix.compose(pos, quat, scaleVec);
}

/**
 * Applies an offset to the frame's base position
 */
export function computeFramePosition(
  basePosition: [number, number, number],
  offset: [number, number, number] = [0, 0, 0]
): [number, number, number] {
  return [
    basePosition[0] + offset[0],
    basePosition[1] + offset[1],
    basePosition[2] + offset[2]
  ];
}

/**
 * Converts Euler angles to quaternion for frame rotation
 */
export function computeFrameRotation(
  pitch: number,
  yaw: number,
  roll: number
): [number, number, number, number] {
  const euler = new Euler(pitch, yaw, roll, 'XYZ');
  const quaternion = new Quaternion();
  quaternion.setFromEuler(euler);
  
  return [quaternion.x, quaternion.y, quaternion.z, quaternion.w];
}

/**
 * Interpolates between two frame poses for smooth transitions
 */
export function interpolateFramePoses(
  start: FrameTransformParams,
  end: FrameTransformParams,
  t: number
): FrameTransformParams {
  const startPos = new Vector3(...start.position);
  const endPos = new Vector3(...end.position);
  const startQuat = new Quaternion(...start.rotation);
  const endQuat = new Quaternion(...end.rotation);
  
  const interpolatedPos = new Vector3();
  interpolatedPos.lerpVectors(startPos, endPos, t);
  
  const interpolatedQuat = new Quaternion();
  Quaternion.slerp(startQuat, endQuat, interpolatedQuat, t);
  
  return {
    position: [interpolatedPos.x, interpolatedPos.y, interpolatedPos.z],
    rotation: [interpolatedQuat.x, interpolatedQuat.y, interpolatedQuat.z, interpolatedQuat.w]
  };
}