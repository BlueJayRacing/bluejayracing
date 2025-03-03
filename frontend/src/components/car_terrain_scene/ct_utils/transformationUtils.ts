// File: src/components/car_terrain_scene/ct_utils/transformationUtils.ts

import * as THREE from 'three';
import { Vector3, Quaternion, Euler } from 'three';

// Unit conversion constants
const INCHES_TO_METERS = 0.0254;
const DEGREES_TO_RADIANS = Math.PI / 180;
const RPM_TO_RADS_PER_SEC = (2 * Math.PI) / 60;

/**
 * Converts inches to meters
 */
export function convertInchesToMeters(inches: number): number {
  return inches * INCHES_TO_METERS;
}

/**
 * Converts degrees to radians
 */
export function convertDegreesToRadians(degrees: number): number {
  return degrees * DEGREES_TO_RADIANS;
}

/**
 * Converts RPM to radians per second
 */
export function convertRPMToRadiansPerSecond(rpm: number): number {
  return rpm * RPM_TO_RADS_PER_SEC;
}

/**
 * Calculates relative position of a child object given parent position and offset
 */
export function calculateRelativePosition(
  parentPosition: [number, number, number],
  childOffset: [number, number, number]
): [number, number, number] {
  return [
    parentPosition[0] + childOffset[0],
    parentPosition[1] + childOffset[1],
    parentPosition[2] + childOffset[2]
  ];
}

/**
 * Creates a transformation matrix from position and quaternion
 */
export function createTransformationMatrix(
  position: [number, number, number],
  quaternion: [number, number, number, number]
): THREE.Matrix4 {
  const matrix = new THREE.Matrix4();
  const pos = new THREE.Vector3(...position);
  const quat = new THREE.Quaternion(...quaternion);
  
  return matrix.compose(pos, quat, new THREE.Vector3(1, 1, 1));
}

/**
 * Applies a transformation to a position vector
 */
export function applyTransformToPosition(
  position: [number, number, number],
  transformMatrix: THREE.Matrix4
): [number, number, number] {
  const vector = new THREE.Vector3(...position);
  vector.applyMatrix4(transformMatrix);
  return [vector.x, vector.y, vector.z];
}

/**
 * Interpolates between two positions with a given factor (0 to 1)
 */
export function interpolatePositions(
  start: [number, number, number],
  end: [number, number, number],
  factor: number
): [number, number, number] {
  return [
    start[0] + (end[0] - start[0]) * factor,
    start[1] + (end[1] - start[1]) * factor,
    start[2] + (end[2] - start[2]) * factor
  ];
}

/**
 * Creates a quaternion from Euler angles (in radians)
 */
export function eulerToQuaternion(
  x: number, 
  y: number, 
  z: number
): [number, number, number, number] {
  const euler = new THREE.Euler(x, y, z);
  const quaternion = new THREE.Quaternion();
  quaternion.setFromEuler(euler);
  return [quaternion.x, quaternion.y, quaternion.z, quaternion.w];
}

/**
 * Interpolates between two quaternions with a given factor (0 to 1)
 */
export function interpolateQuaternions(
  start: [number, number, number, number],
  end: [number, number, number, number],
  factor: number
): [number, number, number, number] {
  const q1 = new THREE.Quaternion(...start);
  const q2 = new THREE.Quaternion(...end);
  q1.slerp(q2, factor);
  return [q1.x, q1.y, q1.z, q1.w];
}

/**
 * Calculates suspension height in meters based on frame position and shock extension
 */
export function calculateSuspensionHeight(
  frameHeightInMeters: number,
  shockExtensionInInches: number,
  baselineHeightInMeters: number
): number {
  const extensionInMeters = convertInchesToMeters(shockExtensionInInches);
  return frameHeightInMeters + extensionInMeters + baselineHeightInMeters;
}

/**
 * Calculates wheel rotation matrix based on RPM and time delta
 */
export function calculateWheelRotation(
  rpmSpeed: number,
  deltaTimeSeconds: number
): THREE.Matrix4 {
  const radiansPerSecond = convertRPMToRadiansPerSecond(rpmSpeed);
  const rotationAngle = radiansPerSecond * deltaTimeSeconds;
  return new THREE.Matrix4().makeRotationX(rotationAngle);
}

/**
 * Combines multiple transformations into a single matrix
 */
export function combineTransformations(
  transforms: THREE.Matrix4[]
): THREE.Matrix4 {
  const result = new THREE.Matrix4();
  transforms.forEach(transform => {
    result.multiply(transform);
  });
  return result;
}

/**
 * Decomposes a transformation matrix into position, quaternion, and scale
 */
export function decomposeTransformationMatrix(
  matrix: THREE.Matrix4
): {
  position: [number, number, number];
  quaternion: [number, number, number, number];
  scale: [number, number, number];
} {
  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  const scale = new THREE.Vector3();
  
  matrix.decompose(position, quaternion, scale);
  
  return {
    position: [position.x, position.y, position.z],
    quaternion: [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
    scale: [scale.x, scale.y, scale.z]
  };
}


export function rotatePositionByQuaternion(position: [number, number, number], quaternion: Quaternion): Vector3 {
  const positionVector = new Vector3(position[0], position[1], position[2]);
  const positionQuaternion = new Quaternion(positionVector.x, positionVector.y, positionVector.z, 0);
  
  const conjugate = quaternion.clone().conjugate();
  const rotatedQuaternion = quaternion.clone()
    .multiply(positionQuaternion)
    .multiply(conjugate);
  
  return new Vector3(rotatedQuaternion.x, rotatedQuaternion.y, rotatedQuaternion.z);
}


export function addEulerToQuaternion(
  quaternion: Quaternion,
  roll = 0,
  pitch = 0,
  yaw = 0
): Quaternion {
  const eulerRotation = new Euler(roll, pitch, yaw, 'XYZ');
  const rotationQuaternion = new Quaternion();
  rotationQuaternion.setFromEuler(eulerRotation);
  
  const resultQuaternion = quaternion.clone();
  resultQuaternion.multiply(rotationQuaternion);
  
  return resultQuaternion;
}