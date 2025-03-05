// File: src/components/car_terrain_scene/car_components/drivetrain/DrivetrainDataManipulation.ts

import * as THREE from 'three';
import { CarConfig } from '../../ct_configs/carConfig';

const INCHES_TO_METERS = 0.0254;
const DEGREES_TO_RADIANS = Math.PI / 180;

/**
 * Calculates wheel rotation angle based on RPM and elapsed time
 */
export function calculateWheelRotation(rpm: number, deltaTime: number): number {
  const radiansPerSecond = (rpm * 2 * Math.PI) / 60;
  return radiansPerSecond * deltaTime;
}

/**
 * Calculates Ackermann steering angle for a wheel
 */
export function calculateAckermannSteeringAngle(
  centralSteeringAngle: number,
  wheelbase: number,
  trackWidth: number,
  isLeftWheel: boolean,
  isFrontWheel: boolean
): number {
  if (!isFrontWheel) return 0; // Rear wheels don't steer
  
  const steeringAngleRadians = centralSteeringAngle * DEGREES_TO_RADIANS;
  
  if (Math.abs(steeringAngleRadians) < 0.001) return 0;

  const turnRadius = wheelbase / Math.tan(steeringAngleRadians);
  const halfTrack = trackWidth / 2;

  // Calculate inner and outer wheel angles
  const innerAngle = Math.atan(wheelbase / (turnRadius - halfTrack));
  const outerAngle = Math.atan(wheelbase / (turnRadius + halfTrack));

  // Determine which angle to use based on steering direction and wheel side
  let wheelAngle: number;
  if (steeringAngleRadians > 0) { // Turning right
    wheelAngle = isLeftWheel ? outerAngle : innerAngle;
  } else { // Turning left
    wheelAngle = isLeftWheel ? innerAngle : outerAngle;
  }

  return wheelAngle / DEGREES_TO_RADIANS; // Convert back to degrees
}

/**
 * Calculates wheel position relative to suspension mount point
 */
export function calculateWheelPosition(
  wheelIndex: number,
  suspensionLength: number,
  steeringAngle: number
): THREE.Vector3 {
  const isLeft = wheelIndex === 0 || wheelIndex === 2;
  const isFront = wheelIndex < 2;
  
  // Base wheel offsets relative to suspension mount
  const xOffset = (isLeft ? -1 : 1) * CarConfig.wheels.offsetFromMount.x;
  const yOffset = -suspensionLength * INCHES_TO_METERS;
  const zOffset = (isFront ? 1 : -1) * CarConfig.wheels.offsetFromMount.z;

  // For front wheels, apply steering offset
  if (isFront) {
    const steeringRad = steeringAngle * DEGREES_TO_RADIANS;
    const cos = Math.cos(steeringRad);
    const sin = Math.sin(steeringRad);
    
    // Apply steering transformation
    return new THREE.Vector3(
      xOffset * cos - zOffset * sin,
      yOffset,
      xOffset * sin + zOffset * cos
    );
  }

  return new THREE.Vector3(xOffset, yOffset, zOffset);
}

/**
 * Calculates wheel speeds based on steering angle and base speed
 */
export function calculateWheelSpeeds(
  baseSpeedRPM: number,
  steeringAngle: number,
  wheelbase: number,
  trackWidth: number
): {
  frontLeft: number;
  frontRight: number;
  rearLeft: number;
  rearRight: number;
} {
  const steeringRad = steeringAngle * DEGREES_TO_RADIANS;
  
  if (Math.abs(steeringRad) < 0.001) {
    return {
      frontLeft: baseSpeedRPM,
      frontRight: baseSpeedRPM,
      rearLeft: baseSpeedRPM,
      rearRight: baseSpeedRPM
    };
  }

  const turnRadius = wheelbase / Math.tan(Math.abs(steeringRad));
  const halfTrack = trackWidth / 2;

  // Calculate radius for each wheel
  const innerFrontRadius = turnRadius - halfTrack;
  const outerFrontRadius = turnRadius + halfTrack;
  const innerRearRadius = Math.sqrt(innerFrontRadius ** 2 + wheelbase ** 2);
  const outerRearRadius = Math.sqrt(outerFrontRadius ** 2 + wheelbase ** 2);

  // Calculate speed ratios based on turn radius
  const innerFrontRatio = innerFrontRadius / turnRadius;
  const outerFrontRatio = outerFrontRadius / turnRadius;
  const innerRearRatio = innerRearRadius / turnRadius;
  const outerRearRatio = outerRearRadius / turnRadius;

  if (steeringRad > 0) { // Turning right
    return {
      frontLeft: baseSpeedRPM * outerFrontRatio,
      frontRight: baseSpeedRPM * innerFrontRatio,
      rearLeft: baseSpeedRPM * outerRearRatio,
      rearRight: baseSpeedRPM * innerRearRatio
    };
  } else { // Turning left
    return {
      frontLeft: baseSpeedRPM * innerFrontRatio,
      frontRight: baseSpeedRPM * outerFrontRatio,
      rearLeft: baseSpeedRPM * innerRearRatio,
      rearRight: baseSpeedRPM * outerRearRatio
    };
  }
}