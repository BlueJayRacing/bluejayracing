// File: src/components/car_terrain_scene/ct_configs/cameraConfig.ts

import { Vector3 } from 'three';

/**
 * Represents a single camera position and target point in 3D space
 */
export interface CameraSetPoint {
  /** Camera position in world space [x, y, z] */
  position: [number, number, number];
  /** Point the camera looks at in world space [x, y, z] */
  target: [number, number, number];
  /** Optional transition duration in milliseconds */
  transitionDuration?: number;
  /** Optional easing function name for the transition */
  easing?: 'linear' | 'easeInOut' | 'easeIn' | 'easeOut';
}

/**
 * Configuration for camera behavior and predefined viewpoints
 */
export interface CameraConfig {
  /** Array of predefined camera positions and targets */
  setPoints: CameraSetPoint[];
  /** Default transition duration in milliseconds if not specified in setPoint */
  defaultTransitionDuration: number;
  /** Default easing function if not specified in setPoint */
  defaultEasing: 'linear' | 'easeInOut' | 'easeIn' | 'easeOut';
  /** Minimum and maximum zoom limits */
  zoomLimits: {
    min: number;
    max: number;
  };
  /** Camera orbit constraints */
  orbitConstraints: {
    /** Minimum and maximum polar angle (vertical rotation) in radians */
    polarAngle: {
      min: number;
      max: number;
    };
    /** Minimum and maximum azimuth angle (horizontal rotation) in radians */
    azimuthAngle: {
      min: number;
      max: number;
    };
  };
}

/**
 * Helper function to create a camera set point
 */
export function createSetPoint(
  position: [number, number, number],
  target: [number, number, number],
  options?: {
    transitionDuration?: number;
    easing?: CameraSetPoint['easing'];
  }
): CameraSetPoint {
  return {
    position,
    target,
    ...options,
  };
}

/**
 * Default camera configurations for common viewing angles
 */
export const DEFAULT_CAMERA_CONFIG: CameraConfig = {
  setPoints: [
    // Front view
    createSetPoint(
      [0, 1.5, 2], // Position slightly elevated and in front
      [0, 0, 0],   // Looking at origin
      { transitionDuration: 1000, easing: 'easeInOut' }
    ),
    // Side view (right)
    createSetPoint(
      [2, 1.5, 0],
      [0, 0, 0],
      { transitionDuration: 1000, easing: 'easeInOut' }
    ),
    // Top view
    createSetPoint(
      [0, 3, 0],
      [0, 0, 0],
      { transitionDuration: 1200, easing: 'easeInOut' }
    ),
    // 3/4 view (isometric-like)
    createSetPoint(
      [1.5, 1.5, 1.5],
      [0, 0, 0],
      { transitionDuration: 1000, easing: 'easeInOut' }
    ),
  ],
  defaultTransitionDuration: 1000,
  defaultEasing: 'easeInOut',
  zoomLimits: {
    min: 1,
    max: 10
  },
  orbitConstraints: {
    polarAngle: {
      min: 0,           // Straight down
      max: Math.PI / 2  // Horizontal view
    },
    azimuthAngle: {
      min: -Math.PI,    // Full rotation allowed
      max: Math.PI
    }
  }
};

/**
 * Validates a camera configuration object
 * @returns true if valid, throws Error if invalid
 */
export function validateCameraConfig(config: CameraConfig): boolean {
  if (!Array.isArray(config.setPoints) || config.setPoints.length === 0) {
    throw new Error('Camera config must have at least one set point');
  }

  // Validate each set point
  config.setPoints.forEach((point, index) => {
    if (!Array.isArray(point.position) || point.position.length !== 3 ||
        !Array.isArray(point.target) || point.target.length !== 3) {
      throw new Error(`Invalid position or target in set point ${index}`);
    }

    if (point.transitionDuration !== undefined && point.transitionDuration < 0) {
      throw new Error(`Invalid transition duration in set point ${index}`);
    }
  });

  // Validate zoom limits
  if (config.zoomLimits.min >= config.zoomLimits.max) {
    throw new Error('Invalid zoom limits: min must be less than max');
  }

  // Validate orbit constraints
  const { polarAngle, azimuthAngle } = config.orbitConstraints;
  if (polarAngle.min >= polarAngle.max) {
    throw new Error('Invalid polar angle constraints: min must be less than max');
  }
  if (azimuthAngle.min >= azimuthAngle.max) {
    throw new Error('Invalid azimuth angle constraints: min must be less than max');
  }

  return true;
}