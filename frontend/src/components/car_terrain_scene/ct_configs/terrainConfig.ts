// File: src/components/car_terrain_scene/ct_configs/terrainConfig.ts

/**
 * Configuration interface for the terrain component.
 * All measurements are in meters unless otherwise specified.
 */

/** Represents a 3D point in space */
export type Point3D = [number, number, number];

/** Quaternion representation for rotation */
export type Quaternion = [number, number, number, number];

/** Base terrain configuration interface */
export interface TerrainConfig {
  /** Uniform scale factor for the entire terrain */
  scale: number;
  
  /** Offset position from origin [x, y, z] in meters */
  offset: Point3D;
  
  /** Rotation as quaternion [x, y, z, w] */
  angle: Quaternion;
  
  /** Origin point for the terrain [x, y, z] in meters */
  origin: Point3D;
  
  /** Array of 3D points defining the track path */
  trackPoints: Point3D[];
}

/**
 * Default configuration values for the terrain
 * These values are derived from the previous implementation
 * and provide a reasonable starting point for most scenes
 */
export const DEFAULT_TERRAIN_CONFIG: TerrainConfig = {
  scale: 1.0,
  offset: [0, -2, 0],
  angle: [0, 0, 0, 1], // No rotation (identity quaternion)
  origin: [0, 0, 0],
  trackPoints: [
    [0, 0, 0],
    [10, 0, 0],
    [20, 0, 10],
    [30, 0, 20],
    [40, 0, 20],
    [50, 0, 10],
    [60, 0, 0]
  ]
};

/**
 * Helper function to create a terrain configuration with custom values
 * while maintaining default values for unspecified properties
 */
export function createTerrainConfig(partial: Partial<TerrainConfig>): TerrainConfig {
  return {
    ...DEFAULT_TERRAIN_CONFIG,
    ...partial
  };
}

/**
 * Utility function to validate a terrain configuration
 * Returns true if the configuration is valid, false otherwise
 */
export function isValidTerrainConfig(config: TerrainConfig): boolean {
  // Check if scale is positive
  if (config.scale <= 0) return false;

  // Check if quaternion is normalized
  const [qx, qy, qz, qw] = config.angle;
  const quatLength = Math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (Math.abs(quatLength - 1) > 0.001) return false;

  // Check if trackPoints array is non-empty
  if (config.trackPoints.length === 0) return false;

  return true;
}

/**
 * Apply a transformation to all track points in a terrain configuration
 * Returns a new TerrainConfig with transformed points
 */
export function transformTerrainConfig(
  config: TerrainConfig,
  transform: (point: Point3D) => Point3D
): TerrainConfig {
  return {
    ...config,
    trackPoints: config.trackPoints.map(transform),
    origin: transform(config.origin)
  };
}

export default {
  DEFAULT_TERRAIN_CONFIG,
  createTerrainConfig,
  isValidTerrainConfig,
  transformTerrainConfig
};