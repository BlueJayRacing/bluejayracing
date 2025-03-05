// File: src/components/car_terrain_scene/ct_configs/carConfig.ts

/**
 * Core interface defining the complete state of a car in the scene
 */
export interface CarState {
  suspensionLengthsInInches: {
    frontLeft: number;
    frontRight: number;
    rearLeft: number;
    rearRight: number;
  };
  wheelSpeedsInRPM: {
    frontLeft: number;
    frontRight: number;
    lockedRear: number;
  };
  steeringAngleInDegrees: number;
  framePose: {
    position: [number, number, number];  // r3 vector
    rotation: [number, number, number, number]; // Quaternion
  };
}

/**
 * Configuration for mapping GLTF nodes to car components
 */
export interface GLTFNodeMapping {
  nodeName: string;
  type: 'suspension' | 'frame' | 'drivetrain';
  subtype?: string;
  defaultTransform?: {
    position?: [number, number, number];
    rotation?: [number, number, number, number];
    scale?: [number, number, number];
  };
}

/**
 * Helper type for component-specific transforms
 */
export interface ComponentTransform {
  position?: [number, number, number];
  rotation?: [number, number, number, number];
  scale?: [number, number, number];
}

/**
 * Main car configuration interface
 */
export interface CarConfig {
  modelPath: string;
  nodeMappings: GLTFNodeMapping[];
  scale: {
    global: number;
    frame: number;
    suspension: number;
    drivetrain: number;
  };
  suspension: {
    scale: number;
    frontThreshold: number;
    baseHeight: number;
    maxExtension: number;
    minExtension: number;
    maxExtensionInches: number;
    minExtensionInches: number;
    dampingCoefficient: number;
    mountPoints: {
      "frontLeft": [number, number, number];
      "frontRight": [number, number, number];
      "rearLeft": [number, number, number];
      "rearRight": [number, number, number];
    };
  };
  steering: {
    maxAngleDegrees: number;
    minAngleDegrees: number;
  };
  wheels: {
    wheelbase: number;
    trackWidth: number;
    offsetFromMount: {
      x: number;
      z: number;
    }
    maxRPM: number;
    radiusInches: number;
  };
  frame: {
    scale: [number,number,number]
  }
}

/**
 * Default car state values
 */
export const DEFAULT_CAR_STATE: CarState = {
  suspensionLengthsInInches: {
    frontLeft: 0,
    frontRight: 0,
    rearLeft: 0,
    rearRight: 0
  },
  wheelSpeedsInRPM: {
    frontLeft: 0,
    frontRight: 0,
    lockedRear: 0
  },
  steeringAngleInDegrees: 0,
  framePose: {
    position: [0, 0, 0],  // Slight elevation to prevent ground intersection
    rotation: [0, 0, 0, 1]   // Identity quaternion
  }
};

/**
 * Default car configuration
 */
export const DEFAULT_CAR_CONFIG: CarConfig = {
  modelPath: '/models/vehicle.glb',
  nodeMappings: [
    {
      nodeName: 'suspension_front_left',
      type: 'suspension',
      subtype: 'frontLeft',
      defaultTransform: {
        position: [-0.15851670762394457, -0.040562102177363245, 0.4257602923760555],
        scale: [0.01, 0.01, 0.01]
      }
    },
    {
      nodeName: 'suspension_front_right',
      type: 'suspension',
      subtype: 'frontRight',
      defaultTransform: {
        position: [0.1662480202628072, -0.040509979737193134, 0.4261460202628072],
        scale: [0.01, 0.01, 0.01]
      }
    },
    {
      nodeName: 'suspension_rear_left',
      type: 'suspension',
      subtype: 'rearLeft',
      defaultTransform: {
        position: [-0.1590329797371928, -0.040509979737193134, 0.07445702026280687],
        scale: [0.01, 0.01, 0.01]
      }
    },
    {
      nodeName: 'suspension_rear_right',
      type: 'suspension',
      subtype: 'rearRight',
      defaultTransform: {
        position: [0.16646525699485823, -0.0404873430051421, 0.0749262569948579],
        scale: [0.01, 0.01, 0.01]
      }
    },
    {
      nodeName: 'frame',
      type: 'frame',
      defaultTransform: {
        rotation: [0, -Math.PI/2, 0, 0 ],
        scale: [0.01, 0.01, 0.01]
      }
    },
    {
      nodeName: 'wheel_front_left',
      type: 'drivetrain',
      subtype: 'wheelFrontLeft',
      defaultTransform: {
        scale: [0.01, 0.01, 0.01]
      }
    },
    {
      nodeName: 'wheel_front_right',
      type: 'drivetrain',
      subtype: 'wheelFrontRight',
      defaultTransform: {
        scale: [0.01, 0.01, 0.01]
      }
    },
    {
      nodeName: 'wheel_rear',
      type: 'drivetrain',
      subtype: 'wheelRear',
      defaultTransform: {
        scale: [0.01, 0.01, 0.01]
      }
    }
  ],
  scale: {
    global: .01,      // Base scale for all components
    frame: 0.01,       // Frame-specific scale
    suspension: 0.01,   // Suspension-specific scale
    drivetrain: 0.01   // Drivetrain-specific scale
  },
  suspension: {
    scale: 0.01,
    frontThreshold: 35,
    baseHeight: 0,
    maxExtension: 0.3048, // 12 inches in meters
    minExtension: -0.3048, // -12 inches in meters
    maxExtensionInches: 12,
    minExtensionInches: -12,
    dampingCoefficient: 0.5,
    mountPoints: {
      "frontLeft": [-0.15851670762394457, -0.040562102177363245, 0.4257602923760555],
      "frontRight": [0.1662480202628072, -0.040509979737193134, 0.4261460202628072],
      "rearLeft": [-0.1590329797371928, -0.040509979737193134, 0.07445702026280687],
      "rearRight": [0.16646525699485823, -0.0404873430051421, 0.0749262569948579]
    },
    defaultTransforms: {
      frontLeft: { 
        position: [-0.15851670762394457, -0.040562102177363245, 0.4257602923760555],
        scale: [0.01, 0.01, 0.01]
      },
      frontRight: {
        position: [0.1662480202628072, -0.040509979737193134, 0.4261460202628072],
        scale: [0.01, 0.01, 0.01]
      },
      rearLeft: {
        position: [-0.1590329797371928, -0.040509979737193134, 0.07445702026280687],
        scale: [0.01, 0.01, 0.01]
      },
      rearRight: {
        position: [0.16646525699485823, -0.0404873430051421, 0.0749262569948579],
        scale: [0.01, 0.01, 0.01]
      }
    }
  },
  steering: {
    maxAngleDegrees: 45,
    minAngleDegrees: -45
  },
  wheels: {
    wheelbase: 0.3048, // 12 inches in meters
    trackWidth: 0.3048, // 12 inches in meters
    offsetFromMount: {
      x: 0.05,
      z: 0.05
    },
    maxRPM: 3000,
    radiusInches: 2.76 // 70mm converted to inches
  },
  frame: {
    scale: [0.01, 0.01, 0.01]
  }
};

/**
 * Helper function to get default transform for a component
 */
export function getDefaultTransform(type: string, subtype?: string): ComponentTransform {
  const mapping = DEFAULT_CAR_CONFIG.nodeMappings.find(
    m => m.type === type && (!subtype || m.subtype === subtype)
  );
  return mapping?.defaultTransform || {};
}

// Export a singleton instance of the car configuration
export const CarConfig = { ...DEFAULT_CAR_CONFIG };