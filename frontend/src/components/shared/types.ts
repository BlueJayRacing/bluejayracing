// src/components/shared/types.ts
export interface TimeValue {
  timestamp: number;
  value: number;
}

export enum ChannelType {
  LINEAR_POTENTIOMETER = 0,
  HALL_EFFECT_SPEED = 1,
  BRAKE_PRESSURE = 2,
  NAVIGATION = 3,
  STEERING_ENCODER = 4,
  AXLE_TORQUE = 5
}

export interface Channel {
  name: string;
  type: ChannelType;
  min_value: number;
  max_value: number;
  samples: TimeValue[];
}

export interface Recording {
  id: string;
  name: string;
  startTime: number;
  endTime: number | null;
  channelData: {
    [channelName: string]: TimeValue[];
  };
  stats: {
    duration: number;    // in milliseconds
    dataSize: number;    // rough approximation in bytes
    sampleCount: number; // total samples across all channels
  };
}

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
    position: [number, number, number];
    rotation: [number, number, number, number];
  };
}