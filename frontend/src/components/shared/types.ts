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

export interface ChannelMetadata {
  name: string;
  type: ChannelType;
  sample_rate: number;  // Hz
  transmission_rate: number;  // Hz
  location: string;    // e.g., "FrontLeft", "RearRight"
  units: string;       // e.g., "V", "RPM", "PSI"
  description: string;
  min_value: number;
  max_value: number;
}

export interface Channel {
  name: string;
  type: ChannelType;
  min_value: number;
  max_value: number;
  samples: TimeValue[];
  metadata?: ChannelMetadata;
}

export interface Recording {
  id: string;
  name: string;
  startTime: number;
  endTime: number | null;
  channelData: {
    [channelName: string]: TimeValue[];
  };
  channelMetadata: {
    [channelName: string]: ChannelMetadata;
  };
  stats: {
    duration: number;    // ms
    dataSize: number;    // bytes
    sampleCount: number; // total samples
    channelCount: number;
    maxSampleRate: number;
    averageSampleRate: number;
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