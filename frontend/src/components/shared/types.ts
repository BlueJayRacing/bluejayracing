// src/components/shared/types.ts

// TimeValue represents a single data point with timestamp and value
export interface TimeValue {
  timestamp: number;
  value: number;
}

// Channel represents a data channel with samples
export interface Channel {
  name: string;             // Full channel name (deviceId/channelName)
  type: number;             // Channel type (analog, digital, etc.)
  min_value: number;        // Min value for scaling
  max_value: number;        // Max value for scaling
  samples: TimeValue[];     // Samples data
  metadata?: ChannelMetadata; // Additional metadata
  device_id?: string;       // Source device ID
}

// ChannelMetadata represents additional information about a channel
export interface ChannelMetadata {
  name: string;
  type: number;
  sample_rate: number;
  transmission_rate: number;
  location: string;
  units: string;
  description: string;
  min_value: number;
  max_value: number;
}

// Device represents a data source
export interface Device {
  id: string;               // Device ID
  name: string;             // Human-readable name
  available: boolean;       // Whether the device is available
  lastSeen?: number;        // Last time the device was seen
  channels?: string[];      // Channel names associated with this device
  config?: any;             // Reference to the device configuration
}

// Recording represents a saved data session
export interface Recording {
  id: string;
  name: string;
  startTime: number;
  endTime?: number;
  devices: string[];
  channels: string[];
  data?: Channel[];
}