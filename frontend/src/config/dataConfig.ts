// src/config/deviceConfig.ts

// Define channel configuration interface
export interface ChannelConfig {
  id: number;            // Universal channel ID (ros_channel_id)
  name: string;          // Human-readable name
  type: number;          // Channel type (analog, digital, etc.)
  min_value?: number;    // Min value for scaling
  max_value?: number;    // Max value for scaling
  units?: string;        // Units (e.g., volts, rpm, etc.)
  category?: string;     // Category for grouping
}

// Define device configuration interface
export interface DeviceConfig {
  id: string;            // Device ID (e.g., "teensy", "esp32")
  name: string;          // Human-readable name
  description?: string;  // Device description
  defaultSelected?: boolean; // Whether this device should be selected by default
  expectedChannels: ChannelConfig[]; // Expected channels for this device
}

// API configuration
export const API_CONFIG = {
  baseUrl: "http://192.168.20.3:9365", // Base API URL
  endpoints: {
    data: "/data",       // Data endpoint
    mapping: "/mapping", // Channel mapping endpoint
    all: "/data/all",    // All data endpoint
  },
  pollingIntervals: {
    data: 200,           // 200ms for data polling
    deviceAvailability: 5000, // 5s for device availability
  }
};

// Teensy device configuration
const TEENSY_DEVICE: DeviceConfig = {
  id: "teensy",
  name: "Teensy 4.1",
  description: "Main controller for data acquisition",
  defaultSelected: true,
  expectedChannels: [
    // Analog channels
    {
      id: 1,
      name: "linpot_front_left",
      type: 0,
      min_value: 0,
      max_value: 5,
      units: "V",
      category: "Potentiometers"
    },
    {
      id: 2,
      name: "linpot_front_right",
      type: 0,
      min_value: 0,
      max_value: 5,
      units: "V",
      category: "Potentiometers"
    },
    {
      id: 3,
      name: "linpot_rear_left",
      type: 0,
      min_value: 0,
      max_value: 5,
      units: "V",
      category: "Potentiometers"
    },
    {
      id: 4,
      name: "linpot_rear_right",
      type: 0,
      min_value: 0,
      max_value: 5,
      units: "V",
      category: "Potentiometers"
    },
    
    // Digital channels
    {
      id: 5,
      name: "wheel_speed_fl",
      type: 1,
      min_value: 0,
      max_value: 10000,
      units: "RPM",
      category: "Wheel Speeds"
    },
    {
      id: 6,
      name: "wheel_speed_fr",
      type: 1,
      min_value: 0,
      max_value: 10000,
      units: "RPM",
      category: "Wheel Speeds"
    },
    {
      id: 7,
      name: "wheel_speed_rl",
      type: 1,
      min_value: 0,
      max_value: 10000,
      units: "RPM",
      category: "Wheel Speeds"
    },
    {
      id: 8,
      name: "wheel_speed_rr",
      type: 1,
      min_value: 0,
      max_value: 10000,
      units: "RPM",
      category: "Wheel Speeds"
    },
    
    // Pressure channels
    {
      id: 9,
      name: "brake_pressure_front",
      type: 2,
      min_value: 0,
      max_value: 2000,
      units: "PSI",
      category: "Brake Pressure"
    },
    {
      id: 10,
      name: "brake_pressure_rear",
      type: 2,
      min_value: 0,
      max_value: 2000,
      units: "PSI",
      category: "Brake Pressure"
    },
    
    // Other sensors
    {
      id: 11,
      name: "steering_angle",
      type: 4,
      min_value: -180,
      max_value: 180,
      units: "deg",
      category: "Steering"
    },
    {
      id: 12,
      name: "axle_torque",
      type: 5,
      min_value: 0,
      max_value: 500,
      units: "Nm",
      category: "Axle"
    }
  ]
};

// ESP32 device configuration
const ESP32_DEVICE: DeviceConfig = {
  id: "esp32",
  name: "ESP32",
  description: "Wireless data acquisition module",
  expectedChannels: [
    {
      id: 101,
      name: "temperature_ambient",
      type: 0,
      min_value: -40,
      max_value: 125,
      units: "°C",
      category: "Temperature"
    },
    {
      id: 102,
      name: "temperature_coolant",
      type: 0,
      min_value: -40,
      max_value: 125,
      units: "°C",
      category: "Temperature"
    },
    {
      id: 103,
      name: "pressure_oil",
      type: 2,
      min_value: 0,
      max_value: 150,
      units: "PSI",
      category: "Pressure"
    },
    {
      id: 104,
      name: "pressure_fuel",
      type: 2,
      min_value: 0,
      max_value: 100,
      units: "PSI",
      category: "Pressure"
    }
  ]
};

// Raspberry Pi device configuration
const RPI_DEVICE: DeviceConfig = {
  id: "rpi",
  name: "Raspberry Pi",
  description: "Central data logger",
  expectedChannels: [
    {
      id: 201,
      name: "gps_latitude",
      type: 10,
      category: "GPS"
    },
    {
      id: 202,
      name: "gps_longitude",
      type: 10,
      category: "GPS"
    },
    {
      id: 203,
      name: "gps_speed",
      type: 1,
      min_value: 0,
      max_value: 150,
      units: "mph",
      category: "GPS"
    },
    {
      id: 204,
      name: "imu_acceleration_x",
      type: 11,
      min_value: -16,
      max_value: 16,
      units: "g",
      category: "IMU"
    },
    {
      id: 205,
      name: "imu_acceleration_y",
      type: 11,
      min_value: -16,
      max_value: 16,
      units: "g",
      category: "IMU"
    },
    {
      id: 206,
      name: "imu_acceleration_z",
      type: 11,
      min_value: -16,
      max_value: 16,
      units: "g",
      category: "IMU"
    }
  ]
};

// WFT_Logger device configuration
const WFT_LOGGER_DEVICE: DeviceConfig = {
  id: "WFT_Logger",
  name: "WFT Logger",
  description: "Wheel Force Transducer Logger",
  expectedChannels: [
    {
      id: 207,
      name: "Channel_1",
      type: 6,
      category: "WFT"
    },
    {
      id: 208,
      name: "Channel_6",
      type: 6,
      category: "WFT"
    },
    {
      id: 209,
      name: "Channel_2",
      type: 6,
      category: "WFT"
    },
    {
      id: 210,
      name: "Channel_7",
      type: 6,
      category: "WFT"
    },
    {
      id: 211,
      name: "Channel_3",
      type: 6,
      category: "WFT"
    },
    {
      id: 212,
      name: "Channel_8",
      type: 6,
      category: "WFT"
    },
    {
      id: 213,
      name: "Channel_4",
      type: 6,
      category: "WFT"
    },
    {
      id: 214,
      name: "Channel_9",
      type: 6,
      category: "WFT"
    },
    {
      id: 215,
      name: "Channel_5",
      type: 6,
      category: "WFT"
    },
    {
      id: 216,
      name: "Channel_10",
      type: 6,
      category: "WFT"
    }
  ]
};

// Export default device configurations
export const DEFAULT_DEVICES: DeviceConfig[] = [
  TEENSY_DEVICE,
  ESP32_DEVICE,
  RPI_DEVICE,
  WFT_LOGGER_DEVICE
];

// Helper functions
export const getChannelCategoryColor = (category: string): string => {
  const categoryColors: Record<string, string> = {
    "Potentiometers": "#4299e1", // Blue
    "Wheel Speeds": "#48bb78",   // Green
    "Brake Pressure": "#ed8936", // Orange
    "Steering": "#9f7aea",       // Purple
    "Axle": "#f56565",           // Red
    "Temperature": "#ed64a6",    // Pink
    "Pressure": "#667eea",       // Indigo
    "GPS": "#4fd1c5",            // Teal
    "IMU": "#fbd38d",            // Yellow
    "WFT": "#90cdf4",            // Light Blue
  };

  return categoryColors[category] || "#a0aec0"; // Default gray
};

// Get device availability color
export const getDeviceAvailabilityColor = (
  isAvailable: boolean,
  isExpected: boolean
): string => {
  if (isAvailable && isExpected) return "#48bb78"; // Green
  if (!isAvailable && isExpected) return "#fc8181"; // Light Red
  if (isAvailable && !isExpected) return "#4299e1"; // Blue
  return "#a0aec0"; // Gray
};

// Get channel availability color
export const getChannelAvailabilityColor = (
  isDeviceAvailable: boolean,
  isDeviceExpected: boolean,
  isChannelExpected: boolean
): string => {
  if (!isDeviceAvailable) return "#a0aec0"; // Gray if device not available
  if (isDeviceExpected && isChannelExpected) return "#48bb78"; // Green
  if (isDeviceExpected && !isChannelExpected) return "#9f7aea"; // Purple
  if (!isDeviceExpected && isChannelExpected) return "#4299e1"; // Blue
  return "#90cdf4"; // Light Blue
};