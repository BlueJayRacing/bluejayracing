// src/config/deviceConfig.ts

export interface ChannelConfig {
  id: number;           // Universal/ROS channel ID
  name: string;         // Human-readable name
  type: ChannelType;    // Type of channel
  category: string;     // Display category
  min_value?: number;   // Min value for scaling
  max_value?: number;   // Max value for scaling
  units?: string;       // Display units
}

export enum ChannelType {
  ANALOG = 0,
  DIGITAL = 1,
  COUNTER = 2,
  NAVIGATION = 3,
  STEERING = 4,
  TORQUE = 5
}

export enum DeviceType {
  TEENSY = 'teensy',
  ESP32 = 'esp32',
  RASPBERRY_PI = 'raspberry_pi'
}

export interface DeviceConfig {
  id: string;               // Device ID used in API
  name: string;             // Human-readable name
  type: DeviceType;         // Type of device
  deviceId?: number;        // Numeric device ID
  defaultSelected?: boolean;// Whether this device is selected by default
  macAddresses?: string[];  // MAC addresses associated with this device
  expectedChannels: ChannelConfig[]; // Channels expected from this device
}

export interface DynamicRange {
  default: {
    min: number;
    max: number;
  };
  ranges: {
    [key: string]: {
      min: number;
      max: number;
    };
  };
}

// Full configuration JSON with all device and channel mappings
export const DEVICE_CONFIG_JSON = {
  "teensy": {
    "variants": {
      "acc_v3-01": {
        "device_id": 1,
        "device_name": "20xt_Logger",
        "mac_addresses": [
          "04:e9:e5:19:61:83"
        ],
        "channel_mapping": {
          "0": 101,
          "1": 102,
          "2": 103,
          "3": 104,
          "4": 105,
          "5": 106,
          "6": 107,
          "7": 108,
          "8": 109,
          "9": 110,
          "10": 111,
          "11": 112,
          "12": 113,
          "13": 114,
          "14": 115,
          "15": 116,
          "16": 117,
          "17": 118,
          "18": 119,
          "19": 120,
          "20": 121,
          "21": 122,
          "22": 123,
          "23": 124,
          "24": 125,
          "25": 126,
          "26": 127,
          "27": 128,
          "28": 129,
          "29": 130
        }
      },
      "acc_v3-02": {
        "device_id": 2,
        "device_name": "WFT_Logger",
        "mac_addresses": [
          "04:e9:e5:19:60:b6"
        ],
        "channel_mapping": {
          "0": 201,
          "1": 202,
          "2": 203,
          "3": 204,
          "4": 205,
          "5": 206,
          "6": 207,
          "7": 208,
          "8": 209,
          "9": 210,
          "10": 211,
          "11": 212,
          "12": 213,
          "13": 214,
          "14": 215,
          "15": 216,
          "16": 217,
          "17": 218,
          "18": 219,
          "19": 220,
          "20": 221,
          "21": 222,
          "22": 223,
          "23": 224,
          "24": 225,
          "25": 226,
          "26": 227,
          "27": 228,
          "28": 229,
          "29": 230
        }
      },
      "acc_v4-01": {
        "device_id": 3,
        "device_name": "Comp_Logger",
        "mac_addresses": [
          "AA:BB:CC:DD:EE:03"
        ],
        "channel_mapping": {
          "0": 301,
          "1": 302,
          "2": 303,
          "3": 304,
          "4": 305,
          "5": 306,
          "6": 307,
          "7": 308,
          "8": 309,
          "9": 310,
          "10": 311,
          "11": 312,
          "12": 313,
          "13": 314,
          "14": 315,
          "15": 316,
          "16": 317,
          "17": 318,
          "18": 319,
          "19": 320,
          "20": 321,
          "21": 322,
          "22": 323,
          "23": 324,
          "24": 325,
          "25": 326,
          "26": 327,
          "27": 328,
          "28": 329,
          "29": 330
        }
      }
    },
    "default": {
      "device_id": 99,
      "device_name": "Teensy_Unspecified",
      "channel_mapping": {}
    }
  },
  "esp32": {
    "variants": {
      "esp_v3_b4": {
        "device_id": 10,
        "device_name": "TEST",
        "mac_addresses": [
          "8C:BF:EA:CB:B4:3C"
        ],
        "channel_mapping": {
          "0": 1001,
          "1": 1002,
          "2": 1003,
          "3": 1004,
          "4": 1005
        }
      }, 
      "esp_v3_b3": {
        "device_id": 11,
        "device_name": "REAR_LEFT",
        "mac_addresses": [
          "8C:BF:EA:CB:AF:58"
        ],
        "channel_mapping": {
          "0": 1101,
          "1": 1102,
          "2": 1103,
          "3": 1104,
          "4": 1105
        }
      },
      "esp_v3_b2": {
        "device_id": 12,
        "device_name": "REAR_RIGHT",
        "mac_addresses": [
          "8C:BF:EA:CC:3B:D4"
        ],
        "channel_mapping": {
          "0": 1201,
          "1": 1202,
          "2": 1203,
          "3": 1204,
          "4": 1205
        }
      },
      "in_shaft": {
        "device_id": 19,
        "device_name": "ESP32_InShaft",
        "mac_addresses": [
          "FF:EE:DD:CC:BB:02"
        ],
        "channel_mapping": {
          "0": 1901,
          "1": 1902,
          "2": 1903,
          "3": 1904,
          "4": 1905
        }
      }
    },
    "default": {
      "device_id": 199,
      "device_name": "ESP32_Unspecified",
      "channel_mapping": {}
    }
  }
};

// Channel type to category mapping
export const CHANNEL_TYPE_CATEGORIES = {
  [ChannelType.ANALOG]: 'Analog',
  [ChannelType.DIGITAL]: 'Digital',
  [ChannelType.COUNTER]: 'Counter',
  [ChannelType.NAVIGATION]: 'Navigation',
  [ChannelType.STEERING]: 'Steering',
  [ChannelType.TORQUE]: 'Torque'
};

// Dynamic ranges for channels by type and category
export const CHANNEL_RANGES: Record<string, DynamicRange> = {
  'Potentiometers': {
    default: { min: 0, max: 5 },
    ranges: {
      'linpot_front_left': { min: 0, max: 5 },
      'linpot_front_right': { min: 0, max: 5 },
      'linpot_rear_left': { min: 0, max: 5 },
      'linpot_rear_right': { min: 0, max: 5 }
    }
  },
  'Wheel Speeds': {
    default: { min: 0, max: 10000 },
    ranges: {
      'wheel_speed_fl': { min: 0, max: 10000 },
      'wheel_speed_fr': { min: 0, max: 10000 },
      'wheel_speed_rl': { min: 0, max: 10000 },
      'wheel_speed_rr': { min: 0, max: 10000 },
      // ESP32 wheel speed sensors may have different ranges
      'wheel_speed': { min: 0, max: 5000 }
    }
  },
  'Brake Pressure': {
    default: { min: 0, max: 2000 },
    ranges: {
      'brake_pressure_front': { min: 0, max: 2000 },
      'brake_pressure_rear': { min: 0, max: 2000 }
    }
  },
  'Steering': {
    default: { min: -180, max: 180 },
    ranges: {
      'steering_angle': { min: -180, max: 180 }
    }
  },
  'Axle': {
    default: { min: 0, max: 500 },
    ranges: {
      'axle_torque': { min: 0, max: 500 }
    }
  }
};

// Default configuration with expected devices and channels
export const DEFAULT_DEVICES: DeviceConfig[] = [
  {
    id: 'WFT_Logger',
    name: 'WFT Logger',
    type: DeviceType.TEENSY,
    deviceId: 2,
    defaultSelected: true,
    macAddresses: ['04:e9:e5:19:60:b6'],
    expectedChannels: [
      {
        id: 201,
        name: 'linpot_front_left',
        type: ChannelType.ANALOG,
        category: 'Potentiometers',
        min_value: 0,
        max_value: 5
      },
      {
        id: 202,
        name: 'linpot_front_right',
        type: ChannelType.ANALOG,
        category: 'Potentiometers',
        min_value: 0,
        max_value: 5
      },
      {
        id: 203,
        name: 'linpot_rear_left',
        type: ChannelType.ANALOG,
        category: 'Potentiometers',
        min_value: 0,
        max_value: 5
      },
      {
        id: 204,
        name: 'linpot_rear_right',
        type: ChannelType.ANALOG,
        category: 'Potentiometers',
        min_value: 0,
        max_value: 5
      },
      {
        id: 205,
        name: 'wheel_speed_fl',
        type: ChannelType.COUNTER,
        category: 'Wheel Speeds',
        min_value: 0,
        max_value: 10000
      },
      {
        id: 206,
        name: 'wheel_speed_fr',
        type: ChannelType.COUNTER,
        category: 'Wheel Speeds',
        min_value: 0,
        max_value: 10000
      },
      {
        id: 207,
        name: 'wheel_speed_rl',
        type: ChannelType.COUNTER,
        category: 'Wheel Speeds',
        min_value: 0,
        max_value: 10000
      },
      {
        id: 208,
        name: 'wheel_speed_rr',
        type: ChannelType.COUNTER,
        category: 'Wheel Speeds',
        min_value: 0,
        max_value: 10000
      },
      {
        id: 209,
        name: 'brake_pressure_front',
        type: ChannelType.ANALOG,
        category: 'Brake Pressure',
        min_value: 0,
        max_value: 2000
      },
      {
        id: 210,
        name: 'brake_pressure_rear',
        type: ChannelType.ANALOG,
        category: 'Brake Pressure',
        min_value: 0,
        max_value: 2000
      },
      {
        id: 211,
        name: 'steering_angle',
        type: ChannelType.STEERING,
        category: 'Steering',
        min_value: -180,
        max_value: 180
      },
      {
        id: 212,
        name: 'axle_torque',
        type: ChannelType.TORQUE,
        category: 'Axle',
        min_value: 0,
        max_value: 500
      }
    ]
  },
  {
    id: '20xt_Logger',
    name: '20XT Logger',
    type: DeviceType.TEENSY,
    deviceId: 1,
    macAddresses: ['04:e9:e5:19:61:83'],
    expectedChannels: [
      // Similar structure for 20XT Logger channels
      // These would be populated based on the channel mapping provided
      {
        id: 107,
        name: 'Channel_1',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      },
      {
        id: 108,
        name: 'Channel_6',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      },
      {
        id: 109,
        name: 'Channel_2',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      },
      {
        id: 110,
        name: 'Channel_7',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      },
      {
        id: 111,
        name: 'Channel_3',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      },
      {
        id: 112,
        name: 'Channel_8',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      },
      {
        id: 113,
        name: 'Channel_4',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      },
      {
        id: 114,
        name: 'Channel_9',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      },
      {
        id: 115,
        name: 'Channel_5',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      },
      {
        id: 116,
        name: 'Channel_10',
        type: ChannelType.ANALOG,
        category: 'Analog Inputs',
        min_value: 0,
        max_value: 5
      }
      // Additional channels would be defined based on the mapping
    ]
  },
  {
    id: 'REAR_LEFT',
    name: 'Rear Left ESP32',
    type: DeviceType.ESP32,
    deviceId: 11,
    macAddresses: ['8C:BF:EA:CB:AF:58'],
    expectedChannels: [
      {
        id: 1101,
        name: 'wheel_speed',
        type: ChannelType.COUNTER,
        category: 'Wheel Speeds',
        min_value: 0,
        max_value: 5000
      },
      {
        id: 1102,
        name: 'temperature',
        type: ChannelType.ANALOG,
        category: 'Temperature',
        min_value: -20,
        max_value: 120
      }
      // Additional ESP32 channels would be defined here
    ]
  },
  {
    id: 'REAR_RIGHT',
    name: 'Rear Right ESP32',
    type: DeviceType.ESP32,
    deviceId: 12,
    macAddresses: ['8C:BF:EA:CC:3B:D4'],
    expectedChannels: [
      {
        id: 1201,
        name: 'wheel_speed',
        type: ChannelType.COUNTER,
        category: 'Wheel Speeds',
        min_value: 0,
        max_value: 5000
      },
      {
        id: 1202,
        name: 'temperature',
        type: ChannelType.ANALOG,
        category: 'Temperature',
        min_value: -20,
        max_value: 120
      }
      // Additional ESP32 channels would be defined here
    ]
  }
];

// Configuration for API settings
export const API_CONFIG = {
  baseUrl: 'http://192.168.20.3:9365',
  endpoints: {
    health: '/health',
    config: '/config',
    allData: '/data/all',
    deviceData: '/data/',  // Append device ID
    mapping: '/mapping'
  },
  pollingIntervals: {
    data: 200,           // 200ms for data
    deviceAvailability: 5000  // 5s for device status
  }
};

/**
 * Get channel value range based on category and name
 * @param category The channel category
 * @param channelName The specific channel name
 * @returns An object with min and max values
 */
export const getChannelValueRange = (
  category: string,
  channelName: string
): { min: number, max: number } => {
  // Find the category in CHANNEL_RANGES
  const categoryRanges = CHANNEL_RANGES[category];
  
  if (!categoryRanges) {
    // Default to 0-100 if category not found
    return { min: 0, max: 100 };
  }
  
  // Check if there's a specific range for this channel
  if (categoryRanges.ranges[channelName]) {
    return categoryRanges.ranges[channelName];
  }
  
  // Fall back to category default
  return categoryRanges.default;
};

/**
 * Get color for a channel category
 * @param category The channel category name
 * @returns A hex color code
 */
export const getChannelCategoryColor = (category: string): string => {
  const categoryColors: Record<string, string> = {
    "Potentiometers": "#4299e1", // Blue
    "Wheel Speeds": "#48bb78",   // Green
    "Brake Pressure": "#ed8936", // Orange
    "Steering": "#9f7aea",       // Purple
    "Axle": "#f56565",           // Red
    "Temperature": "#ed64a6",    // Pink
    "Pressure": "#667eea",       // Indigo
    "Analog Inputs": "#4fd1c5",  // Teal
    "Digital Inputs": "#fbd38d", // Yellow
    "Counter Inputs": "#90cdf4", // Light Blue
    "Navigation": "#68d391",     // Light Green
    "Torque": "#fc8181"          // Light Red
  };

  return categoryColors[category] || "#a0aec0"; // Default gray
};

/**
 * Get color based on channel type
 * @param type The channel type enum
 * @returns A hex color code
 */
export const getChannelTypeColor = (type: ChannelType): string => {
  const typeColors: Record<number, string> = {
    [ChannelType.ANALOG]: "#4299e1",    // Blue
    [ChannelType.DIGITAL]: "#48bb78",   // Green
    [ChannelType.COUNTER]: "#ed8936",   // Orange
    [ChannelType.NAVIGATION]: "#9f7aea",// Purple
    [ChannelType.STEERING]: "#f56565",  // Red
    [ChannelType.TORQUE]: "#ed64a6"     // Pink
  };

  return typeColors[type] || "#a0aec0"; // Default gray
};

/**
 * Get a color representing device availability
 * @param isAvailable Whether the device is currently available
 * @param isExpected Whether the device is in the expected configuration
 * @returns A hex color code
 */
export const getDeviceAvailabilityColor = (
  isAvailable: boolean,
  isExpected: boolean
): string => {
  if (isAvailable && isExpected) return "#48bb78"; // Green - available and expected
  if (!isAvailable && isExpected) return "#fc8181"; // Light Red - expected but unavailable
  if (isAvailable && !isExpected) return "#4299e1"; // Blue - available but unexpected
  return "#a0aec0"; // Gray - unknown or default
};

/**
 * Get a color representing channel availability
 * @param isDeviceAvailable Whether the parent device is available
 * @param isDeviceExpected Whether the parent device is expected
 * @param isChannelExpected Whether the channel is in the expected configuration
 * @returns A hex color code
 */
export const getChannelAvailabilityColor = (
  isDeviceAvailable: boolean,
  isDeviceExpected: boolean,
  isChannelExpected: boolean
): string => {
  if (!isDeviceAvailable) return "#a0aec0"; // Gray - device not available
  if (isDeviceExpected && isChannelExpected) return "#48bb78"; // Green - expected device and channel
  if (isDeviceExpected && !isChannelExpected) return "#9f7aea"; // Purple - expected device, unexpected channel
  if (!isDeviceExpected && isChannelExpected) return "#4299e1"; // Blue - unexpected device, expected channel
  return "#90cdf4"; // Light Blue - unexpected device and channel but available
};

/**
 * Find a device in the config by its ID
 * @param deviceId The device ID to find
 * @returns The device config or null if not found
 */
export const findDeviceById = (deviceId: string): DeviceConfig | null => {
  return DEFAULT_DEVICES.find(device => device.id === deviceId) || null;
};

/**
 * Find channel config by its universal ID
 * @param channelId The universal channel ID
 * @returns The channel config and parent device, or null if not found
 */
export const findChannelById = (
  channelId: number
): { channel: ChannelConfig; device: DeviceConfig } | null => {
  for (const device of DEFAULT_DEVICES) {
    const channel = device.expectedChannels.find(c => c.id === channelId);
    if (channel) {
      return { channel, device };
    }
  }
  return null;
};

/**
 * Get all devices of a specific type
 * @param type The device type
 * @returns Array of matching device configs
 */
export const getDevicesByType = (type: DeviceType): DeviceConfig[] => {
  return DEFAULT_DEVICES.filter(device => device.type === type);
};

/**
 * Get all channels in a specific category
 * @param category The category name
 * @returns Array of { channel, device } objects
 */
export const getChannelsByCategory = (
  category: string
): Array<{ channel: ChannelConfig; device: DeviceConfig }> => {
  const results: Array<{ channel: ChannelConfig; device: DeviceConfig }> = [];
  
  DEFAULT_DEVICES.forEach(device => {
    device.expectedChannels.forEach(channel => {
      if (channel.category === category) {
        results.push({ channel, device });
      }
    });
  });
  
  return results;
};

/**
 * Find device and channel info from the JSON config using internal channel index
 * @param deviceType Type of device (e.g., "teensy", "esp32")
 * @param deviceName Name of the device (e.g., "WFT_Logger")
 * @param channelIndex Internal channel index
 * @returns Object with universal channel ID and full device info or null if not found
 */
export const getChannelMappingFromJson = (
  deviceType: string,
  deviceName: string,
  channelIndex: number
): { universalChannelId: number; deviceInfo: any } | null => {
  const deviceTypeConfig = DEVICE_CONFIG_JSON[deviceType as keyof typeof DEVICE_CONFIG_JSON];
  if (!deviceTypeConfig) return null;
  
  // Look for the device in variants
  for (const variant in deviceTypeConfig.variants) {
    const deviceInfo = deviceTypeConfig.variants[variant];
    if (deviceInfo.device_name === deviceName) {
      // Found the device, now look for the channel
      const channelIndexStr = channelIndex.toString();
      if (channelIndexStr in deviceInfo.channel_mapping) {
        return {
          universalChannelId: deviceInfo.channel_mapping[channelIndexStr],
          deviceInfo
        };
      }
    }
  }
  
  // If not found in variants, check default
  if (deviceTypeConfig.default.device_name === deviceName) {
    const channelIndexStr = channelIndex.toString();
    if (channelIndexStr in deviceTypeConfig.default.channel_mapping) {
      return {
        universalChannelId: deviceTypeConfig.default.channel_mapping[channelIndexStr],
        deviceInfo: deviceTypeConfig.default
      };
    }
  }
  
  return null;
};