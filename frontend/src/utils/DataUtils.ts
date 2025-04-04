// src/utils/DataUtils.ts

import { Channel, TimeValue } from '../components/shared/types';
import { DEFAULT_DEVICES } from '../config/deviceConfig';

/**
 * Maps a universal channel ID to device-specific details
 */
export const mapChannelIdToDeviceChannel = (
  channelId: number
): { deviceId: string; channelName: string } | null => {
  // Search through all devices and their expected channels
  for (const device of DEFAULT_DEVICES) {
    const matchingChannel = device.expectedChannels.find(
      channel => channel.id === channelId
    );
    
    if (matchingChannel) {
      return {
        deviceId: device.id,
        channelName: matchingChannel.name
      };
    }
  }
  
  return null;
};

/**
 * Maps device-specific channel details to universal channel ID
 */
export const mapDeviceChannelToChannelId = (
  deviceId: string,
  channelName: string
): number | null => {
  // Find the device
  const device = DEFAULT_DEVICES.find(d => d.id === deviceId);
  if (!device) return null;
  
  // Find the matching channel
  const channel = device.expectedChannels.find(c => c.name === channelName);
  if (!channel) return null;
  
  return channel.id;
};

/**
 * Creates a full channel name from device ID and channel name
 */
export const createFullChannelName = (
  deviceId: string,
  channelName: string
): string => {
  return `${deviceId}/${channelName}`;
};

/**
 * Splits a full channel name into device ID and channel name
 */
export const splitFullChannelName = (
  fullName: string
): { deviceId: string; channelName: string } => {
  const parts = fullName.split('/');
  if (parts.length < 2) {
    return { deviceId: 'unknown', channelName: fullName };
  }
  return { deviceId: parts[0], channelName: parts[1] };
};

/**
 * Deduplicates time series data by timestamp
 */
export const deduplicateTimeSeriesData = (data: TimeValue[]): TimeValue[] => {
  // Create a map to store the latest value for each timestamp
  const timeMap = new Map<number, number>();
  
  // Process each sample
  data.forEach(sample => {
    // Only keep the latest value for each timestamp
    if (!timeMap.has(sample.timestamp) || 
        timeMap.get(sample.timestamp)! < sample.value) {
      timeMap.set(sample.timestamp, sample.value);
    }
  });
  
  // Convert back to array
  const result: TimeValue[] = [];
  timeMap.forEach((value, timestamp) => {
    result.push({ timestamp, value });
  });
  
  // Sort by timestamp
  result.sort((a, b) => a.timestamp - b.timestamp);
  
  return result;
};

/**
 * Downsamples time series data to a maximum number of points
 */
export const downsampleTimeSeriesData = (
  data: TimeValue[],
  maxPoints: number
): TimeValue[] => {
  // If data is already small enough, return as is
  if (data.length <= maxPoints) return data;
  
  // Calculate stride
  const stride = Math.ceil(data.length / maxPoints);
  
  // Sample at regular intervals
  const result: TimeValue[] = [];
  for (let i = 0; i < data.length; i += stride) {
    result.push(data[i]);
  }
  
  // Always include the last point
  if (result.length > 0 && result[result.length - 1] !== data[data.length - 1]) {
    result.push(data[data.length - 1]);
  }
  
  return result;
};

/**
 * Filters time series data to a specific time window
 */
export const filterTimeSeriesDataByWindow = (
  data: TimeValue[],
  startTime: number,
  endTime: number
): TimeValue[] => {
  return data.filter(
    sample => sample.timestamp >= startTime && sample.timestamp <= endTime
  );
};