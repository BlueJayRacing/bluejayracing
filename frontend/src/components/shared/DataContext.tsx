// src/components/shared/DataContext.tsx

import React, { createContext, useState, useEffect, useRef } from 'react';
import { ApiService } from './ApiService';
import { DEFAULT_DEVICES, DeviceConfig, API_CONFIG } from '../../config/deviceConfig';

// Extending the current Channel type to include device information
export interface Channel {
  name: string;             // Full channel name (deviceId/channelName)
  type: number;             // Channel type (analog, digital, etc.)
  min_value: number;        // Min value for scaling
  max_value: number;        // Max value for scaling
  samples: TimeValue[];     // Samples data
  metadata?: ChannelMetadata; // Additional metadata
  device_id?: string;       // Source device ID
}

export interface TimeValue {
  timestamp: number;
  value: number;
}

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

export interface Device {
  id: string;               // Device ID
  name: string;             // Human-readable name
  available: boolean;       // Whether the device is available
  lastSeen?: number;        // Last time the device was seen
  channels?: string[];      // Channel names associated with this device
  config?: DeviceConfig;    // Reference to the device configuration
}

// Context type definition
export interface DataContextType {
  channels: Channel[];
  isLoading: boolean;
  error: Error | null;
  
  // Device-related state and methods
  devices: Device[];
  selectedDevice: Device | null;
  selectDevice: (deviceId: string) => void;
  
  // Data access methods
  getAllNewData: (channelNames: string[]) => Record<string, { channelName: string, hasNewData: boolean, newSamples: TimeValue[] }>;
  getAllDataForChannel: (channelName: string) => TimeValue[] | null;
  
  // Recording-related state and methods
  isRecording: boolean;
  recordings: any[];
  currentRecording: any | null;
  startRecording: (name?: string) => any;
  stopRecording: () => any;
  deleteRecording: (id: string) => void;
  renameRecording: (id: string, newName: string) => void;
  getRecordingById: (id: string) => any;
}

// Create the context
export const DataContext = createContext<DataContextType | null>(null);

// Create the provider component
export const DataProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  // Channel state
  const [channels, setChannels] = useState<Channel[]>([]);
  
  // Loading and error state
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);
  
  // Device state
  const [devices, setDevices] = useState<Device[]>([]);
  const [selectedDevice, setSelectedDevice] = useState<Device | null>(null);
  
  // Data buffers for each channel
  const dataBuffers = useRef<Record<string, TimeValue[]>>({});
  
  // Last processed timestamp for each channel
  const lastProcessedTimestamp = useRef<Record<string, number>>({});
  
  // Intervals for polling
  const dataIntervalRef = useRef<number | null>(null);
  const deviceIntervalRef = useRef<number | null>(null);
  
  // Debug logging
  const apiCallStatsRef = useRef<{
    fetchCount: number;
    lastFetchTime: number;
    apiResponseSizes: number[];
    lastResponseLog: number;
  }>({
    fetchCount: 0,
    lastFetchTime: 0,
    apiResponseSizes: [],
    lastResponseLog: 0
  });
  
  // Helper to deduplicate channels from API data
  const dedupeChannels = (channels: Channel[]): Channel[] => {
    const channelMap = new Map<string, Channel>();
    channels.forEach(channel => {
      // Use the full channel name (deviceId/channelName) as a unique key
      const key = channel.name;
      if (!channelMap.has(key)) {
         channelMap.set(key, channel);
      } else {
         // Merge sample arrays for duplicate channels
         const existing = channelMap.get(key)!;
         const mergedSamples = [...existing.samples, ...channel.samples];
         // Deduplicate samples based on timestamp
         const sampleMap = new Map<number, TimeValue>();
         mergedSamples.forEach(sample => {
           sampleMap.set(sample.timestamp, sample);
         });
         existing.samples = Array.from(sampleMap.values()).sort((a, b) => a.timestamp - b.timestamp);
         channelMap.set(key, existing);
      }
    });
    return Array.from(channelMap.values());
  };
  
  // Initialize with default devices
  useEffect(() => {
    // Convert default devices to our internal device structure
    const defaultDevices = DEFAULT_DEVICES.map(config => ({
      id: config.id,
      name: config.name,
      available: false, // Will be updated when we check availability
      config: config,
      channels: config.expectedChannels.map(channel => `${config.id}/${channel.name}`)
    }));
    
    setDevices(defaultDevices);
    
    // Select the default device if specified
    const defaultDevice = defaultDevices.find(d => d.config?.defaultSelected);
    if (defaultDevice) {
      setSelectedDevice(defaultDevice);
    } else if (defaultDevices.length > 0) {
      setSelectedDevice(defaultDevices[0]);
    }

    // Immediately check device availability on mount
    checkDeviceAvailability();
    
    console.log('DataContext initialized with polling intervals:', {
      deviceAvailability: API_CONFIG.pollingIntervals.deviceAvailability,
      data: API_CONFIG.pollingIntervals.data
    });
    
    // Initialize interval for checking device availability
    deviceIntervalRef.current = window.setInterval(
      checkDeviceAvailability, 
      API_CONFIG.pollingIntervals.deviceAvailability
    );
    
    // Initialize interval for fetching data
    dataIntervalRef.current = window.setInterval(
      fetchData,
      API_CONFIG.pollingIntervals.data
    );
    
    return () => {
      if (deviceIntervalRef.current) window.clearInterval(deviceIntervalRef.current);
      if (dataIntervalRef.current) window.clearInterval(dataIntervalRef.current);
    };
  }, []);
  
  // Function to select a device
  const selectDevice = (deviceId: string) => {
    const device = devices.find(d => d.id === deviceId);
    if (device) {
      setSelectedDevice(device);
    }
  };
  
  // Check device availability
  const checkDeviceAvailability = async () => {
    try {
      const result = await ApiService.getChannelMapping();
      
      if (result.success && result.data && result.data.mappings) {
        const availableSources = new Set<string>();
        result.data.mappings.forEach((mapping) => {
          availableSources.add(mapping.source_id);
        });
        
        setDevices(prev => prev.map(device => ({
          ...device,
          available: availableSources.has(device.id),
          lastSeen: availableSources.has(device.id) ? Date.now() : device.lastSeen
        })));
        
        availableSources.forEach(sourceId => {
          if (!devices.some(d => d.id === sourceId)) {
            setDevices(prev => [
              ...prev,
              {
                id: sourceId,
                name: sourceId,
                available: true,
                lastSeen: Date.now(),
                channels: []
              }
            ]);
          }
        });
        
        const sourceChannelMap = new Map<string, Set<string>>();
        result.data.mappings.forEach((mapping) => {
          if (!sourceChannelMap.has(mapping.source_id)) {
            sourceChannelMap.set(mapping.source_id, new Set());
          }
          sourceChannelMap.get(mapping.source_id)?.add(mapping.channel_name);
        });
        
        setDevices(prev => prev.map(device => {
          const knownChannels = sourceChannelMap.get(device.id) || new Set();
          const expectedChannels = device.config?.expectedChannels || [];
          const allChannels = new Set([
            ...[...knownChannels].map(channel => `${device.id}/${channel}`),
            ...expectedChannels.map(channel => `${device.id}/${channel.name}`)
          ]);
          
          return {
            ...device,
            channels: [...allChannels]
          };
        }));
      }
    } catch (error) {
      console.error('Error checking device availability:', error);
    }
  };
  
  // Fetch data from API
  const fetchData = async () => {
    try {
      apiCallStatsRef.current.fetchCount++;
      const now = Date.now();
      const timeSinceLastFetch = now - apiCallStatsRef.current.lastFetchTime;
      apiCallStatsRef.current.lastFetchTime = now;
      
      if (apiCallStatsRef.current.fetchCount % 10 === 0) {
        console.log(`API fetch #${apiCallStatsRef.current.fetchCount}, time since last fetch: ${timeSinceLastFetch}ms`);
      }
      
      const response = await ApiService.getAllChannelData();
      
      if (response.success && response.data && response.data.data_chunks) {
        const deviceDataChunks = response.data.data_chunks;
        let newChannels: Channel[] = [];
        
        const responseSize = JSON.stringify(response.data).length;
        apiCallStatsRef.current.apiResponseSizes.push(responseSize);
        
        if (now - apiCallStatsRef.current.lastResponseLog > 5000) {
          apiCallStatsRef.current.lastResponseLog = now;
          const avgSize = apiCallStatsRef.current.apiResponseSizes.reduce((sum, size) => sum + size, 0) / 
            apiCallStatsRef.current.apiResponseSizes.length;
          console.log(`API response stats: chunks=${deviceDataChunks.length}, size=${responseSize} bytes, avg=${avgSize.toFixed(0)} bytes`);
          
          if (apiCallStatsRef.current.apiResponseSizes.length > 100) {
            apiCallStatsRef.current.apiResponseSizes = apiCallStatsRef.current.apiResponseSizes.slice(-20);
          }
        }
        
        deviceDataChunks.forEach((chunk) => {
          const deviceId = chunk.source_id;
          const channelSamples = new Map<number, { timestamps: number[], values: number[] }>();
          
          chunk.samples.forEach((sample) => {
            if (!channelSamples.has(sample.channel_id)) {
              channelSamples.set(sample.channel_id, { timestamps: [], values: [] });
            }
            const channelData = channelSamples.get(sample.channel_id)!;
            channelData.timestamps.push(sample.timestamp);
            channelData.values.push(sample.data_value);
          });
          
          const deviceConfig = DEFAULT_DEVICES.find(d => d.id === deviceId);
          
          channelSamples.forEach((data, channelId) => {
            const channelConfig = deviceConfig?.expectedChannels.find(c => c.id === channelId);
            let channelName = `unknown_${channelId}`;
            let channelType = 0;
            let minValue = 0;
            let maxValue = 100;
            
            if (channelConfig) {
              channelName = channelConfig.name;
              channelType = channelConfig.type;
              minValue = channelConfig.min_value ?? 0;
              maxValue = channelConfig.max_value ?? 100;
            }
            
            const samples = data.timestamps.map((timestamp, index) => ({
              timestamp,
              value: data.values[index]
            }));
            
            newChannels.push({
              name: `${deviceId}/${channelName}`,
              type: channelType,
              min_value: minValue,
              max_value: maxValue,
              samples,
              device_id: deviceId
            });
          });
        });
        
        const dedupedChannels = dedupeChannels(newChannels);
        processNewData(dedupedChannels);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
    } finally {
      if (isLoading) {
        setIsLoading(false);
      }
    }
  };
  
  // Process new data and update buffers
  const processNewData = (newChannels: Channel[]) => {
    setChannels(newChannels);
    const processingStartTime = performance.now();
    
    newChannels.forEach(channel => {
      const { name, samples } = channel;
      
      if (!dataBuffers.current[name]) {
        dataBuffers.current[name] = [];
        lastProcessedTimestamp.current[name] = 0;
        console.log(`Initialized new buffer for ${name}`);
      }
      
      const lastTimestamp = lastProcessedTimestamp.current[name];
      const newSamples = samples.filter(sample => sample.timestamp > lastTimestamp);
      if (newSamples.length === 0) return;
      
      newSamples.sort((a, b) => a.timestamp - b.timestamp);
      
      const mergedSamples = new Map<number, number>();
      dataBuffers.current[name].forEach(sample => {
        mergedSamples.set(sample.timestamp, sample.value);
      });
      newSamples.forEach(sample => {
        mergedSamples.set(sample.timestamp, sample.value);
      });
      
      dataBuffers.current[name] = Array.from(mergedSamples.entries())
        .map(([timestamp, value]) => ({ timestamp, value }))
        .sort((a, b) => a.timestamp - b.timestamp);
      
      if (newSamples.length > 0) {
        lastProcessedTimestamp.current[name] = Math.max(lastProcessedTimestamp.current[name], newSamples[newSamples.length - 1].timestamp);
      }
      
      const bufferLimit = 2000;
      if (dataBuffers.current[name].length > bufferLimit) {
        dataBuffers.current[name] = dataBuffers.current[name].slice(-bufferLimit);
      }
    });
    
    const processingTime = performance.now() - processingStartTime;
    if (processingTime > 50) {
      console.log(`Data processing took ${processingTime.toFixed(1)}ms for ${newChannels.length} channels`);
    }
  };
  
  // Get all new data for specified channels
  const getAllNewData = (channelNames: string[]) => {
    const result: Record<string, { channelName: string, hasNewData: boolean, newSamples: TimeValue[] }> = {};
    const shouldLog = apiCallStatsRef.current.fetchCount % 10 === 0;
    
    if (shouldLog) {
      console.log(`getAllNewData called for ${channelNames.length} channels:`, channelNames);
    }
    
    channelNames.forEach(channelName => {
      const channel = channels.find(c => c.name === channelName);
      
      if (!channel) {
        if (shouldLog) console.debug(`Channel not found: ${channelName}`);
        result[channelName] = {
          channelName,
          hasNewData: false,
          newSamples: []
        };
        return;
      }
      
      const buffer = dataBuffers.current[channelName] || [];
      
      if (shouldLog) {
        console.debug(`Buffer for ${channelName}: ${buffer.length} samples`);
        if (buffer.length > 0) {
          const earliest = new Date(buffer[0].timestamp).toISOString();
          const latest = new Date(buffer[buffer.length-1].timestamp).toISOString();
          const timespan = (buffer[buffer.length-1].timestamp - buffer[0].timestamp) / 1000;
          console.log(`${channelName} buffer spans ${timespan.toFixed(1)}s from ${earliest} to ${latest}`);
        }
      }
      
      result[channelName] = {
        channelName,
        hasNewData: buffer.length > 0,
        newSamples: buffer
      };
      
      if (shouldLog) {
        console.debug(`Returning ${result[channelName].newSamples.length} samples for ${channelName}`);
        if (result[channelName].newSamples.length > 0) {
          console.debug(`Sample range: ${new Date(result[channelName].newSamples[0].timestamp).toISOString()} to ${
            new Date(result[channelName].newSamples[result[channelName].newSamples.length-1].timestamp).toISOString()
          }`);
        }
      }
    });
    
    return result;
  };
  
  // Get all data for a specific channel
  const getAllDataForChannel = (channelName: string): TimeValue[] | null => {
    const buffer = dataBuffers.current[channelName];
    if (buffer) {
      const bufferSpan = buffer.length > 0 ? (buffer[buffer.length-1].timestamp - buffer[0].timestamp) / 1000 : 0;
      console.log(`getAllDataForChannel(${channelName}): returning ${buffer.length} samples spanning ${bufferSpan.toFixed(1)}s`);
    } else {
      console.log(`getAllDataForChannel(${channelName}): no buffer found`);
    }
    return buffer || null;
  };
  
  // Recording state and functionality
  const [isRecording, setIsRecording] = useState(false);
  const [recordings, setRecordings] = useState<any[]>([]);
  const [currentRecording, setCurrentRecording] = useState<any | null>(null);
  
  const startRecording = (name?: string) => {
    setIsRecording(true);
    const newRecording = { id: Date.now().toString(), name: name || `Recording ${new Date().toLocaleString()}` };
    setCurrentRecording(newRecording);
    return newRecording;
  };
  
  const stopRecording = () => {
    setIsRecording(false);
    if (currentRecording) {
      const completed = { ...currentRecording, endTime: Date.now() };
      setRecordings(prev => [...prev, completed]);
      setCurrentRecording(null);
      return completed;
    }
    return null;
  };
  
  const deleteRecording = (id: string) => {
    setRecordings(prev => prev.filter(r => r.id !== id));
  };
  
  const renameRecording = (id: string, newName: string) => {
    setRecordings(prev => prev.map(r => r.id === id ? { ...r, name: newName } : r));
  };
  
  const getRecordingById = (id: string) => {
    return recordings.find(r => r.id === id);
  };
  
  const contextValue: DataContextType = {
    channels,
    isLoading,
    error,
    devices,
    selectedDevice,
    selectDevice,
    getAllNewData,
    getAllDataForChannel,
    isRecording,
    recordings,
    currentRecording,
    startRecording,
    stopRecording,
    deleteRecording,
    renameRecording,
    getRecordingById
  };
  
  return (
    <DataContext.Provider value={contextValue}>
      {children}
    </DataContext.Provider>
  );
};
