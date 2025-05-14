// src/hooks/useDataApi.ts
import { useState, useEffect, useRef } from 'react';
import { ApiService } from '../components/shared/ApiService';
import { Channel } from '../components/shared/DataContext';
import { DEFAULT_DEVICES } from '../config/deviceConfig';
// import { generateMockData } from './useDataContext';

// Enhanced to support device-specific data
export const useDataApi = (pollingInterval = 200) => {
  const [channels, setChannels] = useState<Channel[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);
  const [useMockData, setUseMockData] = useState(false);
  const lastFetchTime = useRef<number>(0);
  const connectionAttempts = useRef<number>(0);

  useEffect(() => {
    const fetchData = async () => {
      try {
        // Avoid fetching too frequently
        const now = Date.now();
        if (now - lastFetchTime.current < pollingInterval * 0.8) {
          return;
        }
        
        lastFetchTime.current = now;
        
        // If we're using mock data, continue using it
        // if (useMockData) {
        //   const mockData = generateMockData();
          
        //   // Update timestamps to be current
        //   const updatedChannels = mockData.channels.map(channel => ({
        //     ...channel,
        //     // Add device prefix to channel name (use first default device)
        //     name: `${DEFAULT_DEVICES[0].id}/${channel.name}`,
        //     samples: channel.samples.map((sample, idx) => ({
        //       ...sample,
        //       timestamp: now - (99 - idx) * 100,
        //       // Add some random variation to the value
        //       value: sample.value + (Math.random() - 0.5) * (channel.max_value - channel.min_value) * 0.05
        //     }))
        //   }));
          
        //   setChannels(updatedChannels);
        //   setIsLoading(false);
        //   return;
        // }
        
        // Get all data from all devices
        const response = await ApiService.getAllChannelData();
        
        if (response.success && response.data && response.data.data_chunks) {
          const deviceDataChunks = response.data.data_chunks;
          const allChannels: Channel[] = [];
          
          // Process each device data chunk
          deviceDataChunks.forEach((chunk: any) => {
            // Extract device ID from source_id
            const deviceId = chunk.source_id;
            
            // Group samples by channel ID
            const channelSamples = new Map<number, { 
              timestamps: number[], 
              values: number[] 
            }>();
            
            // Process all samples in this chunk
            chunk.samples.forEach((sample: any) => {
              // Get or create entry for this channel
              if (!channelSamples.has(sample.channel_id)) {
                channelSamples.set(sample.channel_id, { 
                  timestamps: [], 
                  values: [] 
                });
              }
              
              // Add sample data
              const channelData = channelSamples.get(sample.channel_id)!;
              channelData.timestamps.push(sample.timestamp);
              channelData.values.push(sample.data_value);
            });
            
            // Create Channel objects for each channel
            channelSamples.forEach((data, channelId) => {
              // Find channel config from default devices if available
              let channelName = `channel_${channelId}`;
              let channelType = 0;
              let minValue = 0;
              let maxValue = 100;
              
              // Look for channel in device config
              for (const device of DEFAULT_DEVICES) {
                if (device.id === deviceId) {
                  const channelConfig = device.expectedChannels.find(c => c.id === channelId);
                  if (channelConfig) {
                    channelName = channelConfig.name;
                    channelType = channelConfig.type;
                    minValue = channelConfig.min_value ?? 0;
                    maxValue = channelConfig.max_value ?? 100;
                    break;
                  }
                }
              }
              
              // Create samples array
              const samples = data.timestamps.map((timestamp, index) => ({
                timestamp,
                value: data.values[index]
              }));
              
              // Create Channel object with device prefix in name
              allChannels.push({
                name: `${deviceId}/${channelName}`,
                type: channelType,
                min_value: minValue,
                max_value: maxValue,
                samples,
                device_id: deviceId
              });
            });
          });
          
          setChannels(allChannels);
          setIsLoading(false);
          connectionAttempts.current = 0; // Reset connection attempts on success
        }
      } catch (err) {
        console.error('Error fetching data:', err);
        connectionAttempts.current += 1;
        
        // // After 3 failed attempts, switch to mock data
        // if (connectionAttempts.current >= 3) {
        //   console.log('Switching to mock data after failed API connections');
        //   setUseMockData(true);
        //   const mockData = generateMockData();
          
        //   // Add device prefix to channel names
        //   const devicePrefix = DEFAULT_DEVICES[0].id;
        //   const updatedChannels = mockData.channels.map(channel => ({
        //     ...channel,
        //     name: `${devicePrefix}/${channel.name}`
        //   }));
          
        //   setChannels(updatedChannels);
        // }
        
        setError(err instanceof Error ? err : new Error('Unknown error'));
        
        // Only set isLoading to false after we have some data (real or mock)
        if (channels.length > 0 || useMockData) {
          setIsLoading(false);
        }
      }
    };

    // Debug initial state
    console.log("useDataApi hook initialized", { pollingInterval, useMockData });

    // Fetch initial data
    fetchData();

    // Set up polling interval
    const intervalId = setInterval(fetchData, pollingInterval);

    return () => clearInterval(intervalId);
  }, [pollingInterval, isLoading, useMockData, channels.length]);

  return { 
    channels, 
    isLoading, 
    error,
    useMockData
  };
};