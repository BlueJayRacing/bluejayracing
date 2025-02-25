// src/hooks/useDataApi.ts
import { useState, useEffect, useRef } from 'react';
import { ApiService } from '../components/shared/ApiService';
import { Channel } from '../components/shared/types';
import { generateMockData } from './useDataContext';

interface ApiResponse {
  channels: Channel[];
  metadata: {
    timestamp: number;
    channel_count: number;
    max_samples_per_channel: number;
  };
}

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
        if (useMockData) {
          const mockData = generateMockData();
          
          // Debug mock data
          console.log("Generated mock data:", 
            mockData.channels.length + " channels, " + 
            mockData.channels[0].samples.length + " samples per channel"
          );
          
          // Update timestamps to be current
          const updatedChannels = mockData.channels.map(channel => ({
            ...channel,
            samples: channel.samples.map((sample, idx) => ({
              ...sample,
              timestamp: now - (99 - idx) * 100,
              // Add some random variation to the value
              value: sample.value + (Math.random() - 0.5) * (channel.max_value - channel.min_value) * 0.05
            }))
          }));
          
          setChannels(updatedChannels);
          setIsLoading(false);
          return;
        }
        
        // If not using mock data, try to fetch from API
        const response = await ApiService.getAllChannelData();
        
        if (response && response.channels) {
          console.log("API data received:", 
            response.channels.length + " channels" + 
            (response.channels.length > 0 ? ", " + response.channels[0].samples.length + " samples" : "")
          );
          
          setChannels(response.channels);
          setIsLoading(false);
          connectionAttempts.current = 0; // Reset connection attempts on success
        }
      } catch (err) {
        console.error('Error fetching data:', err);
        connectionAttempts.current += 1;
        
        // After 3 failed attempts, switch to mock data
        if (connectionAttempts.current >= 3) {
          console.log('Switching to mock data after failed API connections');
          setUseMockData(true);
          const mockData = generateMockData();
          setChannels(mockData.channels);
        }
        
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