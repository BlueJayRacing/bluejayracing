// src/hooks/useDataBuffer.ts
import { useState, useEffect, useRef } from 'react';
import { Channel, TimeValue } from '../components/shared/types';

// Interface for buffer data structure
interface BufferData {
  [channelName: string]: TimeValue[];
}

export const useDataBuffer = (rawChannels: Channel[], pollingRate: number) => {
  const [bufferedData, setBufferedData] = useState<Channel[]>([]);
  const bufferRef = useRef<BufferData>({});
  
  // Calculate buffer size based on polling rate
  // Aim to store approximately 20 seconds of data
  const bufferSize = Math.max(100, pollingRate * 20);

  useEffect(() => {
    if (!rawChannels || rawChannels.length === 0) return;

    // Process new data and update buffer
    const newBuffer = { ...bufferRef.current };
    
    rawChannels.forEach(channel => {
      // Initialize buffer for this channel if it doesn't exist
      if (!newBuffer[channel.name]) {
        newBuffer[channel.name] = [];
      }
      
      const existingData = newBuffer[channel.name];
      
      // Add new samples, ensuring no duplicates by timestamp
      channel.samples.forEach(sample => {
        if (!existingData.find(s => s.timestamp === sample.timestamp)) {
          existingData.push(sample);
        }
      });
      
      // Sort by timestamp to ensure correct order
      existingData.sort((a, b) => a.timestamp - b.timestamp);
      
      // Trim to buffer size to prevent memory growth
      if (existingData.length > bufferSize) {
        newBuffer[channel.name] = existingData.slice(-bufferSize);
      } else {
        newBuffer[channel.name] = existingData;
      }
    });
    
    bufferRef.current = newBuffer;
    
    // Format buffered data back into Channel format for components
    const formattedData = rawChannels.map(channel => ({
      ...channel,
      samples: newBuffer[channel.name] || []
    }));
    
    setBufferedData(formattedData);
  }, [rawChannels, bufferSize]);

  return { 
    bufferedData,
    // Helper methods for buffer management
    clearBuffer: () => {
      bufferRef.current = {};
      setBufferedData([]);
    },
    getBufferSize: () => bufferSize
  };
};