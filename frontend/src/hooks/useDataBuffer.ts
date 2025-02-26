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

    // Debug buffer status
    // console.log('Buffer update triggered');
    // console.log('Buffer size setting:', bufferSize);

    // Process new data and update buffer
    const newBuffer = { ...bufferRef.current };
    
    rawChannels.forEach(channel => {
      // Initialize buffer for this channel if it doesn't exist
      if (!newBuffer[channel.name]) {
        newBuffer[channel.name] = [];
      }
      
      const existingData = newBuffer[channel.name];
      
      // Debug: log data before processing
      // console.log(`Channel ${channel.name}: Existing ${existingData.length} samples, New ${channel.samples.length} samples`);
      
      // Add new samples, ensuring no duplicates by timestamp
      let newSamplesAdded = 0;
      
      // Debug first few samples - safely
      if (channel.samples.length > 0) {
        try {
          const firstSample = channel.samples[0];
          const lastSample = channel.samples[channel.samples.length-1];
          //console.log(`First sample timestamp: ${firstSample.timestamp} (type: ${typeof firstSample.timestamp})`);
          //console.log(`Last sample timestamp: ${lastSample.timestamp} (type: ${typeof lastSample.timestamp})`);
          
          // Convert nanoseconds to milliseconds for date display
          const firstTimestampMs = Math.floor(Number(firstSample.timestamp) / 1000000);
          const lastTimestampMs = Math.floor(Number(lastSample.timestamp) / 1000000);
          
          //console.log(`First sample timestamp (converted to ms): ${firstTimestampMs}`);
          //console.log(`Last sample timestamp (converted to ms): ${lastTimestampMs}`);
          
          if (!isNaN(firstTimestampMs) && firstTimestampMs > 0) {
            //console.log(`First sample as date: ${new Date(firstTimestampMs).toLocaleString()}`);
          } else {
            //console.log(`First sample timestamp is not a valid date number`);
          }
          
          if (!isNaN(lastTimestampMs) && lastTimestampMs > 0) {
            //console.log(`Last sample as date: ${new Date(lastTimestampMs).toLocaleString()}`);
          } else {
            //console.log(`Last sample timestamp is not a valid date number`);
          }
        } catch (e) {
          //console.error("Error logging sample timestamps:", e);
        }
      }
      
      channel.samples.forEach(sample => {
        // Debug timestamp from the first few samples
        const isEarlySample = newSamplesAdded < 5;
        
        // Normalize the timestamp (ensure it's a number)
        if (typeof sample.timestamp === 'string') {
          sample.timestamp = Number(sample.timestamp);
          if (isEarlySample) {
            // console.log(`Converted string timestamp to number: ${sample.timestamp}`);
          }
        }
        
        // Skip if it's not a valid number
        if (isNaN(sample.timestamp)) {
          if (isEarlySample) {
            // console.warn(`Skipping sample with invalid timestamp: ${sample.timestamp}`);
          }
          return;
        }
        
        // Convert nanoseconds to milliseconds (if it's a nanosecond timestamp)
        const originalTimestamp = sample.timestamp;
        if (sample.timestamp > 1000000000000000) { // If timestamp is > year 33658 in milliseconds, it's likely nanoseconds
          sample.timestamp = Math.floor(sample.timestamp / 1000000); // Convert nano to milliseconds
          if (isEarlySample) {
            // console.log(`Converted nanoseconds to milliseconds: ${originalTimestamp} → ${sample.timestamp}`);
          }
        }
        
        // If it's a very small number (likely seconds), convert to milliseconds
        else if (sample.timestamp < 10000000000) { // Smaller than 2286-11-20 in milliseconds
          sample.timestamp = sample.timestamp * 1000;
          if (isEarlySample) {
            // console.log(`Converted seconds to milliseconds: ${originalTimestamp} → ${sample.timestamp}`);
          }
        }
        
        // Add to buffer if not a duplicate
        if (!existingData.find(s => s.timestamp === sample.timestamp)) {
          existingData.push(sample);
          newSamplesAdded++;
        }
      });
      
      // Debug: log new samples added
      if (newSamplesAdded > 0) {
        // console.log(`Added ${newSamplesAdded} new samples to ${channel.name}`);
      }
      
      // Sort by timestamp to ensure correct order
      existingData.sort((a, b) => a.timestamp - b.timestamp);
      
      // Take the most recent samples instead of trimming from the beginning
      // This ensures we always have the latest data
      if (existingData.length > bufferSize) {
        const excessCount = existingData.length - bufferSize;
        // console.log(`Trimming buffer for ${channel.name}: ${existingData.length} → ${bufferSize} (removing oldest ${excessCount} samples)`);
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