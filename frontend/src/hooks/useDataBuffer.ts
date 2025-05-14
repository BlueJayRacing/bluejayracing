// src/hooks/useDataBuffer.ts
import { useState, useEffect, useRef } from 'react';
import { Channel, TimeValue } from '../components/shared/types';

// Interface for buffer data structure
interface BufferData {
  [channelName: string]: TimeValue[];
}

export interface NewDataResult {
  hasNewData: boolean;
  channelName: string;
  newSamples: TimeValue[];
}

export const useDataBuffer = (rawChannels: Channel[], pollingRate: number) => {
  const [bufferedData, setBufferedData] = useState<Channel[]>([]);
  const bufferRef = useRef<BufferData>({});
  
  // Track which data points we've already sent to charts
  const chartProcessedTimestampsRef = useRef<{[channelName: string]: Set<number>}>({});
  
  // Calculate buffer size based on polling rate
  // Aim to store approximately 60 seconds of data for better continuity (increased from 30s)
  const bufferSize = Math.max(500, pollingRate * 60); // Increased from 30 to 60 seconds and min size from 200 to 500
  
  // DEBUG: Track buffer statistics
  const bufferStatsRef = useRef<{
    lastTrimTimestamp: number;
    trimmedPoints: {[channelName: string]: number};
    totalPointsAdded: {[channelName: string]: number};
    totalPointsRequested: {[channelName: string]: number};
  }>({
    lastTrimTimestamp: Date.now(),
    trimmedPoints: {},
    totalPointsAdded: {},
    totalPointsRequested: {}
  });

  useEffect(() => {
    if (!rawChannels || rawChannels.length === 0) return;

    console.log('Buffer update triggered with', rawChannels.length, 'channels, buffer size limit:', bufferSize);

    // Process new data and update buffer
    const newBuffer = { ...bufferRef.current };
    let totalNewSamples = 0;
    
    rawChannels.forEach(channel => {
      // Initialize buffer and tracking for this channel if it doesn't exist
      if (!newBuffer[channel.name]) {
        newBuffer[channel.name] = [];
        console.log(`Initializing buffer for channel: ${channel.name}`);
      }
      
      if (!chartProcessedTimestampsRef.current[channel.name]) {
        chartProcessedTimestampsRef.current[channel.name] = new Set();
      }
      
      // Initialize buffer stats tracking
      if (!bufferStatsRef.current.trimmedPoints[channel.name]) {
        bufferStatsRef.current.trimmedPoints[channel.name] = 0;
      }
      if (!bufferStatsRef.current.totalPointsAdded[channel.name]) {
        bufferStatsRef.current.totalPointsAdded[channel.name] = 0;
      }
      if (!bufferStatsRef.current.totalPointsRequested[channel.name]) {
        bufferStatsRef.current.totalPointsRequested[channel.name] = 0;
      }
      
      const existingData = newBuffer[channel.name];
      const existingTimestamps = new Set(existingData.map(sample => sample.timestamp));
      
      // Add new samples, ensuring no duplicates by timestamp
      let newSamplesAdded = 0;
      
      channel.samples.forEach(sample => {
        // Normalize the timestamp (ensure it's a number)
        if (typeof sample.timestamp === 'string') {
          sample.timestamp = Number(sample.timestamp);
        }
        
        // Skip if it's not a valid number
        if (isNaN(sample.timestamp)) {
          return;
        }
        
        // Handle timestamp conversions - store original for reference
        const originalTimestamp = sample.timestamp;
        
        // Convert nanoseconds to milliseconds (if it's a nanosecond timestamp)
        if (sample.timestamp > 1000000000000000) { // If timestamp is > year 33658 in milliseconds, it's likely nanoseconds
          sample.timestamp = Math.floor(sample.timestamp / 1000000); // Convert nano to milliseconds
        }
        
        // If it's a very small number (likely seconds), convert to milliseconds
        else if (sample.timestamp < 10000000000) { // Smaller than 2286-11-20 in milliseconds
          sample.timestamp = sample.timestamp * 1000;
        }
        
        // Add to buffer if not already there (using Set for faster lookups)
        if (!existingTimestamps.has(sample.timestamp)) {
          existingData.push({
            timestamp: sample.timestamp,
            value: sample.value
          });
          existingTimestamps.add(sample.timestamp);
          newSamplesAdded++;
          totalNewSamples++;
          
          // Track for debug stats
          bufferStatsRef.current.totalPointsAdded[channel.name]++;
        }
      });
      
      if (newSamplesAdded > 0) {
        console.log(`Added ${newSamplesAdded} new samples to ${channel.name}`);
        
        // Always sort by timestamp to ensure correct order
        existingData.sort((a, b) => a.timestamp - b.timestamp);
        
        // Keep buffer size in check
        if (existingData.length > bufferSize) {
          const excessCount = existingData.length - bufferSize;
          console.log(`Trimming buffer for ${channel.name}: removing ${excessCount} oldest samples, keeping ${bufferSize}`);
          
          // Get timestamps being removed
          const removedTimestamps = existingData.slice(0, excessCount).map(d => d.timestamp);
          
          // Track earliest and latest timestamps being removed
          if (removedTimestamps.length > 0) {
            const earliestRemoved = new Date(removedTimestamps[0]).toISOString();
            const latestRemoved = new Date(removedTimestamps[removedTimestamps.length-1]).toISOString();
            console.log(`Trimmed time range: ${earliestRemoved} to ${latestRemoved}`);
          }
          
          // Drop oldest data points
          existingData.splice(0, excessCount);
          
          // Update the existing timestamps Set to match what we've kept
          removedTimestamps.forEach(ts => existingTimestamps.delete(ts));
          
          // Track trimmed points for stats
          bufferStatsRef.current.trimmedPoints[channel.name] += excessCount;
          bufferStatsRef.current.lastTrimTimestamp = Date.now();
        }
        
        newBuffer[channel.name] = existingData;
      }
    });
    
    bufferRef.current = newBuffer;
    
    // Format buffered data back into Channel format for components
    const formattedData = rawChannels.map(channel => {
      const samples = newBuffer[channel.name] || [];
      
      // Ensure data series is ordered and continuous
      return {
        ...channel,
        samples: [...samples] // Return a copy to prevent mutation
      };
    });
    
    // Log buffer stats periodically (every 10 seconds)
    const now = Date.now();
    if (now - bufferStatsRef.current.lastTrimTimestamp > 10000) {
      console.log(`Buffer stats: ${totalNewSamples} new samples total`);
      
      // Log stats for channels with significant data
      Object.keys(bufferStatsRef.current.totalPointsAdded)
        .filter(channel => bufferStatsRef.current.totalPointsAdded[channel] > 0)
        .forEach(channel => {
          const added = bufferStatsRef.current.totalPointsAdded[channel];
          const trimmed = bufferStatsRef.current.trimmedPoints[channel];
          const requested = bufferStatsRef.current.totalPointsRequested[channel];
          const current = newBuffer[channel]?.length || 0;
          
          console.log(`${channel}: added=${added}, trimmed=${trimmed}, requested=${requested}, current=${current}`);
          
          if (newBuffer[channel] && newBuffer[channel].length > 0) {
            const oldestPoint = new Date(newBuffer[channel][0].timestamp).toISOString();
            const newestPoint = new Date(newBuffer[channel][newBuffer[channel].length-1].timestamp).toISOString();
            console.log(`  Time range: ${oldestPoint} to ${newestPoint}`);
          }
        });
      
      // Reset trim timestamp for next periodic log
      bufferStatsRef.current.lastTrimTimestamp = now;
    }
    
    setBufferedData(formattedData);
  }, [rawChannels, bufferSize]);

  // Function to get new data for a specific channel since last retrieval
  const getNewData = (channelName: string): NewDataResult => {
    const channel = bufferedData.find(c => c.name === channelName);
    
    if (!channel || !channel.samples || channel.samples.length === 0) {
      return { hasNewData: false, channelName, newSamples: [] };
    }
    
    // Get the processed timestamps for this chart
    const processedTimestamps = chartProcessedTimestampsRef.current[channelName] || new Set();
    
    // Find samples we haven't sent to the chart yet
    const newSamples = channel.samples.filter(sample => 
      !processedTimestamps.has(sample.timestamp)
    );
    
    // Update stats
    if (!bufferStatsRef.current.totalPointsRequested[channelName]) {
      bufferStatsRef.current.totalPointsRequested[channelName] = 0;
    }
    bufferStatsRef.current.totalPointsRequested[channelName] += newSamples.length;
    
    // If we have new samples, mark them as processed
    if (newSamples.length > 0) {
      // Create a new processed set if needed
      if (!chartProcessedTimestampsRef.current[channelName]) {
        chartProcessedTimestampsRef.current[channelName] = new Set();
      }
      
      // Mark these samples as processed for the chart
      newSamples.forEach(sample => {
        chartProcessedTimestampsRef.current[channelName].add(sample.timestamp);
      });
      
      console.log(`getNewData for ${channelName}: found ${newSamples.length} new samples`);
      
      // Log time range of new samples
      if (newSamples.length > 0) {
        const firstSample = new Date(newSamples[0].timestamp).toISOString();
        const lastSample = new Date(newSamples[newSamples.length - 1].timestamp).toISOString();
        console.log(`  New samples time range: ${firstSample} to ${lastSample}`);
      }
      
      return {
        hasNewData: true,
        channelName,
        newSamples: [...newSamples]
      };
    }
    
    return { hasNewData: false, channelName, newSamples: [] };
  };

  // Function to get new data for multiple channels
  const getAllNewData = (channelNames: string[]): Record<string, NewDataResult> => {
    const result: Record<string, NewDataResult> = {};
    
    channelNames.forEach(channelName => {
      result[channelName] = getNewData(channelName);
    });
    
    const newDataCount = Object.values(result).filter(r => r.hasNewData).length;
    const totalNewPoints = Object.values(result).reduce((sum, r) => sum + r.newSamples.length, 0);
    
    if (newDataCount > 0) {
      console.log(`getAllNewData for ${channelNames.length} channels: ${newDataCount} channels have new data (${totalNewPoints} points)`);
    }
    
    return result;
  };

  // Reset the processed state for a channel to force retrieving all data
  const resetChannelProcessing = (channelName: string): void => {
    console.log(`Resetting processing state for channel: ${channelName}`);
    chartProcessedTimestampsRef.current[channelName] = new Set();
  };

  // Reset all processed timestamps
  const resetAllProcessing = (): void => {
    console.log('Resetting processing state for all channels');
    Object.keys(chartProcessedTimestampsRef.current).forEach(key => {
      chartProcessedTimestampsRef.current[key] = new Set();
    });
  };

  // Get all available data for a channel
  const getAllDataForChannel = (channelName: string): TimeValue[] => {
    const channel = bufferedData.find(c => c.name === channelName);
    if (!channel || !channel.samples) return [];
    
    console.log(`getAllDataForChannel ${channelName}: returning ${channel.samples.length} samples`);
    
    // Log time range
    if (channel.samples.length > 0) {
      const firstSample = new Date(channel.samples[0].timestamp).toISOString();
      const lastSample = new Date(channel.samples[channel.samples.length - 1].timestamp).toISOString();
      console.log(`  Time range: ${firstSample} to ${lastSample}`);
    }
    
    // Return a copy of all samples
    return [...channel.samples];
  };

  // Get data slice for a specific time window
  const getDataForTimeWindow = (channelName: string, startTime: number, endTime: number): TimeValue[] => {
    const channel = bufferedData.find(c => c.name === channelName);
    if (!channel || !channel.samples) return [];
    
    const filteredData = channel.samples.filter(sample => 
      sample.timestamp >= startTime && sample.timestamp <= endTime
    );
    
    console.log(`getDataForTimeWindow ${channelName}: returning ${filteredData.length}/${channel.samples.length} samples in window`);
    
    return filteredData;
  };

  return { 
    bufferedData,
    getNewData,
    getAllNewData,
    getAllDataForChannel,
    getDataForTimeWindow,
    resetChannelProcessing,
    resetAllProcessing,
    clearBuffer: () => {
      console.log('Clearing all buffer data');
      bufferRef.current = {};
      chartProcessedTimestampsRef.current = {};
      setBufferedData([]);
      
      // Reset stats
      Object.keys(bufferStatsRef.current.trimmedPoints).forEach(key => {
        bufferStatsRef.current.trimmedPoints[key] = 0;
        bufferStatsRef.current.totalPointsAdded[key] = 0;
        bufferStatsRef.current.totalPointsRequested[key] = 0;
      });
    },
    getBufferSize: () => bufferSize
  };
};