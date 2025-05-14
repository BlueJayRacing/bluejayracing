// src/contexts/TimestampContext.tsx
import React, { createContext, useContext, useState, useEffect } from 'react';

interface TimestampContextType {
  // Current mode (absolute or relative)
  isRelativeMode: boolean;
  
  // Toggle mode
  toggleTimestampMode: () => void;
  
  // Convert a timestamp to display format based on current mode
  convertTimestamp: (timestamp: number) => number;
  
  // Update the latest timestamp (used to compute relative time)
  updateLastTimestamp: (timestamp: number) => void;
  
  // Get a device-specific time offset
  getDeviceTimeOffset: (deviceId: string) => number;
  
  // Set a device-specific time offset
  setDeviceTimeOffset: (deviceId: string, offset: number) => void;
}

// Default context
const defaultTimestampContext: TimestampContextType = {
  isRelativeMode: false,
  toggleTimestampMode: () => {},
  convertTimestamp: (timestamp: number) => timestamp,
  updateLastTimestamp: (timestamp: number) => {},
  getDeviceTimeOffset: (deviceId: string) => 0,
  setDeviceTimeOffset: (deviceId: string, offset: number) => {}
};

// Create context
const TimestampContext = createContext<TimestampContextType>(defaultTimestampContext);

// Provider component
export const TimestampProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  // State for mode
  const [isRelativeMode, setIsRelativeMode] = useState(false);
  
  // Last timestamp received
  const [lastTimestamp, setLastTimestamp] = useState<number>(0);
  
  // Reference time for relative mode
  const [referenceTime, setReferenceTime] = useState<number>(Date.now());
  
  // Device-specific offsets to handle multiple devices with different time origins
  const [deviceOffsets, setDeviceOffsets] = useState<Record<string, number>>({});
  
  // Toggle mode
  const toggleTimestampMode = () => {
    setIsRelativeMode(!isRelativeMode);
    console.log(`Timestamp mode toggled to: ${!isRelativeMode ? 'relative' : 'absolute'}`);
  };
  
  // Update latest timestamp
  const updateLastTimestamp = (timestamp: number) => {
    // Only update if this is a newer timestamp
    if (timestamp > lastTimestamp) {
      setLastTimestamp(timestamp);
      
      // If this is the first valid timestamp, initialize reference time
      if (lastTimestamp === 0) {
        const now = Date.now();
        setReferenceTime(now);
        console.log(`Timestamp reference initialized: mapping ${timestamp} to ${now}`);
      }
    }
  };
  
  // Get device time offset
  const getDeviceTimeOffset = (deviceId: string): number => {
    return deviceOffsets[deviceId] || 0;
  };
  
  // Set device time offset
  const setDeviceTimeOffset = (deviceId: string, offset: number): void => {
    console.log(`Setting time offset for device ${deviceId}: ${offset}ms`);
    setDeviceOffsets(prev => ({
      ...prev,
      [deviceId]: offset
    }));
  };
  
  // Convert timestamp based on current mode
  const convertTimestamp = (timestamp: number): number => {
    // Handle Teensy's microsecond timestamps
    if (timestamp > 1000000000000) { // Likely microseconds
      timestamp = Math.floor(timestamp / 1000); // Convert to milliseconds
    }
    
    // For very small timestamps (likely relative time from boot in seconds)
    if (timestamp < 10000000) { // Less than about 116 days in seconds
      timestamp = timestamp * 1000; // Convert to milliseconds
    }
    
    // If timestamp is very old (before 2000), it's likely a relative timestamp
    if (timestamp < 946684800000) { // January 1, 2000
      // Map it to current time window
      // For visualization, map the earliest timestamp to (now - 30 seconds)
      // and scale all other timestamps accordingly
      if (lastTimestamp === timestamp) {
        // This is the earliest/only timestamp - map to now
        return Date.now();
      } else if (lastTimestamp > 0) {
        // Calculate the proportion of time elapsed
        const timeRange = Math.max(lastTimestamp - timestamp, 1); // Avoid division by zero
        const proportion = (lastTimestamp - timestamp) / timeRange;
        
        // Map to current time, 30 seconds in the past
        const displayRange = 30000; // 30 seconds
        return Date.now() - (proportion * displayRange);
      } else {
        // Fallback: map directly to current time
        return Date.now();
      }
    }
    
    if (isRelativeMode) {
      // In relative mode, show timestamps relative to most recent
      if (lastTimestamp > 0) {
        // Calculate difference from last timestamp
        const diff = lastTimestamp - timestamp;
        // Map to reference time
        return referenceTime - diff;
      } else {
        // Fallback to absolute time if no last timestamp
        return timestamp;
      }
    } else {
      // In absolute mode, use timestamps as-is
      return timestamp;
    }
  };
  
  // Update reference time periodically to keep relative times accurate
  useEffect(() => {
    // Only update reference time when in relative mode
    if (!isRelativeMode) return;
    
    const interval = setInterval(() => {
      setReferenceTime(Date.now());
    }, 1000); // Update every second
    
    return () => clearInterval(interval);
  }, [isRelativeMode]);
  
  // Create context value
  const contextValue: TimestampContextType = {
    isRelativeMode,
    toggleTimestampMode,
    convertTimestamp,
    updateLastTimestamp,
    getDeviceTimeOffset,
    setDeviceTimeOffset
  };
  
  return (
    <TimestampContext.Provider value={contextValue}>
      {children}
    </TimestampContext.Provider>
  );
};

// Custom hook for using the timestamp context
export const useTimestamp = () => useContext(TimestampContext);

export default TimestampContext;