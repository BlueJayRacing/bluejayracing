// src/components/shared/DataContext.tsx
import React, { createContext, useState, useEffect, useContext } from 'react';
import { useDataApi } from '../../hooks/useDataApi';
import { useDataBuffer } from '../../hooks/useDataBuffer';
import { Channel, Recording } from './types';

export interface DataContextType {
  channels: Channel[];
  selectedChannels: string[];
  isLoading: boolean;
  isRecording: boolean;
  maxDataRate: number;
  useMockData: boolean;
  setSelectedChannels: (channels: string[]) => void;
  // Recording functionality will be added later
}

export const DataContext = createContext<DataContextType | null>(null);

export const DataProvider: React.FC<React.PropsWithChildren<{}>> = ({ children }) => {
  // State for data management
  const [selectedChannels, setSelectedChannels] = useState<string[]>([]);
  const [isRecording, setIsRecording] = useState(false);
  const [maxDataRate, setMaxDataRate] = useState(5); // 5 Hz default

  // Custom hooks for data fetching and buffering
  const { channels, isLoading, useMockData } = useDataApi();
  const { bufferedData } = useDataBuffer(channels, maxDataRate);

  // Poll API occasionally to check max data rate
  useEffect(() => {
    const checkMaxDataRate = async () => {
      // This would normally query the API to determine max rate
      // For now, we'll use a fixed value
      setMaxDataRate(5);
    };

    // Initial check
    checkMaxDataRate();

    const interval = setInterval(checkMaxDataRate, 30000); // Check every 30 seconds
    return () => clearInterval(interval);
  }, []);

  return (
    <DataContext.Provider
      value={{
        channels: bufferedData,
        selectedChannels,
        isLoading,
        isRecording,
        maxDataRate,
        useMockData,
        setSelectedChannels
      }}
    >
      {children}
    </DataContext.Provider>
  );
};

// Custom hook to use the DataContext
export const useDataContext = () => {
  const context = useContext(DataContext);
  if (!context) {
    throw new Error('useDataContext must be used within a DataProvider');
  }
  return context;
};