// src/components/shared/DataContext.tsx
import React, { createContext, useState, useEffect, useContext } from 'react';
import { useDataApi } from '../../hooks/useDataApi';
import { useDataBuffer, NewDataResult } from '../../hooks/useDataBuffer';
import { Channel, TimeValue } from './types';

export interface DataContextType {
  channels: Channel[];
  selectedChannels: string[];
  isLoading: boolean;
  isRecording: boolean;
  maxDataRate: number;
  useMockData: boolean;
  getNewData: (channelName: string) => NewDataResult;
  getAllNewData: (channelNames: string[]) => Record<string, NewDataResult>;
  getAllDataForChannel: (channelName: string) => TimeValue[];
  getDataForTimeWindow: (channelName: string, startTime: number, endTime: number) => TimeValue[];
  resetChannelProcessing: (channelName: string) => void;
  resetAllProcessing: () => void;
  clearBuffer: () => void;
  setSelectedChannels: (channels: string[]) => void;
}

export const DataContext = createContext<DataContextType | null>(null);

export const DataProvider: React.FC<React.PropsWithChildren<{}>> = ({ children }) => {
  // State for data management
  const [selectedChannels, setSelectedChannels] = useState<string[]>([]);
  const [isRecording, setIsRecording] = useState(false);
  const [maxDataRate, setMaxDataRate] = useState(10); // 10 Hz default
  const [useMockData, setUseMockData] = useState(true); // Default to mock data for testing

  // Custom hooks for data fetching and buffering
  const { channels: apiChannels, isLoading, useMockData: apiUsingMock } = useDataApi();
  const { 
    bufferedData, 
    getNewData, 
    getAllNewData, 
    getAllDataForChannel, 
    getDataForTimeWindow, 
    resetChannelProcessing, 
    resetAllProcessing, 
    clearBuffer 
  } = useDataBuffer(apiChannels, maxDataRate);

  // Update useMockData state when API changes
  useEffect(() => {
    setUseMockData(apiUsingMock);
  }, [apiUsingMock]);

  // Debug data context
  useEffect(() => {
    const channelInfo = bufferedData.map(c => ({
      name: c.name,
      sampleCount: c.samples.length,
      timespan: c.samples.length > 0 ? 
        `${new Date(c.samples[0].timestamp).toISOString().split('T')[1].split('.')[0]} - ${new Date(c.samples[c.samples.length-1].timestamp).toISOString().split('T')[1].split('.')[0]}` :
        'none'
    }));
    
    // console.log("DataContext bufferedData updated:", {
    //   channelsCount: bufferedData.length,
    //   channelInfo,
    //   isLoading,
    //   useMockData
    // });
  }, [bufferedData, isLoading, useMockData]);

  return (
    <DataContext.Provider
      value={{
        channels: bufferedData,
        selectedChannels,
        isLoading,
        isRecording,
        maxDataRate,
        useMockData,
        getNewData,
        getAllNewData,
        getAllDataForChannel,
        getDataForTimeWindow,
        resetChannelProcessing,
        resetAllProcessing,
        clearBuffer,
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