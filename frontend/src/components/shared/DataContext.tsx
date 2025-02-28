// src/components/shared/DataContext.tsx
import React, { createContext, useState, useEffect, useContext } from 'react';
import { useDataApi } from '../../hooks/useDataApi';
import { useDataBuffer, NewDataResult } from '../../hooks/useDataBuffer';
import { useRecordings } from '../../hooks/useRecordings';
import { Channel, TimeValue, Recording } from './types';

export interface DataContextType {
  channels: Channel[];
  selectedChannels: string[];
  isLoading: boolean;
  isRecording: boolean;
  currentRecording: Recording | null;
  recordings: Recording[];
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
  startRecording: (name?: string) => void;
  stopRecording: () => void;
  deleteRecording: (id: string) => void;
  renameRecording: (id: string, newName: string) => void;
  getRecordingById: (id: string) => Recording | undefined;
}

export const DataContext = createContext<DataContextType | null>(null);

interface DataProviderProps {
  children: React.ReactNode;
  initialPlaybackRecordingId?: string | null;
}

export const DataProvider: React.FC<DataProviderProps> = ({ 
  children, 
  initialPlaybackRecordingId = null 
}) => {
  // State for data management
  const [selectedChannels, setSelectedChannels] = useState<string[]>([]);
  const [maxDataRate, setMaxDataRate] = useState(10); // 10 Hz default

  // Custom hooks for data fetching and buffering
  const { channels: apiChannels, isLoading, useMockData } = useDataApi();
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
  
  // Hook for recording management
  const {
    recordings,
    isRecording,
    currentRecording,
    startRecording,
    stopRecording,
    deleteRecording,
    renameRecording,
    getRecordingById
  } = useRecordings(bufferedData);

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
        currentRecording,
        recordings,
        maxDataRate,
        useMockData,
        getNewData,
        getAllNewData,
        getAllDataForChannel,
        getDataForTimeWindow,
        resetChannelProcessing,
        resetAllProcessing,
        clearBuffer,
        setSelectedChannels,
        startRecording,
        stopRecording,
        deleteRecording,
        renameRecording,
        getRecordingById
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