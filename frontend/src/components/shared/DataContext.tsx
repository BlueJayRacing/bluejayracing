// src/components/shared/DataContext.tsx
import React, { createContext, useState, useEffect, useContext } from 'react';
import { useDataApi } from '../../hooks/useDataApi';
import { useDataBuffer } from '../../hooks/useDataBuffer';
import { useRecordings } from '../../hooks/useRecordings';
import { Channel, Recording } from './types';
import { DATA_CONFIG, debugLog } from '../../config/dataConfig';

export interface DataContextType {
  channels: Channel[];
  selectedChannels: string[];
  isLoading: boolean;
  isRecording: boolean;
  currentRecording: Recording | null;
  recordings: Recording[];
  maxDataRate: number;
  useMockData: boolean;
  setSelectedChannels: (channels: string[]) => void;
  startRecording: (name?: string) => Recording | undefined;
  stopRecording: () => Recording | undefined;
  deleteRecording: (id: string) => void;
  renameRecording: (id: string, newName: string) => void;
  getRecordingById: (id: string) => Recording | undefined;
  playbackMode: boolean;
  currentPlaybackRecording: Recording | null;
  setPlaybackRecording: (recordingId: string | null) => void;
  playbackTime: number;
  setPlaybackTime: (time: number) => void;
  isPlaybackPlaying: boolean;
  setPlaybackPlaying: (playing: boolean) => void;
  playbackRate: number;
  setPlaybackRate: (rate: number) => void;
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
  const [maxDataRate, setMaxDataRate] = useState(DATA_CONFIG.API_POLLING_RATE);
  const [playbackMode, setPlaybackMode] = useState(!!initialPlaybackRecordingId);
  const [currentPlaybackRecording, setCurrentPlaybackRecording] = useState<Recording | null>(null);
  
  // Playback control state
  const [playbackTime, setPlaybackTime] = useState(0);
  const [isPlaybackPlaying, setPlaybackPlaying] = useState(false);
  const [playbackRate, setPlaybackRate] = useState(1.0);
  
  // Custom hooks for data fetching and buffering
  const { channels: apiChannels, isLoading, useMockData } = useDataApi(1000 / DATA_CONFIG.API_POLLING_RATE);
  const { bufferedData } = useDataBuffer(apiChannels, DATA_CONFIG.BUFFER_TIME_WINDOW);
  
  // Recording hook
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

  // Set up playback if an initial recording ID is provided
  useEffect(() => {
    if (initialPlaybackRecordingId) {
      const recording = recordings.find(r => r.id === initialPlaybackRecordingId);
      if (recording) {
        setCurrentPlaybackRecording(recording);
        setPlaybackMode(true);
        setPlaybackTime(0);
      }
    }
  }, [initialPlaybackRecordingId, recordings]);

  // Set playback recording
  const setPlaybackRecording = (recordingId: string | null) => {
    if (!recordingId) {
      setCurrentPlaybackRecording(null);
      setPlaybackMode(false);
      setPlaybackTime(0);
      setPlaybackPlaying(false);
      return;
    }
    
    const recording = getRecordingById(recordingId);
    if (recording) {
      setCurrentPlaybackRecording(recording);
      setPlaybackMode(true);
      setPlaybackTime(0);
      setPlaybackPlaying(false);
    } else {
      console.error(`Recording with ID ${recordingId} not found`);
    }
  };

  // Generate channels from playback recording if in playback mode
  const channels = playbackMode && currentPlaybackRecording 
    ? Object.entries(currentPlaybackRecording.channelData).map(([name, samples]) => {
        const metadata = currentPlaybackRecording.channelMetadata[name];
        
        // Filter samples based on current playback time
        const filteredSamples = samples.filter(sample => {
          const sampleTime = sample.timestamp - currentPlaybackRecording.startTime;
          return sampleTime <= playbackTime;
        });
        
        return {
          name,
          type: metadata?.type || 0,
          min_value: metadata?.min_value || 0,
          max_value: metadata?.max_value || 100,
          samples: filteredSamples,
          metadata
        };
      })
    : bufferedData;

  // Debug data context
  useEffect(() => {
    debugLog('DATA_API', "DataContext state:", {
      playbackMode,
      channelsCount: channels.length,
      isRecording,
      currentRecording: currentRecording ? {
        id: currentRecording.id,
        name: currentRecording.name,
        duration: currentRecording.stats.duration,
        sampleCount: currentRecording.stats.sampleCount
      } : null,
      samplesPerChannel: channels.length > 0 ? 
        channels.slice(0, 2).map(c => ({ name: c.name, samples: c.samples.length })) : [],
      isLoading: playbackMode ? false : isLoading,
      useMockData
    });
    
    // More detailed logging for debugging
    if (isRecording && currentRecording) {
      console.log("Active recording:", currentRecording.name);
      console.log("Recording duration:", currentRecording.stats.duration);
      console.log("Recording samples:", currentRecording.stats.sampleCount);
      console.log("Available channels:", channels.map(c => c.name).join(", "));
      console.log("Sample counts:", channels.map(c => `${c.name}: ${c.samples.length}`).join(", "));
    }
  }, [channels, isLoading, useMockData, playbackMode, isRecording, currentRecording?.stats.duration]);

  return (
    <DataContext.Provider
      value={{
        channels,
        selectedChannels,
        isLoading: playbackMode ? false : isLoading,
        isRecording,
        currentRecording,
        recordings,
        maxDataRate,
        useMockData,
        setSelectedChannels,
        startRecording,
        stopRecording,
        deleteRecording,
        renameRecording,
        getRecordingById,
        playbackMode,
        currentPlaybackRecording,
        setPlaybackRecording,
        playbackTime,
        setPlaybackTime,
        isPlaybackPlaying,
        setPlaybackPlaying,
        playbackRate,
        setPlaybackRate
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