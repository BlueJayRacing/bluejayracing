// useRecordings.ts
import { useState, useEffect } from 'react';

export interface Recording {
  id: string;
  name: string;
  startTime: number;
  endTime: number | null;
  channelData: {
    [channelName: string]: Array<{ timestamp: number; value: number }>;
  };
  stats: {
    duration: number;
    dataSize: number; // Approximate size in bytes
    sampleCount: number;
  };
}

export const useRecordings = (channels) => {
  const [recordings, setRecordings] = useState<Recording[]>([]);
  const [isRecording, setIsRecording] = useState(false);
  const [currentRecording, setCurrentRecording] = useState<Recording | null>(null);

  // Start a new recording
  const startRecording = () => {
    if (isRecording) return;
    
    const newRecording: Recording = {
      id: Date.now().toString(),
      name: `Recording ${recordings.length + 1}`,
      startTime: Date.now(),
      endTime: null,
      channelData: Object.fromEntries(
        channels.map(channel => [channel.name, []])
      ),
      stats: {
        duration: 0,
        dataSize: 0,
        sampleCount: 0
      }
    };
    
    setCurrentRecording(newRecording);
    setIsRecording(true);
  };

  // Stop the current recording
  const stopRecording = () => {
    if (!isRecording || !currentRecording) return;
    
    const endTime = Date.now();
    const completedRecording = {
      ...currentRecording,
      endTime,
      stats: {
        duration: endTime - currentRecording.startTime,
        dataSize: calculateDataSize(currentRecording.channelData),
        sampleCount: calculateSampleCount(currentRecording.channelData)
      }
    };
    
    setRecordings([...recordings, completedRecording]);
    setCurrentRecording(null);
    setIsRecording(false);
  };

  // Rename a recording
  const renameRecording = (id: string, newName: string) => {
    setRecordings(
      recordings.map(recording => 
        recording.id === id 
          ? { ...recording, name: newName } 
          : recording
      )
    );
  };

  // Add new channel data to current recording
  useEffect(() => {
    if (!isRecording || !currentRecording || !channels.length) return;
    
    const updatedChannelData = { ...currentRecording.channelData };
    
    channels.forEach(channel => {
      const existingData = updatedChannelData[channel.name] || [];
      const latestSamples = channel.samples.filter(sample => 
        sample.timestamp > currentRecording.startTime &&
        !existingData.some(existing => existing.timestamp === sample.timestamp)
      );
      
      if (latestSamples.length > 0) {
        updatedChannelData[channel.name] = [...existingData, ...latestSamples];
      }
    });
    
    setCurrentRecording({
      ...currentRecording,
      channelData: updatedChannelData
    });
  }, [channels, isRecording, currentRecording]);

  // Helper functions
  const calculateDataSize = (channelData) => {
    // Estimate size in bytes (rough approximation)
    let totalSize = 0;
    Object.values(channelData).forEach((samples: any[]) => {
      // Each sample has timestamp (8 bytes) and value (8 bytes)
      totalSize += samples.length * 16;
    });
    return totalSize;
  };

  const calculateSampleCount = (channelData) => {
    return Object.values(channelData).reduce(
      (total, samples: any[]) => total + samples.length, 
      0
    );
  };

  return {
    recordings,
    isRecording,
    currentRecording,
    startRecording,
    stopRecording,
    renameRecording
  };
};