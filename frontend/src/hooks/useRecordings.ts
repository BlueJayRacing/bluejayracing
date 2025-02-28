// src/hooks/useRecordings.ts
import { useState, useEffect, useRef } from 'react';
import { Channel, Recording, ChannelMetadata } from '../components/shared/types';
import { DATA_CONFIG, debugLog, logSummary } from '../config/dataConfig';
import { safeLocalStorage } from '../utils/dataTypes';

export const useRecordings = (channels: Channel[]) => {
  const [recordings, setRecordings] = useState<Recording[]>([]);
  const [isRecording, setIsRecording] = useState(false);
  const [currentRecording, setCurrentRecording] = useState<Recording | null>(null);
  
  // Track memory usage
  const memoryUsageRef = useRef<number>(0);
  // Track samples added per update (for logging)
  const batchSamplesRef = useRef<number>(0);
  // Track last log time to avoid excessive logging
  const lastLogTimeRef = useRef<number>(0);
  
  // Try to load saved recordings from localStorage on first load
  useEffect(() => {
    const loadedRecordings = safeLocalStorage.getObject<Recording[]>('recordings', []);
    
    // Filter out invalid recordings
    const validRecordings = loadedRecordings.filter(rec => 
      rec && rec.id && rec.startTime && rec.channelData
    );
    
    if (validRecordings.length > 0) {
      logSummary('RECORDING', `Loaded ${validRecordings.length} recordings from localStorage`);
      setRecordings(validRecordings);
    }
  }, []);
  
  // Save recordings to localStorage when they change
  useEffect(() => {
    if (recordings.length > 0) {
      const success = safeLocalStorage.setObject('recordings', recordings);
      if (success) {
        logSummary('RECORDING', `Saved ${recordings.length} recordings to localStorage`);
      }
    }
  }, [recordings]);
  
  // Start a new recording
  const startRecording = (name?: string) => {
    if (isRecording) return;
    
    const startTime = Date.now();
    const channelMetadata: { [key: string]: ChannelMetadata } = {};
    
    // If no name provided, create one with date and time
    const autoName = name || `Recording ${new Date(startTime).toLocaleString([], {
      year: 'numeric',
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    })}`;
    
    // Extract metadata from channels
    channels.forEach(channel => {
      if (channel.metadata) {
        channelMetadata[channel.name] = channel.metadata;
      } else {
        // Create basic metadata if not available
        channelMetadata[channel.name] = {
          name: channel.name,
          type: channel.type,
          sample_rate: 2000, // Default assumption
          transmission_rate: 100, // Default assumption
          location: "",
          units: "",
          description: `${channel.name} data channel`,
          min_value: channel.min_value,
          max_value: channel.max_value
        };
      }
    });
    
    // Initialize new recording
    const newRecording: Recording = {
      id: `rec_${startTime}_${Math.floor(Math.random() * 10000)}`,
      name: autoName,
      startTime,
      endTime: null,
      channelData: Object.fromEntries(
        channels.map(channel => [channel.name, []])
      ),
      channelMetadata,
      stats: {
        duration: 0,
        dataSize: 0,
        sampleCount: 0,
        channelCount: channels.length,
        maxSampleRate: Math.max(...channels.map(c => c.metadata?.sample_rate || 2000)),
        averageSampleRate: channels.reduce((sum, c) => sum + (c.metadata?.sample_rate || 2000), 0) / 
                           Math.max(1, channels.length)
      }
    };
    
    memoryUsageRef.current = 0;
    batchSamplesRef.current = 0;
    lastLogTimeRef.current = Date.now();
    
    setCurrentRecording(newRecording);
    setIsRecording(true);
    
    logSummary('RECORDING', 'Started recording:', newRecording.name);
    return newRecording;
  };

  // Stop the current recording
  const stopRecording = () => {
    if (!isRecording || !currentRecording) return;
    
    const endTime = Date.now();
    const duration = endTime - currentRecording.startTime;
    
    // Calculate final stats
    const sampleCount = Object.values(currentRecording.channelData)
                              .reduce((sum, samples) => sum + samples.length, 0);
    
    const completedRecording = {
      ...currentRecording,
      endTime,
      stats: {
        ...currentRecording.stats,
        duration,
        sampleCount,
        dataSize: memoryUsageRef.current
      }
    };
    
    setRecordings([...recordings, completedRecording]);
    setCurrentRecording(null);
    setIsRecording(false);
    
    logSummary('RECORDING', 'Stopped recording:', completedRecording.name, {
      duration: formatDuration(duration),
      samples: sampleCount,
      size: formatFileSize(memoryUsageRef.current)
    });
    
    // Save to localStorage immediately
    try {
      const updatedRecordings = [...recordings, completedRecording];
      localStorage.setItem('recordings', JSON.stringify(updatedRecordings));
    } catch (error) {
      console.error('Error saving recordings to localStorage:', error);
    }
    
    return completedRecording;
  };

  // Delete a recording
  const deleteRecording = (id: string) => {
    // Validate that the recording exists before attempting to delete
    const recordingExists = recordings.some(rec => rec && rec.id === id);
    if (!recordingExists) {
      console.warn(`Attempted to delete non-existent recording with ID: ${id}`);
      return;
    }
    
    const updatedRecordings = recordings.filter(recording => 
      recording && recording.id && recording.id !== id
    );
    
    setRecordings(updatedRecordings);
    logSummary('RECORDING', `Deleted recording: ${id}`);
    
    // Update localStorage immediately
    try {
      localStorage.setItem('recordings', JSON.stringify(updatedRecordings));
    } catch (error) {
      console.error('Error saving recordings to localStorage after deletion:', error);
    }
  };

  // Rename a recording
  const renameRecording = (id: string, newName: string) => {
    const updatedRecordings = recordings.map(recording => 
      recording.id === id ? { ...recording, name: newName } : recording
    );
    
    setRecordings(updatedRecordings);
    debugLog('RECORDING', `Renamed recording ${id} to "${newName}"`);
    
    // Update localStorage immediately
    try {
      localStorage.setItem('recordings', JSON.stringify(updatedRecordings));
    } catch (error) {
      console.error('Error saving recordings to localStorage after rename:', error);
    }
  };

  // Add new data to current recording
  useEffect(() => {
    if (!isRecording || !currentRecording || !channels.length) return;
    
    // Check memory limits
    const memoryLimitBytes = DATA_CONFIG.MAX_RECORDING_SIZE_MB * 1024 * 1024;
    if (memoryUsageRef.current > memoryLimitBytes) {
      logSummary('RECORDING', 'Memory limit reached, stopping recording');
      stopRecording();
      return;
    }
    
    // Check time limits
    const currentDuration = Date.now() - currentRecording.startTime;
    if (currentDuration > DATA_CONFIG.MAX_RECORDING_DURATION_SEC * 1000) {
      logSummary('RECORDING', 'Time limit reached, stopping recording');
      stopRecording();
      return;
    }
    
    // Update recording stats even if no new data (to update duration)
    const updatedRecording = {
      ...currentRecording,
      stats: {
        ...currentRecording.stats,
        duration: currentDuration
      }
    };
    
    // Update recording with new data
    let newDataSize = 0;
    const updatedChannelData = { ...currentRecording.channelData };
    let addedSamples = false;
    let totalNewSamples = 0;
    
    // Only log channel details periodically to reduce logging
    const shouldLogDetails = Date.now() - lastLogTimeRef.current > 5000; // Every 5 seconds
    
    channels.forEach(channel => {
      if (!channel.samples.length) return;
      
      const existingData = updatedChannelData[channel.name] || [];
      
      // Get existing timestamps for faster lookup
      const existingTimestamps = new Set(existingData.map(sample => sample.timestamp));
      
      // Only add new samples that occurred after recording started
      const newSamples = channel.samples.filter(sample => 
        sample.timestamp > currentRecording.startTime && 
        !existingTimestamps.has(sample.timestamp)
      );
      
      if (newSamples.length > 0) {
        // Only log detailed channel info if enabled and time threshold met
        if (shouldLogDetails) {
          debugLog('RECORDING', `Channel ${channel.name}: ${newSamples.length} new samples`);
        }
        
        updatedChannelData[channel.name] = [...existingData, ...newSamples];
        
        // Estimate memory: 16 bytes per sample (8 for timestamp, 8 for value)
        newDataSize += newSamples.length * 16;
        totalNewSamples += newSamples.length;
        addedSamples = true;
      }
    });
    
    if (shouldLogDetails) {
      lastLogTimeRef.current = Date.now();
    }
    
    // Update memory usage tracking
    memoryUsageRef.current += newDataSize;
    batchSamplesRef.current += totalNewSamples;
    
    // Always update duration, but only update data if there are new samples
    if (addedSamples) {
      updatedRecording.channelData = updatedChannelData;
      updatedRecording.stats.dataSize = memoryUsageRef.current;
      updatedRecording.stats.sampleCount = Object.values(updatedChannelData)
        .reduce((sum, samples) => sum + samples.length, 0);
      
      // Only log summary periodically or for large batches
      if (shouldLogDetails || totalNewSamples > 50) {
        logSummary('RECORDING', `Recording update: +${batchSamplesRef.current} samples, total: ${updatedRecording.stats.sampleCount}`);
        batchSamplesRef.current = 0;
      }
    }
    
    setCurrentRecording(updatedRecording);
  }, [channels, isRecording, currentRecording]);

  // Get recording by ID with validation
  const getRecordingById = (id: string): Recording | undefined => {
    const recording = recordings.find(rec => rec && rec.id === id);
    
    // Validate the recording has required properties
    if (recording && recording.id && recording.startTime && recording.channelData) {
      return recording;
    }
    
    return undefined;
  };

  // Helper functions
  const formatDuration = (ms: number): string => {
    const seconds = Math.floor(ms / 1000);
    if (seconds < 60) return `${seconds} seconds`;
    const minutes = Math.floor(seconds / 60);
    if (minutes < 60) return `${minutes} minutes, ${seconds % 60} seconds`;
    const hours = Math.floor(minutes / 60);
    return `${hours} hours, ${minutes % 60} minutes`;
  };

  const formatFileSize = (bytes: number): string => {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  };

  return {
    recordings,
    isRecording,
    currentRecording,
    startRecording,
    stopRecording,
    deleteRecording,
    renameRecording,
    getRecordingById
  };
};