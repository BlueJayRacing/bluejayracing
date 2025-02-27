// src/components/data_view/RecordingControls.tsx
import React, { useState } from 'react';
import { Button, CircularProgress, Tooltip } from '@mui/material';
import { useDataContext } from '../../hooks/useDataContext';

const RecordingControls: React.FC = () => {
  const { channels } = useDataContext();
  const [isRecording, setIsRecording] = useState(false);
  const [recordingStartTime, setRecordingStartTime] = useState<number | null>(null);
  const [recordingDuration, setRecordingDuration] = useState(0);
  
  // Start recording
  const handleStartRecording = () => {
    if (isRecording) return;
    
    setIsRecording(true);
    setRecordingStartTime(Date.now());
    
    // Start timer to update duration
    const intervalId = setInterval(() => {
      if (recordingStartTime) {
        setRecordingDuration(Math.floor((Date.now() - recordingStartTime) / 1000));
      }
    }, 1000);
    
    // Store interval ID so we can clear it later
    (window as any).recordingIntervalId = intervalId;
  };
  
  // Stop recording
  const handleStopRecording = () => {
    if (!isRecording) return;
    
    setIsRecording(false);
    
    // Clear interval
    if ((window as any).recordingIntervalId) {
      clearInterval((window as any).recordingIntervalId);
      (window as any).recordingIntervalId = null;
    }
    
    // Calculate final duration
    if (recordingStartTime) {
      const finalDuration = Math.floor((Date.now() - recordingStartTime) / 1000);
      setRecordingDuration(finalDuration);
      // TODO: Save recording data
    }
    
    setRecordingStartTime(null);
  };
  
  // Check if we have valid data to record
  const hasValidData = channels && Array.isArray(channels) && channels.length > 0 && 
                      channels.some(channel => channel.samples && channel.samples.length > 0);
  
  return (
    <div className="flex items-center space-x-4">
      {isRecording && (
        <div className="flex items-center mr-2">
          <CircularProgress size={20} color="error" className="mr-2" />
          <span className="text-red-600 font-medium">
            Recording: {recordingDuration}s
          </span>
        </div>
      )}
      
      {isRecording ? (
        <Button
          variant="contained"
          color="error"
          onClick={handleStopRecording}
        >
          Stop Recording
        </Button>
      ) : (
        <Tooltip title={!hasValidData ? "No data available to record" : ""}>
          <span>
            <Button
              variant="outlined"
              color="primary"
              onClick={handleStartRecording}
              disabled={!hasValidData}
            >
              Start Recording
            </Button>
          </span>
        </Tooltip>
      )}
    </div>
  );
};

export default RecordingControls;