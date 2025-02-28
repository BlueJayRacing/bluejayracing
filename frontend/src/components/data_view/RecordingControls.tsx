// src/components/data_view/RecordingControls.tsx
import React, { useState, useEffect } from 'react';
import { Button, Dialog, DialogTitle, DialogContent, DialogActions, TextField, Box, Chip, CircularProgress } from '@mui/material';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import StopIcon from '@mui/icons-material/Stop';
import { useDataContext } from '../../hooks/useDataContext';
import { Link } from 'react-router-dom';

const RecordingControls: React.FC = () => {
  const { isRecording, currentRecording, startRecording, stopRecording, recordings } = useDataContext();
  const [elapsedTime, setElapsedTime] = useState(0);
  const [recordingStats, setRecordingStats] = useState({
    sampleCount: 0,
    dataSize: 0
  });
  
  // Update elapsed time during recording
  useEffect(() => {
    if (!isRecording || !currentRecording) {
      setElapsedTime(0);
      return;
    }
    
    // Initial update
    setElapsedTime(Date.now() - currentRecording.startTime);
    if (currentRecording.stats) {
      setRecordingStats({
        sampleCount: currentRecording.stats.sampleCount,
        dataSize: currentRecording.stats.dataSize
      });
    }
    
    // Regular updates using Animation Frame for smoother UI
    let animationId: number;
    const updateTime = () => {
      setElapsedTime(Date.now() - currentRecording.startTime);
      
      // Update stats (every ~500ms to avoid over-updating)
      if (Date.now() % 500 < 50 && currentRecording.stats) {
        setRecordingStats({
          sampleCount: currentRecording.stats.sampleCount,
          dataSize: currentRecording.stats.dataSize
        });
      }
      
      animationId = requestAnimationFrame(updateTime);
    };
    
    animationId = requestAnimationFrame(updateTime);
    
    return () => {
      if (animationId) {
        cancelAnimationFrame(animationId);
      }
    };
  }, [isRecording, currentRecording]);
  
  const handleStartClick = () => {
    // Generate automatic name with date and time
    const nowDate = new Date();
    const formattedDate = nowDate.toLocaleDateString();
    const formattedTime = nowDate.toLocaleTimeString([], {
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    });
    
    const autoName = `Recording ${formattedDate} ${formattedTime}`;
    startRecording(autoName);
  };
  
  const handleStopClick = () => {
    stopRecording();
    setElapsedTime(0);
  };
  
  // Format time display
  const formatElapsedTime = (ms: number) => {
    const seconds = Math.floor(ms / 1000);
    const minutes = Math.floor(seconds / 60);
    const hours = Math.floor(minutes / 60);
    
    return [
      hours > 0 ? String(hours).padStart(2, '0') : '',
      String(minutes % 60).padStart(2, '0'),
      String(seconds % 60).padStart(2, '0')
    ].filter(Boolean).join(':');
  };
  
  // Format file size
  const formatFileSize = (bytes: number) => {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  };
  
  return (
    <div>
      {isRecording ? (
        <Box display="flex" alignItems="center">
          <Box mr={2} display="flex" alignItems="center">
            <Chip 
              color="error"
              icon={<CircularProgress size={16} color="inherit" />}
              label={`Recording: ${formatElapsedTime(elapsedTime)}`}
              sx={{
                animation: 'pulse 2s infinite',
                '@keyframes pulse': {
                  '0%': { opacity: 0.8 },
                  '50%': { opacity: 1 },
                  '100%': { opacity: 0.8 }
                }
              }}
            />
          </Box>
          <Box mr={2}>
            <Chip 
              variant="outlined"
              label={`${recordingStats.sampleCount.toLocaleString()} samples`}
              size="small"
            />
          </Box>
          <Box mr={2}>
            <Chip 
              variant="outlined"
              label={formatFileSize(recordingStats.dataSize)}
              size="small"
            />
          </Box>
          <Button
            variant="contained"
            color="error"
            startIcon={<StopIcon />}
            onClick={handleStopClick}
          >
            Stop Recording
          </Button>
        </Box>
      ) : (
        <Box display="flex" alignItems="center">
          <Button
            variant="contained"
            color="primary"
            startIcon={<PlayArrowIcon />}
            onClick={handleStartClick}
            sx={{ mr: 2 }}
          >
            Start Recording
          </Button>
          
          {recordings.length > 0 && (
            <Button
              variant="outlined"
              component={Link}
              to="/recordings"
            >
              View Recordings ({recordings.length})
            </Button>
          )}
        </Box>
      )}
    </div>
  );
};

export default RecordingControls;