// src/components/data_view/RecordingControls.tsx

import React, { useState } from 'react';
import { 
  Box, 
  Button, 
  Dialog, 
  DialogTitle, 
  DialogContent, 
  DialogActions,
  TextField,
  IconButton,
  Tooltip,
  Chip
} from '@mui/material';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import StopIcon from '@mui/icons-material/Stop';
import VisibilityIcon from '@mui/icons-material/Visibility';
import { useDataContext } from '../../hooks/useDataContext';

const RecordingControls: React.FC = () => {
  const { isRecording, startRecording, stopRecording, currentRecording } = useDataContext();
  const [recordingName, setRecordingName] = useState<string>('');
  const [showDialog, setShowDialog] = useState<boolean>(false);
  
  // Handle start recording
  const handleStartRecording = () => {
    if (recordingName.trim()) {
      startRecording(recordingName.trim());
      setRecordingName('');
      setShowDialog(false);
    } else {
      // Generate a default name
      const defaultName = `Recording ${new Date().toLocaleString()}`;
      startRecording(defaultName);
      setShowDialog(false);
    }
  };
  
  // Handle stop recording
  const handleStopRecording = () => {
    stopRecording();
  };
  
  // Format recording time
  const formatRecordingTime = (startTime: number): string => {
    const seconds = Math.floor((Date.now() - startTime) / 1000);
    
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const remainingSeconds = seconds % 60;
    
    const pad = (num: number) => num.toString().padStart(2, '0');
    
    return `${pad(hours)}:${pad(minutes)}:${pad(remainingSeconds)}`;
  };
  
  return (
    <Box display="flex" alignItems="center" gap={2}>
      {isRecording ? (
        <>
          <Chip 
            color="error"
            label={`Recording: ${formatRecordingTime(parseInt(currentRecording?.id || ''))}` }
            className="animate-pulse"
          />
          
          <Tooltip title="Stop Recording">
            <IconButton 
              color="error" 
              onClick={handleStopRecording}
              size="small"
            >
              <StopIcon />
            </IconButton>
          </Tooltip>
        </>
      ) : (
        <Button
          variant="contained"
          color="error"
          startIcon={<PlayArrowIcon />}
          onClick={() => setShowDialog(true)}
        >
          Start Recording
        </Button>
      )}
      
      <Button
        variant="outlined"
        startIcon={<VisibilityIcon />}
        component="a"
        href="/recordings"
      >
        View Recordings
      </Button>
      
      {/* Dialog for naming a recording */}
      <Dialog open={showDialog} onClose={() => setShowDialog(false)}>
        <DialogTitle>New Recording</DialogTitle>
        <DialogContent>
          <TextField
            autoFocus
            margin="dense"
            label="Recording Name"
            fullWidth
            variant="outlined"
            value={recordingName}
            onChange={(e) => setRecordingName(e.target.value)}
            placeholder="Enter a name for this recording"
          />
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowDialog(false)} color="primary">
            Cancel
          </Button>
          <Button onClick={handleStartRecording} color="primary" variant="contained">
            Start Recording
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default RecordingControls;