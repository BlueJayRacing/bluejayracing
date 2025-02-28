// src/pages/PlaybackPage.tsx
import React, { useEffect, useState } from 'react';
import { useParams, useNavigate, Link } from 'react-router-dom';
import { Container, Typography, Box, Button, Paper, Alert, CircularProgress, AppBar, Toolbar, IconButton } from '@mui/material';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import HomeIcon from '@mui/icons-material/Home';
import { useDataContext } from '../hooks/useDataContext';
import RecordingPlayback from '../components/recording_view/RecordingPlayback';
import { Recording } from '../components/shared/types';
import { safeLocalStorage } from '../utils/dataTypes';

const PlaybackPage: React.FC = () => {
const { recordingId } = useParams<{ recordingId: string }>();
const navigate = useNavigate();
const { recordings, getRecordingById } = useDataContext();
const [recording, setRecording] = useState<Recording | null>(null);
const [loading, setLoading] = useState(true);
const [error, setError] = useState<string | null>(null);
const [recordingInitialized, setRecordingInitialized] = useState(false);

// Try to load saved recordings from localStorage
const loadRecordingsFromStorage = () => {
  return safeLocalStorage.getObject<Recording[]>('recordings', []);
};

useEffect(() => {
  // console.log("PlaybackPage useEffect run");
  // Don't re-run if already initialized
  if (recordingInitialized) {
    // console.log("Recording already initialized, skipping load");
    return;
  }
  
  // Properly handle loading of a recording by ID
  const loadRecording = () => {
    // console.log("Loading recording with ID:", recordingId);
    if (!recordingId) {
      setError('No recording ID provided');
      setLoading(false);
      setRecordingInitialized(true);
      return;
    }
    
    try {
      // First try to get recording from context
      let foundRecording = getRecordingById ? getRecordingById(recordingId) : 
        recordings.find(r => r.id === recordingId);
        
      // Validate the recording
      if (foundRecording && foundRecording.id && foundRecording.startTime && foundRecording.channelData) {
        // Deep validate recording structure
        const isValid = validateRecording(foundRecording);
        if (isValid) {
          setRecording(foundRecording);
          setLoading(false);
          return;
        }
      }
      
      // Try to load from localStorage directly as fallback
      try {
        const storedRecordings = loadRecordingsFromStorage();
        if (storedRecordings.length > 0) {
          const storedRecording = storedRecordings.find(r => r.id === recordingId);
          if (storedRecording && validateRecording(storedRecording)) {
            setRecording(storedRecording);
            setLoading(false);
            return;
          }
        }
      } catch (storageErr) {
        console.error('Error loading recording from localStorage:', storageErr);
      }
      
      setError(`Recording with ID ${recordingId} not found or is invalid`);
      setLoading(false);
      setRecordingInitialized(true);
    } catch (err) {
      console.error('Error finding recording:', err);
      setError(`Error loading recording: ${err.message}`);
      setLoading(false);
      setRecordingInitialized(true);
    }
  };
  
  // Helper function to validate recording structure
  const validateRecording = (recording: any): boolean => {
    if (!recording || typeof recording !== 'object') return false;
    if (!recording.id || !recording.startTime) return false;
    if (!recording.channelData || typeof recording.channelData !== 'object') return false;
    
    // Ensure at least some channel data exists
    const channels = Object.keys(recording.channelData);
    if (channels.length === 0) return false;
    
    // Ensure stats object exists
    if (!recording.stats || typeof recording.stats !== 'object') {
      // Create minimal stats if missing
      recording.stats = {
        duration: recording.endTime ? (recording.endTime - recording.startTime) : 0,
        dataSize: 0,
        sampleCount: 0,
        channelCount: channels.length
      };
    }
    
    // Ensure channelMetadata exists
    if (!recording.channelMetadata) {
      recording.channelMetadata = {};
    }
    
    return true;
  };
  
  // Short delay to ensure recordings are loaded from localStorage
  const timer = setTimeout(loadRecording, 50);
  
  return () => clearTimeout(timer);
}, [recordingId, getRecordingById, recordings, recordingInitialized]);

const handleClose = () => {
  navigate('/recordings');
};

// Set page title
useEffect(() => {
  if (recording) {
    document.title = `Playback: ${recording.name}`;
  } else {
    document.title = 'Recording Playback';
  }
  
  return () => {
    document.title = 'BlueJay Racing';
  };
}, [recording]);

if (loading) {
  return (
    <Container maxWidth="lg">
      <Box 
        display="flex" 
        justifyContent="center" 
        alignItems="center" 
        minHeight="50vh"
      >
        <CircularProgress />
        <Typography variant="h6" sx={{ ml: 2 }}>
          Loading recording...
        </Typography>
      </Box>
    </Container>
  );
}

if (error || !recording) {
  return (
    <Container maxWidth="lg">
      <Box py={4}>
        <Box display="flex" alignItems="center" mb={4}>
          <Button
            component={Link}
            to="/recordings"
            startIcon={<ArrowBackIcon />}
            sx={{ mr: 2 }}
          >
            Back to Recordings
          </Button>
          <Typography variant="h4" component="h1">
            Playback Error
          </Typography>
        </Box>
        
        <Alert severity="error" sx={{ mb: 4 }}>
          {error || 'Recording not found. It may have been deleted.'}
        </Alert>
        
        <Button 
          variant="contained"
          component={Link}
          to="/recordings"
        >
          Return to Recordings
        </Button>
      </Box>
    </Container>
  );
}

return (
  <>
    <AppBar position="static" color="transparent" elevation={0}>
      <Toolbar>
        <Button
          startIcon={<ArrowBackIcon />}
          component={Link}
          to="/recordings"
          sx={{ mr: 2 }}
        >
          Back to Recordings
        </Button>
        
        <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
          Playback: {recording.name}
        </Typography>
        
        <IconButton 
          edge="end" 
          color="inherit" 
          component={Link} 
          to="/"
          title="Home"
        >
          <HomeIcon />
        </IconButton>
      </Toolbar>
    </AppBar>
    
    <Container maxWidth="xl" sx={{ mt: 2, mb: 4 }}>
      <RecordingPlayback 
        recording={recording} 
        onClose={handleClose}
      />
    </Container>
  </>
);
};

export default PlaybackPage;