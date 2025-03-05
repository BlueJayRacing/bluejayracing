// src/pages/PlaybackPage.tsx
import React, { useEffect, useState } from 'react';
import { useParams, useNavigate, Link } from 'react-router-dom';
import { Container, Typography, Box, Button, Paper, Alert, CircularProgress, AppBar, Toolbar, IconButton } from '@mui/material';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import HomeIcon from '@mui/icons-material/Home';
import { useDataContext } from '../hooks/useDataContext';
import RecordingPlayback from '../components/recording_view/RecordingPlayback';
import { Recording } from '../components/shared/types';
import { IndexedStorage } from '../utils/indexedDbStorage';

const PlaybackPage: React.FC = () => {
  const { recordingId } = useParams<{ recordingId: string }>();
  const navigate = useNavigate();
  const { recordings, getRecordingById } = useDataContext();
  const [recording, setRecording] = useState<Recording | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [recordingInitialized, setRecordingInitialized] = useState(false);

  useEffect(() => {
    // Prevent re-running if already initialized
    if (recordingInitialized) return;

    const loadRecording = async () => {
      if (!recordingId) {
        setError('No recording ID provided');
        setLoading(false);
        setRecordingInitialized(true);
        return;
      }

      try {
        // First try to get recording from context/memory
        let foundRecording = getRecordingById ? getRecordingById(recordingId) : 
          recordings.find(r => r.id === recordingId);
        
        // If not found in context, try IndexedDB
        if (!foundRecording) {
          try {
            const allRecordings = await IndexedStorage.getRecordings();
            foundRecording = allRecordings.find(r => r.id === recordingId);
          } catch (storedErr) {
            console.error('Error loading from IndexedDB:', storedErr);
          }
        }

        // Validate recording
        if (validateRecording(foundRecording)) {
          setRecording(foundRecording);
          setLoading(false);
          setRecordingInitialized(true);
          return;
        }

        // If we get here, no valid recording was found
        setError(`Recording with ID ${recordingId} not found or is invalid`);
        setLoading(false);
        setRecordingInitialized(true);
      } catch (err) {
        console.error('Error finding recording:', err);
        setError(`Error loading recording: ${(err as any).message}`);
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
          channelCount: channels.length,
          maxSampleRate: 2000,
          averageSampleRate: 2000
        };
      }
      
      // Ensure channelMetadata exists
      if (!recording.channelMetadata) {
        recording.channelMetadata = {};
      }
      
      return true;
    };

    // Start loading
    loadRecording();

    return () => {
      setRecordingInitialized(false);
    };
  }, [recordingId, getRecordingById, recordings, recordingInitialized]);

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

  const handleClose = () => {
    navigate('/recordings');
  };

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