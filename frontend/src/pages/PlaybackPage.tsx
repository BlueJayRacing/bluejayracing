// src/pages/PlaybackPage.tsx
import React, { useEffect, useState } from 'react';
import { useParams, useNavigate, Link } from 'react-router-dom';
import { Container, Typography, Box, Button, Paper, Alert, CircularProgress } from '@mui/material';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import { useDataContext } from '../hooks/useDataContext';
import RecordingPlayback from '../components/recording_view/RecordingPlayback';
import { DataProvider } from '../components/shared/DataContext';
import { Recording } from '../components/shared/types';

// This wrapper component uses the recording ID from URL params
const PlaybackPageWrapper: React.FC = () => {
  const { recordingId } = useParams<{ recordingId: string }>();
  
  return (
    <DataProvider initialPlaybackRecordingId={recordingId || null}>
      <PlaybackPageContent />
    </DataProvider>
  );
};

// Main content component that has access to the DataContext
const PlaybackPageContent: React.FC = () => {
  const { recordingId } = useParams<{ recordingId: string }>();
  const navigate = useNavigate();
  const { recordings, getRecordingById } = useDataContext();
  const [recording, setRecording] = useState<Recording | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  
  useEffect(() => {
    // Short delay to ensure recordings are loaded
    const timer = setTimeout(() => {
      if (!recordingId) {
        setError('No recording ID provided');
        setLoading(false);
        return;
      }
      
      const foundRecording = getRecordingById(recordingId);
      if (foundRecording) {
        setRecording(foundRecording);
        setLoading(false);
      } else {
        setError(`Recording with ID ${recordingId} not found`);
        setLoading(false);
      }
    }, 100);
    
    return () => clearTimeout(timer);
  }, [recordingId, getRecordingById, recordings]);
  
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
            Playback
          </Typography>
        </Box>
        
        <RecordingPlayback 
          recording={recording} 
          onClose={handleClose}
        />
      </Box>
    </Container>
  );
};

export default PlaybackPageWrapper;