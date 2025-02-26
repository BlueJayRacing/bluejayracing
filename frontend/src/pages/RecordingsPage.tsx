// src/pages/RecordingsPage.tsx
import React from 'react';
import { Link } from 'react-router-dom';
import { Container, Typography, Box, Button, Paper, Alert } from '@mui/material';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import { useDataContext } from '../hooks/useDataContext';
import RecordingItem from '../components/recording_view/RecordingItem';
import RecordingTimeline from '../components/recording_view/RecordingTimeline';

const RecordingsPage: React.FC = () => {
  const { recordings, startRecording } = useDataContext();
  
  return (
    <Container maxWidth="lg">
      <Box py={4}>
        <Box display="flex" justifyContent="space-between" alignItems="center" mb={4}>
          <Box display="flex" alignItems="center">
            <Button
              component={Link}
              to="/data"
              startIcon={<ArrowBackIcon />}
              sx={{ mr: 2 }}
            >
              Back to Data
            </Button>
            <Typography variant="h4" component="h1">
              Recordings
            </Typography>
          </Box>
          
          <Button 
            variant="contained" 
            color="primary"
            onClick={() => startRecording()}
          >
            New Recording
          </Button>
        </Box>
        
        {recordings.length === 0 ? (
          <Alert severity="info" sx={{ mb: 4 }}>
            No recordings yet. Start a new recording from the Data page.
          </Alert>
        ) : (
          <>
            {/* Timeline */}
            <Box mb={6}>
              <Typography variant="h5" gutterBottom>Timeline</Typography>
              <RecordingTimeline recordings={recordings} />
            </Box>
            
            {/* Recordings list */}
            <Typography variant="h5" gutterBottom>All Recordings</Typography>
            <Box className="grid gap-4 md:grid-cols-2">
              {recordings.map(recording => (
                <RecordingItem 
                  key={recording.id} 
                  recording={recording}
                />
              ))}
            </Box>
          </>
        )}
      </Box>
    </Container>
  );
};

export default RecordingsPage;