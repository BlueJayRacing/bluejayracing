import React from 'react';
import { useNavigate } from 'react-router-dom';
import { Paper, Typography, Box, Tooltip } from '@mui/material';
import { Recording } from '../shared/types';

interface RecordingTimelineProps {
  recordings: Recording[];
  onSelectRecording?: (recording: Recording) => void;
}

const RecordingTimeline: React.FC<RecordingTimelineProps> = ({ 
  recordings, 
  onSelectRecording 
}) => {
  const navigate = useNavigate();
  
  if (!recordings.length) {
    return (
      <Paper sx={{ p: 2, textAlign: 'center' }}>
        <Typography variant="body1" color="text.secondary">
          No recordings available yet. Start a new recording from the Data page.
        </Typography>
      </Paper>
    );
  }
  
  // Sort recordings by start time and filter out any invalid recordings
  const sortedRecordings = [...recordings]
    .filter(rec => rec && rec.id && rec.startTime)
    .sort((a, b) => a.startTime - b.startTime);
  
  // Calculate timeline range with padding on both sides
  const earliestTime = Math.min(...recordings.map(r => r.startTime));
  const latestTime = Math.max(...recordings.map(r => r.endTime || Date.now()));
  
  // Add 5% padding on both sides
  const timeRange = latestTime - earliestTime;
  const padding = timeRange * 0.05;
  
  const startTime = earliestTime - padding;
  const endTime = latestTime + padding;
  const totalTimeRange = endTime - startTime;
  
  // Format date for display
  const formatDate = (timestamp: number) => {
    return new Date(timestamp).toLocaleString([], {
      month: 'short',
      day: 'numeric',
      hour: '2-digit',
      minute: '2-digit'
    });
  };
  
  // Format duration
  const formatDuration = (ms: number) => {
    const seconds = Math.floor(ms / 1000);
    if (seconds < 60) return `${seconds}s`;
    const minutes = Math.floor(seconds / 60);
    if (minutes < 60) return `${minutes}m ${seconds % 60}s`;
    const hours = Math.floor(minutes / 60);
    return `${hours}h ${minutes % 60}m`;
  };
  
  const handleRecordingClick = (recording: Recording) => {
    if (onSelectRecording) {
      onSelectRecording(recording);
    } else {
      navigate(`/playback/${recording.id}`);
    }
  };
  
  return (
    <Paper sx={{ p: 3 }}>
      <Box sx={{ 
        height: '8rem', 
        border: '1px solid', 
        borderColor: 'divider',
        borderRadius: 1,
        position: 'relative', 
        bgcolor: 'grey.50',
        mx: 3,
        px: 2,
        mb: 3 // Increased bottom margin for time labels
      }}>
        {/* Recording blocks */}
        {sortedRecordings.map((recording, index) => {
          const startOffset = ((recording.startTime - startTime) / totalTimeRange) * 100;
          const duration = recording.endTime ? 
            recording.endTime - recording.startTime : 
            Date.now() - recording.startTime;
          const width = (duration / totalTimeRange) * 100;
          
          // Calculate vertical position to avoid overlaps
          const row = index % 5;
          const topPosition = row * 16 + 15;
          
          return (
            <Tooltip
              key={recording.id}
              title={
                <>
                  <Typography variant="subtitle2">{recording.name}</Typography>
                  <Typography variant="body2">Duration: {formatDuration(duration)}</Typography>
                  <Typography variant="body2">Channels: {recording.stats.channelCount}</Typography>
                  <Typography variant="body2">Samples: {recording.stats.sampleCount.toLocaleString()}</Typography>
                  <Typography variant="caption">Click to view</Typography>
                </>
              }
              arrow
              placement="top"
            >
              <Box
                onClick={() => handleRecordingClick(recording)}
                sx={{
                  position: 'absolute',
                  height: '1.75rem',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  overflow: 'hidden',
                  left: `${Math.max(startOffset, 0)}%`,
                  width: `${Math.max(Math.min(width, 100), 2)}%`,
                  top: `${topPosition}%`,
                  bgcolor: recording.endTime ? 'primary.main' : 'error.main',
                  opacity: 0.8,
                  border: '1px solid rgba(0,0,0,0.1)',
                  zIndex: 10 - row,
                  '&:hover': {
                    opacity: 1,
                    zIndex: 20,
                    boxShadow: '0 2px 5px rgba(0,0,0,0.2)'
                  }
                }}
              >
                {width > 10 && (
                  <Typography 
                    variant="caption" 
                    noWrap
                    sx={{ 
                      color: 'white', 
                      px: 1,
                      textShadow: '0 0 2px rgba(0,0,0,0.5)',
                      width: '100%',
                      overflow: 'hidden',
                      textOverflow: 'ellipsis'
                    }}
                  >
                    {recording.name}
                  </Typography>
                )}
              </Box>
            </Tooltip>
          );
        })}
        
        {/* Time grid lines */}
        {Array.from({ length: 5 }).map((_, index) => {
          const position = `${(index + 1) * 20}%`;
          return (
            <Box 
              key={`gridline-${index}`}
              sx={{ 
                position: 'absolute', 
                height: '100%', 
                borderLeft: '1px solid', 
                borderColor: 'grey.200',
                left: position 
              }} 
            />
          );
        })}
      </Box>
      
      {/* Time labels - moved to bottom with better spacing */}
      <Box sx={{ 
        display: 'flex', 
        justifyContent: 'space-between',
        mx: 3,
        px: 0,
        position: 'relative', 
        height: '2rem',
        mt: -1
      }}>
        {/* We'll use specific positions for each label with proper alignment */}
        <Box sx={{ 
          position: 'absolute', 
          left: '0%',
          fontSize: '0.75rem',
          color: 'text.secondary'
        }}>
          {formatDate(startTime)}
        </Box>
        
        <Box sx={{ 
          position: 'absolute', 
          left: '20%', 
          transform: 'translateX(-50%)',
          fontSize: '0.75rem',
          color: 'text.secondary'
        }}>
          {formatDate(startTime + totalTimeRange * 0.2)}
        </Box>
        
        <Box sx={{ 
          position: 'absolute', 
          left: '40%', 
          transform: 'translateX(-50%)',
          fontSize: '0.75rem',
          color: 'text.secondary'
        }}>
          {formatDate(startTime + totalTimeRange * 0.4)}
        </Box>
        
        <Box sx={{ 
          position: 'absolute', 
          left: '60%', 
          transform: 'translateX(-50%)',
          fontSize: '0.75rem',
          color: 'text.secondary'
        }}>
          {formatDate(startTime + totalTimeRange * 0.6)}
        </Box>
        
        <Box sx={{ 
          position: 'absolute', 
          left: '80%', 
          transform: 'translateX(-50%)',
          fontSize: '0.75rem',
          color: 'text.secondary'
        }}>
          {formatDate(startTime + totalTimeRange * 0.8)}
        </Box>
        
        {/* Special handling for the last label to prevent overflow */}
        <Box sx={{ 
          position: 'absolute', 
          right: '0%',
          fontSize: '0.75rem',
          color: 'text.secondary',
          textAlign: 'right'
        }}>
          {formatDate(endTime)}
        </Box>
      </Box>
    </Paper>
  );
};

export default RecordingTimeline;