// src/components/recording_view/RecordingTimeline.tsx
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
      <Paper className="p-4 text-center">
        <Typography variant="body1" color="textSecondary">
          No recordings available yet. Start a new recording from the Data page.
        </Typography>
      </Paper>
    );
  }
  
  // Sort recordings by start time and filter out any invalid recordings
  const sortedRecordings = [...recordings]
    .filter(rec => rec && rec.id && rec.startTime) // Ensure valid recordings only
    .sort((a, b) => a.startTime - b.startTime);
  
  // Calculate timeline range
  const startTime = Math.min(...recordings.map(r => r.startTime)) - 1000; // Add small buffer
  const endTime = Math.max(...recordings.map(r => r.endTime || Date.now())) + 1000; // Add small buffer
  const timeRange = endTime - startTime;
  
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
    <Paper className="p-4">
      <div className="mb-2 flex justify-between text-xs text-gray-500">
        <div>{formatDate(startTime)}</div>
        <div>{formatDate(endTime)}</div>
      </div>
      
      <div className="h-32 border border-gray-300 rounded-md relative bg-gray-50">
        {/* Time grid lines */}
        {Array.from({ length: 5 }).map((_, index) => {
          const position = `${(index + 1) * 20}%`;
          const timestamp = startTime + (timeRange * (index + 1) / 5);
          
          return (
            <div key={`gridline-${index}`}>
              <div 
                className="absolute h-full border-l border-gray-200" 
                style={{ left: position }} 
              />
              <div 
                className="absolute top-0 text-xs text-gray-400" 
                style={{ left: position }}
              >
                {formatDate(timestamp)}
              </div>
            </div>
          );
        })}
        
        {/* Recording blocks */}
        {sortedRecordings.map((recording, index) => {
          const startOffset = ((recording.startTime - startTime) / timeRange) * 100;
          const duration = recording.endTime ? 
            recording.endTime - recording.startTime : 
            Date.now() - recording.startTime;
          const width = (duration / timeRange) * 100;
          
          // Calculate vertical position to avoid overlaps
          // Use more space (5 rows) to reduce overlap likelihood 
          const row = index % 5;
          const topPosition = row * 20 + 10; // 5 rows, each 20% of height
          
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
                className="absolute h-8 rounded-md cursor-pointer hover:opacity-90 flex items-center overflow-hidden"
                sx={{
                  left: `${Math.max(startOffset, 0)}%`,
                  width: `${Math.max(Math.min(width, 100), 2)}%`,
                  top: `${topPosition}%`,
                  backgroundColor: recording.endTime ? '#3f51b5' : '#f50057',
                  opacity: 0.8,
                  border: '1px solid rgba(0,0,0,0.1)',
                  zIndex: 10 - row, // Higher rows get lower z-index
                  '&:hover': {
                    opacity: 1,
                    zIndex: 20,
                    boxShadow: '0 2px 5px rgba(0,0,0,0.2)'
                  }
                }}
                onClick={() => handleRecordingClick(recording)}
              >
                <Typography 
                  variant="caption" 
                  noWrap 
                  sx={{ 
                    color: 'white', 
                    px: 1,
                    textShadow: '0 0 2px rgba(0,0,0,0.5)',
                    display: width > 10 ? 'block' : 'none',
                    width: '100%',
                    overflow: 'hidden',
                    textOverflow: 'ellipsis'
                  }}
                >
                  {recording.name}
                </Typography>
              </Box>
            </Tooltip>
          );
        })}
      </div>
    </Paper>
  );
};

export default RecordingTimeline;