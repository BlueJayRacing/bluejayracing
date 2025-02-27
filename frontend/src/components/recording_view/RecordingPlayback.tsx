// src/components/recording_view/RecordingPlayback.tsx
import React, { useState, useEffect, useRef } from 'react';
import { Paper, Slider, IconButton, Typography, Box, Card, CardContent, Grid, Button, Alert } from '@mui/material';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import PauseIcon from '@mui/icons-material/Pause';
import RestartAltIcon from '@mui/icons-material/RestartAlt';
import SkipNextIcon from '@mui/icons-material/SkipNext';
import SkipPreviousIcon from '@mui/icons-material/SkipPrevious';
import MultiViewGraph from '../data_view/MultiViewGraph';
import ChannelSelector from '../data_view/ChannelSelector';
import NumericDisplay from '../data_view/NumericDisplay';
import { Recording, Channel, TimeValue } from '../shared/types';

interface RecordingPlaybackProps {
  recording: Recording;
  onClose?: () => void;
}

const RecordingPlayback: React.FC<RecordingPlaybackProps> = ({ recording, onClose }) => {
  const [isPlaying, setIsPlaying] = useState(false);
  const [currentTime, setCurrentTime] = useState(0);
  const [playbackRate, setPlaybackRate] = useState(1);
  const [selectedChannels, setSelectedChannels] = useState<string[]>([]);
  const [visibleData, setVisibleData] = useState<Channel[]>([]);
  const [hasData, setHasData] = useState(false);
  
  // Animation frame reference
  const animationRef = useRef<number | null>(null);
  const lastFrameTime = useRef<number | null>(null);
  
  // Total duration of recording
  const totalDuration = recording.stats.duration || 0;
  
  // Initialize selected channels when recording changes
  useEffect(() => {
    // Check if recording has actual data
    const channelNames = Object.keys(recording.channelData || {});
    const hasSamples = channelNames.some(name => {
      const samples = recording.channelData[name];
      return samples && samples.length > 0;
    });
    
    setHasData(hasSamples);
    
    if (hasSamples) {
      console.log("Recording data found:", recording.name);
      console.log("Channels:", channelNames);
      console.log("Sample counts:", channelNames.map(name => 
        `${name}: ${recording.channelData[name]?.length || 0}`
      ));
      
      // Select first few channels by default or all if there are only a few
      setSelectedChannels(
        channelNames.length <= 4 
          ? channelNames 
          : channelNames.slice(0, Math.min(3, channelNames.length))
      );
    } else {
      console.log("No data found in recording:", recording.name);
    }
    
    // Reset playback state
    setCurrentTime(0);
    setIsPlaying(false);
    updateVisibleData(0);
  }, [recording]);
  
  // Function to filter data for current playback time
  const updateVisibleData = (playbackTime: number) => {
    if (!recording.channelData) return;
    
    const channelData: Channel[] = selectedChannels.map(channelName => {
      const samples = recording.channelData[channelName] || [];
      const metadata = recording.channelMetadata[channelName];
      
      // Filter samples up to current time
      const filteredSamples = samples.filter(sample => {
        const relativeTime = sample.timestamp - recording.startTime;
        return relativeTime <= playbackTime;
      });
      
      return {
        name: channelName,
        type: metadata?.type || 0,
        min_value: metadata?.min_value || 0,
        max_value: metadata?.max_value || 100,
        samples: filteredSamples,
        metadata
      };
    });
    
    setVisibleData(channelData);
  };
  
  // Playback animation
  const animate = (timestamp: number) => {
    if (!lastFrameTime.current) {
      lastFrameTime.current = timestamp;
      animationRef.current = requestAnimationFrame(animate);
      return;
    }
    
    // Calculate time delta in milliseconds
    const deltaTime = timestamp - lastFrameTime.current;
    lastFrameTime.current = timestamp;
    
    // Only update if playing
    if (isPlaying) {
      // Calculate new time based on playback rate
      const newTime = Math.min(
        currentTime + (deltaTime * playbackRate),
        totalDuration
      );
      
      // Check if we reached the end
      if (newTime >= totalDuration) {
        setCurrentTime(totalDuration);
        setIsPlaying(false);
        updateVisibleData(totalDuration);
      } else {
        setCurrentTime(newTime);
        updateVisibleData(newTime);
      }
    }
    
    // Continue animation loop
    animationRef.current = requestAnimationFrame(animate);
  };
  
  // Start/stop animation based on playing state
  useEffect(() => {
    console.log("Playback state changed:", isPlaying ? "Playing" : "Paused", "at time:", currentTime);
    
    if (isPlaying) {
      // Start animation when playing
      lastFrameTime.current = null;
      animationRef.current = requestAnimationFrame(animate);
      console.log("Started animation frame");
    }
    
    // Clean up animation on unmount or when state changes
    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
        animationRef.current = null;
      }
    };
  }, [isPlaying]);
  
  // Update data when channels change or time is manually changed
  useEffect(() => {
    // Update filtered data for current time
    if (!isPlaying) {
      updateVisibleData(currentTime);
    }
  }, [selectedChannels, currentTime]);
  
  // Playback controls
  const handlePlayPause = () => {
    setIsPlaying(!isPlaying);
  };
  
  const handleRestart = () => {
    setCurrentTime(0);
    setIsPlaying(false);
    updateVisibleData(0);
  };
  
  const handleSeek = (event: Event, value: number | number[]) => {
    const newTime = typeof value === 'number' ? value : value[0];
    setCurrentTime(newTime);
    if (!isPlaying) {
      updateVisibleData(newTime);
    }
  };
  
  const handleSkipForward = () => {
    const newTime = Math.min(currentTime + 5000, totalDuration);
    setCurrentTime(newTime);
    updateVisibleData(newTime);
  };
  
  const handleSkipBackward = () => {
    const newTime = Math.max(currentTime - 5000, 0);
    setCurrentTime(newTime);
    updateVisibleData(newTime);
  };
  
  // Format time for display
  const formatTime = (ms: number) => {
    const seconds = Math.floor(ms / 1000);
    const minutes = Math.floor(seconds / 60);
    const hours = Math.floor(minutes / 60);
    
    const formattedHours = hours > 0 ? `${String(hours).padStart(2, '0')}:` : '';
    return `${formattedHours}${String(minutes % 60).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
  };
  
  return (
    <Paper className="p-4">
      <Box display="flex" justifyContent="space-between" alignItems="center" mb={2}>
        <Typography variant="h5">
          {recording.name}
        </Typography>
        {onClose && (
          <Button onClick={onClose}>
            Close Playback
          </Button>
        )}
      </Box>
      
      {!hasData ? (
        <Alert severity="warning" sx={{ mb: 2 }}>
          This recording does not contain any data. Try creating a new recording.
        </Alert>
      ) : (
        <Grid container spacing={3}>
          {/* Channel selector */}
          <Grid item xs={12} md={3}>
            <Card variant="outlined">
              <CardContent>
                <Typography variant="subtitle1" gutterBottom>Channels</Typography>
                <ChannelSelector
                  availableChannels={Object.keys(recording.channelData)}
                  selectedChannels={selectedChannels}
                  onSelectionChange={setSelectedChannels}
                />
              </CardContent>
            </Card>
          </Grid>
          
          {/* Graph and controls */}
          <Grid item xs={12} md={9}>
            <Card variant="outlined">
              <CardContent>
                {/* Graph */}
                <Box height={300} mb={2}>
                  {visibleData.length > 0 && selectedChannels.length > 0 ? (
                    <MultiViewGraph
                      channelNames={selectedChannels}
                      customData={visibleData}
                      height={300}
                    />
                  ) : (
                    <Box 
                      display="flex" 
                      justifyContent="center" 
                      alignItems="center" 
                      height="100%"
                      bgcolor="rgba(0,0,0,0.03)"
                      borderRadius={1}
                    >
                      <Typography color="textSecondary">
                        {selectedChannels.length === 0 
                          ? "Select channels to display" 
                          : "No data available for selected time range"}
                      </Typography>
                    </Box>
                  )}
                </Box>
                
                {/* Playback controls */}
                <Box>
                  <Box display="flex" alignItems="center" justifyContent="space-between" mb={1}>
                    <Box display="flex" alignItems="center">
                      <IconButton onClick={handleRestart} size="small" title="Restart">
                        <RestartAltIcon />
                      </IconButton>
                      <IconButton onClick={handleSkipBackward} size="small" title="Skip back 5 seconds">
                        <SkipPreviousIcon />
                      </IconButton>
                      <IconButton 
                        onClick={handlePlayPause} 
                        color="primary"
                        size="large"
                        title={isPlaying ? "Pause" : "Play"}
                      >
                        {isPlaying ? <PauseIcon /> : <PlayArrowIcon />}
                      </IconButton>
                      <IconButton onClick={handleSkipForward} size="small" title="Skip forward 5 seconds">
                        <SkipNextIcon />
                      </IconButton>
                    </Box>
                    
                    <Typography variant="body2">
                      {formatTime(currentTime)} / {formatTime(totalDuration)}
                    </Typography>
                    
                    <Box display="flex" alignItems="center">
                      <Typography variant="body2" mr={1}>Speed:</Typography>
                      <Button 
                        size="small" 
                        variant={playbackRate === 0.5 ? "contained" : "outlined"}
                        onClick={() => setPlaybackRate(0.5)}
                        sx={{ minWidth: '40px', mx: 0.5 }}
                      >
                        0.5x
                      </Button>
                      <Button 
                        size="small"
                        variant={playbackRate === 1 ? "contained" : "outlined"}
                        onClick={() => setPlaybackRate(1)}
                        sx={{ minWidth: '40px', mx: 0.5 }}
                      >
                        1x
                      </Button>
                      <Button 
                        size="small"
                        variant={playbackRate === 2 ? "contained" : "outlined"}
                        onClick={() => setPlaybackRate(2)}
                        sx={{ minWidth: '40px', mx: 0.5 }}
                      >
                        2x
                      </Button>
                    </Box>
                  </Box>
                  
                  <Slider
                    value={currentTime}
                    min={0}
                    max={totalDuration || 1}
                    onChange={handleSeek}
                    sx={{ mt: 1 }}
                  />
                </Box>
              </CardContent>
            </Card>
            
            {/* Current Values Display */}
            {visibleData.length > 0 && (
              <Card variant="outlined" sx={{ mt: 2 }}>
                <CardContent>
                  <Typography variant="subtitle1" gutterBottom>
                    Current Values
                  </Typography>
                  <NumericDisplay channels={visibleData} />
                </CardContent>
              </Card>
            )}
          </Grid>
        </Grid>
      )}
    </Paper>
  );
};

export default RecordingPlayback;