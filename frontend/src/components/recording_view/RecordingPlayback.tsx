// src/components/recording_view/RecordingPlayback.tsx
import React, { useState, useEffect, useRef } from 'react';
import { Paper, Slider, IconButton, Typography, Box, Card, CardContent, Grid, Divider, Button } from '@mui/material';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import PauseIcon from '@mui/icons-material/Pause';
import RestartAltIcon from '@mui/icons-material/RestartAlt';
import FastForwardIcon from '@mui/icons-material/FastForward';
import FastRewindIcon from '@mui/icons-material/FastRewind';
import SkipNextIcon from '@mui/icons-material/SkipNext';
import SkipPreviousIcon from '@mui/icons-material/SkipPrevious';
import MultiViewGraph from '../data_view/MultiViewGraph';
import ChannelSelector from '../data_view/ChannelSelector';
import NumericDisplay from '../data_view/NumericDisplay';
import { Recording, Channel } from '../shared/types';
import { DATA_CONFIG, debugLog } from '../../config/dataConfig';

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
  
  const requestRef = useRef<number>();
  const previousTimeRef = useRef<number>();
  
  // Calculate total duration
  const totalDuration = recording.stats.duration || 0;
  
  // Initialize selected channels when recording changes
  useEffect(() => {
    // Get all channel names from the recording
    const channelNames = Object.keys(recording.channelData);
    
    console.log("Available recording channels:", channelNames);
    console.log("Recording duration:", totalDuration);
    console.log("Sample counts:", channelNames.map(name => 
      `${name}: ${recording.channelData[name]?.length || 0}`
    ));
    
    // Select a few channels by default, or all if there are only a few
    setSelectedChannels(
      channelNames.length <= 4 
        ? channelNames 
        : channelNames.slice(0, Math.min(3, channelNames.length))
    );
    setCurrentTime(0);
    setIsPlaying(false);
    
    // Filter data for initial view
    updateVisibleData(0);
    
  }, [recording]);
  
  // Animation loop for playback
  const animate = (time: number) => {
    if (previousTimeRef.current === undefined) {
      previousTimeRef.current = time;
    }
    
    const deltaTime = time - previousTimeRef.current;
    previousTimeRef.current = time;
    
    // Only update time if playing
    if (isPlaying) {
      // Calculate new time based on playback rate and delta
      let newTime = currentTime + (deltaTime * playbackRate * 100); // Convert ms to s then apply rate
      
      // Cap at total duration
      if (newTime > totalDuration) {
        newTime = totalDuration;
        setIsPlaying(false);
      }
      
      // Update time and visible data
      setCurrentTime(newTime);
      updateVisibleData(newTime);
    }
    
    // Continue animation loop
    requestRef.current = requestAnimationFrame(animate);
  };
  
  // Update visible data based on current playback time
  const updateVisibleData = (playbackTime: number) => {
    // Generate channel data for the selected channels up to the current time
    const filteredChannels: Channel[] = selectedChannels.map(channelName => {
      const samples = recording.channelData[channelName] || [];
      const metadata = recording.channelMetadata[channelName];
      
      // Filter samples up to current time
      const filteredSamples = samples.filter(sample => {
        const relativeSampleTime = sample.timestamp - recording.startTime;
        return relativeSampleTime <= playbackTime;
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
    
    setVisibleData(filteredChannels);
  };
  
  // Start/stop animation loop based on playing state
  useEffect(() => {
    console.log("Playback state changed:", isPlaying ? "Playing" : "Paused");
    
    if (isPlaying) {
      previousTimeRef.current = undefined;
      requestRef.current = requestAnimationFrame(animate);
    }
    
    return () => {
      if (requestRef.current) {
        cancelAnimationFrame(requestRef.current);
        requestRef.current = undefined;
      }
    };
  }, [isPlaying, playbackRate, selectedChannels]);
  
  // Effect for manual time changes (e.g., from slider)
  useEffect(() => {
    // Don't update while playing - the animation loop handles it
    if (!isPlaying) {
      updateVisibleData(currentTime);
    }
  }, [currentTime, selectedChannels]);
  
  // Format time display
  const formatTime = (ms: number) => {
    const totalSeconds = Math.floor(ms / 1000);
    const minutes = Math.floor(totalSeconds / 60);
    const seconds = totalSeconds % 60;
    return `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
  };
  
  // Handle playback controls
  const handlePlayPause = () => {
    setIsPlaying(!isPlaying);
  };
  
  const handleRestart = () => {
    setCurrentTime(0);
    setIsPlaying(false);
    updateVisibleData(0);
  };
  
  const handleSkipForward = () => {
    const newTime = Math.min(currentTime + 5000, totalDuration); // Skip 5 seconds
    setCurrentTime(newTime);
    if (!isPlaying) {
      updateVisibleData(newTime);
    }
  };
  
  const handleSkipBackward = () => {
    const newTime = Math.max(currentTime - 5000, 0); // Back 5 seconds
    setCurrentTime(newTime);
    if (!isPlaying) {
      updateVisibleData(newTime);
    }
  };
  
  const handleSpeedChange = (newSpeed: number) => {
    setPlaybackRate(newSpeed);
  };
  
  const handleSliderChange = (event: Event, value: number | number[]) => {
    const newTime = value as number;
    setCurrentTime(newTime);
    // Don't call updateVisibleData here - it will be called by the effect
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
                      Select channels to display
                    </Typography>
                  </Box>
                )}
              </Box>
              
              {/* Playback controls */}
              <Box>
                <Box display="flex" alignItems="center" justifyContent="space-between" mb={1}>
                  <Box display="flex" alignItems="center">
                    <IconButton onClick={handleRestart} size="small">
                      <RestartAltIcon />
                    </IconButton>
                    <IconButton onClick={handleSkipBackward} size="small">
                      <SkipPreviousIcon />
                    </IconButton>
                    <IconButton 
                      onClick={handlePlayPause} 
                      color="primary"
                      size="large"
                    >
                      {isPlaying ? <PauseIcon /> : <PlayArrowIcon />}
                    </IconButton>
                    <IconButton onClick={handleSkipForward} size="small">
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
                      onClick={() => handleSpeedChange(0.5)}
                      sx={{ minWidth: '40px', mx: 0.5 }}
                    >
                      0.5x
                    </Button>
                    <Button 
                      size="small"
                      variant={playbackRate === 1 ? "contained" : "outlined"}
                      onClick={() => handleSpeedChange(1)}
                      sx={{ minWidth: '40px', mx: 0.5 }}
                    >
                      1x
                    </Button>
                    <Button 
                      size="small"
                      variant={playbackRate === 2 ? "contained" : "outlined"}
                      onClick={() => handleSpeedChange(2)}
                      sx={{ minWidth: '40px', mx: 0.5 }}
                    >
                      2x
                    </Button>
                  </Box>
                </Box>
                
                <Slider
                  value={currentTime}
                  min={0}
                  max={totalDuration}
                  onChange={handleSliderChange}
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
    </Paper>
  );
};

export default RecordingPlayback;