// src/components/recording_view/RecordingPlayback.tsx
import React, { useState, useEffect, useRef, useMemo } from 'react';
import { Grid, Card, CardContent, Typography, Box } from '@mui/material';
import { Recording, Channel } from '../shared/types';
import PlaybackGraph from '../playback_view/PlaybackGraph';
import PlaybackControls from '../playback_view/PlaybackControls';
import ChannelSelector from '../data_view/ChannelSelector';

interface RecordingPlaybackProps {
  recording: Recording;
  onClose?: () => void;
}

const RecordingPlayback: React.FC<RecordingPlaybackProps> = ({ recording, onClose }) => {
  // Prevent excessive re-renders
  const renderCountRef = useRef(0);
  
  // Playback state
  const [isPlaying, setIsPlaying] = useState(false);
  const [currentTime, setCurrentTime] = useState(0);
  const [playbackRate, setPlaybackRate] = useState(1);
  const [selectedChannels, setSelectedChannels] = useState<string[]>([]);
  const [visibleData, setVisibleData] = useState<Channel[]>([]);
  
  // Animation frame reference
  const animationFrameRef = useRef<number | null>(null);
  const lastFrameTimeRef = useRef<number | null>(null);
  const hasInitializedRef = useRef(false);
  
  // console.log(`RecordingPlayback render with recording: ${recording.id}, count: ${++renderCountRef.current}`);
  
  // Initialize with default channels or restore from localStorage
  useEffect(() => {
    if (hasInitializedRef.current) {
      return; // Skip re-initialization
    }
    
    hasInitializedRef.current = true;
    
    // Get available channels
    const availableChannels = Object.keys(recording.channelData || {});
    
    if (availableChannels.length > 0) {
      // Select first few channels by default (max 3)
      const initialSelected = availableChannels.slice(0, Math.min(3, availableChannels.length));
      setSelectedChannels(initialSelected);
      console.log(`Initialized ${initialSelected.length} channels from ${availableChannels.length} available`);
    }
    
    // Reset playback state
    setCurrentTime(0);
    setIsPlaying(false);
    updateVisibleData(0); // Initial data update
  }, [recording.id]); // Only reinitialize on recording change
  
  // Total duration of recording
  const totalDuration = useMemo(() => (
    recording.stats?.duration || 
    (recording.endTime ? recording.endTime - recording.startTime : 0)
  ), [recording]);
  
  // Filter data for current playback time - Memoized for performance
  const updateVisibleData = (playbackTime: number) => {
    if (!recording.channelData) return;
    
    const channelData: Channel[] = selectedChannels.map(channelName => {
      const samples = recording.channelData[channelName] || [];
      const metadata = recording.channelMetadata?.[channelName];
      
      // Filter samples up to current time
      const filteredSamples = samples.filter(sample => {
        const relativeTime = sample.timestamp - recording.startTime;
        return relativeTime <= playbackTime;
      });
      
      return {
        name: channelName,
        type: metadata?.type || 0,
        min_value: metadata?.min_value ?? 0,
        max_value: metadata?.max_value ?? 100,
        samples: filteredSamples,
        metadata
      };
    });
    
    setVisibleData(channelData);
  };
  
  // Animation frame for playback - using callback to prevent re-creating on every render
  const animatePlayback = React.useCallback((timestamp: number) => {
    if (!lastFrameTimeRef.current) {
      lastFrameTimeRef.current = timestamp;
      animationFrameRef.current = requestAnimationFrame(animatePlayback);
      return;
    }
    
    // Calculate time delta in milliseconds
    const deltaTime = timestamp - lastFrameTimeRef.current;
    lastFrameTimeRef.current = timestamp;
    
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
    animationFrameRef.current = requestAnimationFrame(animatePlayback);
  }, [isPlaying, currentTime, totalDuration, playbackRate]);
  
  // Start/stop animation based on playing state
  useEffect(() => {
    if (isPlaying && !animationFrameRef.current) {
      // console.log('Starting playback animation');
      lastFrameTimeRef.current = null;
      animationFrameRef.current = requestAnimationFrame(animatePlayback);
    }
    
    // Clean up animation on unmount or when state changes
    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
        animationFrameRef.current = null;
      }
    };
  }, [isPlaying, animatePlayback]);
  
  // Update data when channels change or time is manually changed
  useEffect(() => {
    if (!isPlaying) {
      updateVisibleData(currentTime);
    }
  }, [selectedChannels, currentTime, isPlaying]);
  
  // Playback controls handlers
  const handlePlayPause = () => {
    setIsPlaying(!isPlaying);
  };
  
  const handleRestart = () => {
    setCurrentTime(0);
    setIsPlaying(false);
    updateVisibleData(0);
  };
  
  const handleSeek = (time: number) => {
    setCurrentTime(time);
    if (!isPlaying) {
      updateVisibleData(time);
    }
  };
  
  const handleSkipForward = (seconds = 5) => {
    const newTime = Math.min(currentTime + (seconds * 1000), totalDuration);
    setCurrentTime(newTime);
    updateVisibleData(newTime);
  };
  
  const handleSkipBackward = (seconds = 5) => {
    const newTime = Math.max(currentTime - (seconds * 1000), 0);
    setCurrentTime(newTime);
    updateVisibleData(newTime);
  };
  
  const handleRateChange = (rate: number) => {
    setPlaybackRate(rate);
  };
  
  return (
    <Box mb={8}>
      <Grid container spacing={3}>
        {/* Channel selector */}
        <Grid item xs={12} md={3}>
          <Card variant="outlined">
            <CardContent>
              <Typography variant="subtitle1" gutterBottom>Channels</Typography>
              <ChannelSelector
                availableChannels={Object.keys(recording.channelData || {})}
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
                <PlaybackGraph
                  channelNames={selectedChannels}
                  visibleData={visibleData}
                  currentTime={currentTime}
                  totalDuration={totalDuration}
                  recording={recording}
                  height={300}
                />
              </Box>
            </CardContent>
          </Card>
          
          {/* Playback controls */}
          <PlaybackControls
            isPlaying={isPlaying}
            currentTime={currentTime}
            totalDuration={totalDuration}
            playbackRate={playbackRate}
            onPlayPause={handlePlayPause}
            onSeek={handleSeek}
            onRestart={handleRestart}
            onSkipForward={handleSkipForward}
            onSkipBackward={handleSkipBackward}
            onRateChange={handleRateChange}
          />
        </Grid>
      </Grid>
    </Box>
  );
};

export default React.memo(RecordingPlayback);