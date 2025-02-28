// src/components/playback_view/PlaybackControls.tsx
import React from 'react';
import { 
  Box, 
  Slider, 
  IconButton, 
  Button, 
  Typography, 
  Paper,
  Tooltip,
  ButtonGroup
} from '@mui/material';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import PauseIcon from '@mui/icons-material/Pause';
import RestartAltIcon from '@mui/icons-material/RestartAlt';
import SkipNextIcon from '@mui/icons-material/SkipNext';
import SkipPreviousIcon from '@mui/icons-material/SkipPrevious';
import Forward10Icon from '@mui/icons-material/Forward10';
import Replay10Icon from '@mui/icons-material/Replay10';

interface PlaybackControlsProps {
  isPlaying: boolean;
  currentTime: number;
  totalDuration: number;
  playbackRate: number;
  onPlayPause: () => void;
  onSeek: (time: number) => void;
  onRestart: () => void;
  onSkipForward: (seconds?: number) => void;
  onSkipBackward: (seconds?: number) => void;
  onRateChange: (rate: number) => void;
}

const PlaybackControls: React.FC<PlaybackControlsProps> = ({
  isPlaying,
  currentTime,
  totalDuration,
  playbackRate,
  onPlayPause,
  onSeek,
  onRestart,
  onSkipForward,
  onSkipBackward,
  onRateChange
}) => {
  // Format time for display
  const formatTime = (ms: number) => {
    const seconds = Math.floor(ms / 1000);
    const minutes = Math.floor(seconds / 60);
    const hours = Math.floor(minutes / 60);
    
    const formattedHours = hours > 0 ? `${String(hours).padStart(2, '0')}:` : '';
    return `${formattedHours}${String(minutes % 60).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
  };
  
  // Handle slider change
  const handleSliderChange = (_: Event, value: number | number[]) => {
    const newTime = typeof value === 'number' ? value : value[0];
    onSeek(newTime);
  };
  
  // Available playback rates
  const playbackRates = [0.25, 0.5, 1, 1.5, 2];
  
  // Calculate progress percentage
  const progressPercentage = totalDuration > 0 ? (currentTime / totalDuration) * 100 : 0;

  return (
    <Paper 
      elevation={3} 
      sx={{
        position: 'sticky',
        bottom: 0,
        left: 0,
        right: 0,
        zIndex: 1000,
        p: 2,
        mt: 2,
        borderRadius: '8px 8px 0 0',
        background: 'rgba(255, 255, 255, 0.95)',
        backdropFilter: 'blur(10px)'
      }}
    >
      {/* Slider track with progress */}
      <Box sx={{ mb: 2 }}>
        <Slider
          value={currentTime}
          min={0}
          max={totalDuration || 100}
          onChange={handleSliderChange}
          sx={{
            '& .MuiSlider-thumb': {
              width: 12,
              height: 12,
              transition: '0.3s cubic-bezier(.47,1.64,.41,.8)',
              '&::before': {
                boxShadow: '0 2px 12px 0 rgba(0,0,0,0.4)',
              },
              '&:hover, &.Mui-focusVisible': {
                boxShadow: `0px 0px 0px 8px rgba(63, 81, 181, 0.16)`,
              },
              '&.Mui-active': {
                width: 16,
                height: 16,
              },
            },
            '& .MuiSlider-rail': {
              opacity: 0.28,
            },
          }}
        />
      </Box>
      
      {/* Main controls and time display */}
      <Box display="flex" justifyContent="space-between" alignItems="center">
        {/* Left side: Skip and play controls */}
        <Box display="flex" alignItems="center">
          <IconButton onClick={onRestart} title="Restart">
            <RestartAltIcon />
          </IconButton>
          
          <IconButton onClick={() => onSkipBackward(10)} title="Back 10 seconds">
            <Replay10Icon />
          </IconButton>
          
          <IconButton onClick={() => onSkipBackward()} title="Previous section">
            <SkipPreviousIcon />
          </IconButton>
          
          <IconButton 
            onClick={onPlayPause} 
            color="primary" 
            size="large"
            title={isPlaying ? "Pause" : "Play"}
            sx={{
              bgcolor: 'primary.main',
              color: 'white',
              '&:hover': {
                bgcolor: 'primary.dark',
              },
              mx: 1
            }}
          >
            {isPlaying ? <PauseIcon /> : <PlayArrowIcon />}
          </IconButton>
          
          <IconButton onClick={() => onSkipForward()} title="Next section">
            <SkipNextIcon />
          </IconButton>
          
          <IconButton onClick={() => onSkipForward(10)} title="Forward 10 seconds">
            <Forward10Icon />
          </IconButton>
        </Box>
        
        {/* Center: Time display */}
        <Box 
          sx={{ 
            display: 'flex', 
            alignItems: 'center',
            px: 2,
            borderRadius: 1,
            bgcolor: 'rgba(0,0,0,0.05)',
            py: 0.5
          }}
        >
          <Typography variant="body2" fontFamily="monospace" fontWeight="medium">
            {formatTime(currentTime)} / {formatTime(totalDuration)}
          </Typography>
        </Box>
        
        {/* Right side: Playback speed controls */}
        <Box display="flex" alignItems="center">
          <Typography variant="body2" mr={1}>Speed:</Typography>
          <ButtonGroup size="small" variant="outlined">
            {playbackRates.map(rate => (
              <Button 
                key={rate}
                variant={playbackRate === rate ? "contained" : "outlined"}
                onClick={() => onRateChange(rate)}
                sx={{ minWidth: '48px' }}
              >
                {rate}x
              </Button>
            ))}
          </ButtonGroup>
        </Box>
      </Box>
    </Paper>
  );
};

export default PlaybackControls;