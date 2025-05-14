// src/components/data_view/DataBufferManager.tsx

import React, { useState, useEffect } from 'react';
import {
  Box,
  Button,
  Typography,
  Chip,
  Tooltip,
  IconButton,
  Paper,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  SelectChangeEvent
} from '@mui/material';
import RefreshIcon from '@mui/icons-material/Refresh';
import { useDataContext } from '../../hooks/useDataContext';
import { API_CONFIG } from '../../config/deviceConfig';
import TimestampModeToggle from './TimestampModeToggle';
import { useTimestamp } from '../../contexts/TimestampContext';

const DataBufferManager: React.FC = () => {
  
  const { devices, channels } = useDataContext();
  const [lastUpdated, setLastUpdated] = useState<Date>(new Date());
  const [totalSamples, setTotalSamples] = useState<number>(0);
  const [windowDuration, setWindowDuration] = useState<number>(60000); // 60 seconds default (increased from 30s)
  
  // Count total samples across all channels
  useEffect(() => {
    let count = 0;
    channels.forEach(channel => {
      count += channel.samples.length;
    });
    setTotalSamples(count);
    setLastUpdated(new Date());
  }, [channels]);
  
  // Handle window duration change
  const handleWindowDurationChange = (event: SelectChangeEvent<number>) => {
    const newDuration = event.target.value as number;
    console.log(`Window duration changed to ${newDuration}ms`);
    setWindowDuration(newDuration);
    
    // Dispatch custom event to notify other components
    const durationChangeEvent = new CustomEvent('windowDurationChange', {
      detail: { duration: newDuration }
    });
    window.dispatchEvent(durationChangeEvent);
  };
  
  // Count available devices
  const availableDeviceCount = devices.filter(d => d.available).length;
  const totalDeviceCount = devices.length;
  
  // Format time ago
  const formatTimeAgo = (date: Date): string => {
    const seconds = Math.floor((new Date().getTime() - date.getTime()) / 1000);
    
    if (seconds < 5) return 'just now';
    if (seconds < 60) return `${seconds} seconds ago`;
    
    const minutes = Math.floor(seconds / 60);
    if (minutes === 1) return '1 minute ago';
    if (minutes < 60) return `${minutes} minutes ago`;
    
    const hours = Math.floor(minutes / 60);
    if (hours === 1) return '1 hour ago';
    return `${hours} hours ago`;
  };

  return (
    <Paper className="p-3 bg-white">
      <Box display="flex" justifyContent="space-between" alignItems="center">
        <Box>
          <Typography variant="subtitle2" gutterBottom>
            Data Buffer Status
          </Typography>
          
          <Box display="flex" flexWrap="wrap" gap={1} alignItems="center">
            <Tooltip title="Number of samples in memory">
              <Chip 
                label={`${totalSamples.toLocaleString()} samples`} 
                color="primary" 
                size="small"
              />
            </Tooltip>
            
            <Tooltip title="Available devices">
              <Chip 
                label={`${availableDeviceCount}/${totalDeviceCount} devices`} 
                color={availableDeviceCount > 0 ? "success" : "error"} 
                size="small"
              />
            </Tooltip>
            
            <Tooltip title={`Polling every ${API_CONFIG.pollingIntervals.data}ms`}>
              <Chip 
                label={`${API_CONFIG.pollingIntervals.data}ms polling`} 
                color="default" 
                size="small"
              />
            </Tooltip>
            
            <Typography variant="caption" color="text.secondary">
              Last updated: {formatTimeAgo(lastUpdated)}
            </Typography>
            
            <Tooltip title="Refresh status">
              <IconButton 
                size="small" 
                color="primary"
                onClick={() => setLastUpdated(new Date())}
              >
                <RefreshIcon fontSize="small" />
              </IconButton>
            </Tooltip>
          </Box>
        </Box>
        
        <Box>
          <FormControl size="small" variant="outlined" style={{ minWidth: 120 }}>
            <InputLabel>Window</InputLabel>
            <Select
              value={windowDuration}
              onChange={handleWindowDurationChange}
              label="Window"
            >
              <MenuItem value={5000}>5 seconds</MenuItem>
              <MenuItem value={10000}>10 seconds</MenuItem>
              <MenuItem value={30000}>30 seconds</MenuItem>
              <MenuItem value={60000}>1 minute</MenuItem>
              <MenuItem value={120000}>2 minutes</MenuItem>
              <MenuItem value={300000}>5 minutes</MenuItem>
            </Select>
          </FormControl>

          <TimestampModeToggle />
        </Box>
      </Box>
    </Paper>
  );
};

export default DataBufferManager;