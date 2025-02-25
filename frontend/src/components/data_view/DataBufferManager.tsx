// src/components/data_view/DataBufferManager.tsx
import React, { useEffect, useState } from 'react';
import { 
  Box, 
  Card, 
  CardContent, 
  Typography, 
  LinearProgress,
  Chip,
  Stack,
  Alert
} from '@mui/material';
import { useDataContext } from '../../hooks/useDataContext';

interface DataBufferManagerProps {
  bufferSize?: number;
}

const DataBufferManager: React.FC<DataBufferManagerProps> = ({ 
  bufferSize = 20 // Default 20 seconds
}) => {
  const { channels, maxDataRate, isLoading, useMockData } = useDataContext();
  const [bufferUsage, setBufferUsage] = useState(0);
  const [dataPointCount, setDataPointCount] = useState(0);
  const [updateRate, setUpdateRate] = useState(0);
  const [lastUpdateTime, setLastUpdateTime] = useState(Date.now());
  const [updateInterval, setUpdateInterval] = useState(200); // ms

  // Calculate buffer statistics
  useEffect(() => {
    if (!channels.length) return;

    // Count total data points across all channels
    const totalPoints = channels.reduce(
      (sum, channel) => sum + channel.samples.length, 
      0
    );
    
    setDataPointCount(totalPoints);
    
    // Calculate buffer usage (ratio of actual points to maximum possible)
    const maxPointsPossible = channels.length * bufferSize * maxDataRate;
    const usage = Math.min(100, (totalPoints / maxPointsPossible) * 100);
    setBufferUsage(usage);
    
    // Calculate update rate
    const now = Date.now();
    const timeDiff = now - lastUpdateTime;
    
    if (timeDiff > 1000) { // Only update once per second for stability
      setUpdateRate(1000 / updateInterval);
      setLastUpdateTime(now);
    }
  }, [channels, bufferSize, maxDataRate, lastUpdateTime, updateInterval]);

  return (
    <Card variant="outlined" className="mb-4">
      <CardContent>
        <Box display="flex" justifyContent="space-between" alignItems="center" mb={2}>
          <Typography variant="h6">
            Data Buffer Status
          </Typography>
          
          {useMockData && (
            <Alert severity="info" className="py-0">
              Using simulated data - API not available
            </Alert>
          )}
        </Box>
        
        <Stack direction="row" spacing={2} className="mb-3">
          <Chip 
            label={`${isLoading ? 'Loading...' : useMockData ? 'Using Mock Data' : 'Connected'}`}
            color={isLoading ? 'warning' : useMockData ? 'info' : 'success'}
          />
          <Chip 
            label={`Update Rate: ${updateRate.toFixed(1)} Hz`}
            color="primary"
            variant="outlined"
          />
          <Chip 
            label={`Data Points: ${dataPointCount}`}
            color="default"
            variant="outlined"
          />
        </Stack>
        
        <Typography variant="body2" color="text.secondary" gutterBottom>
          Buffer Usage ({bufferUsage.toFixed(1)}%)
        </Typography>
        
        <LinearProgress 
          variant="determinate" 
          value={bufferUsage}
          color={bufferUsage > 90 ? 'error' : bufferUsage > 70 ? 'warning' : 'primary'}
        />
        
        <Box sx={{ mt: 2 }}>
          <Typography variant="body2" color="text.secondary">
            Buffer Size: {bufferSize} seconds
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Max Data Rate: {maxDataRate} Hz
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Channels: {channels.length}
          </Typography>
        </Box>
      </CardContent>
    </Card>
  );
};

export default DataBufferManager;