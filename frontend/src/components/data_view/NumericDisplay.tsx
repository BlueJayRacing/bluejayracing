// src/components/data_view/NumericDisplay.tsx

import React from 'react';
import { Box, Paper, Typography } from '@mui/material';
import { Channel } from '../shared/DataContext';
import { getChannelCategoryColor } from '../../config/deviceConfig';

interface NumericDisplayProps {
  channels: Channel[];
}

const NumericDisplay: React.FC<NumericDisplayProps> = ({ channels }) => {
  if (channels.length === 0) {
    return (
      <Box textAlign="center" py={2}>
        <Typography variant="body2" color="text.secondary">
          No channels selected
        </Typography>
      </Box>
    );
  }

  // Group channels by device
  const channelsByDevice = channels.reduce((groups, channel) => {
    const deviceId = channel.device_id || 'unknown';
    if (!groups[deviceId]) {
      groups[deviceId] = [];
    }
    groups[deviceId].push(channel);
    return groups;
  }, {} as Record<string, Channel[]>);

  // Helper to get the latest value for a channel
  const getLatestValue = (channel: Channel) => {
    if (!channel.samples || channel.samples.length === 0) {
      return 'N/A';
    }
    
    // Get the last sample
    const lastSample = channel.samples[channel.samples.length - 1];
    
    // Format the value based on its magnitude
    const value = lastSample.value;
    
    if (Math.abs(value) < 0.01) {
      return value.toFixed(5);
    } else if (Math.abs(value) < 10) {
      return value.toFixed(3);
    } else if (Math.abs(value) < 100) {
      return value.toFixed(2);
    } else if (Math.abs(value) < 1000) {
      return value.toFixed(1);
    } else {
      return value.toFixed(0);
    }
  };
  
  // Helper to get the channel name without device prefix
  const getChannelShortName = (channel: Channel) => {
    const parts = channel.name.split('/');
    return parts.length > 1 ? parts[1] : channel.name;
  };
  
  // Helper to get the channel category
  const getChannelCategory = (channel: Channel) => {
    // Extract just the channel name from the full name (deviceId/channelName)
    const channelName = getChannelShortName(channel);
    
    if (channelName.includes("linpot_")) return "Potentiometers";
    if (channelName.includes("wheel_speed_")) return "Wheel Speeds";
    if (channelName.includes("brake_pressure_")) return "Brake Pressure";
    if (channelName.includes("steering_")) return "Steering";
    if (channelName.includes("axle_")) return "Axle";
    if (channelName.includes("temperature_")) return "Temperature";
    if (channelName.includes("pressure_")) return "Pressure";
    if (channelName.includes("imu_")) return "IMU";
    if (channelName.includes("gps_")) return "GPS";
    if (channelName.includes("Channel_")) return "WFT";
    
    return "Other";
  };

  return (
    <Box>
      {Object.entries(channelsByDevice).map(([deviceId, deviceChannels]) => (
        <Box key={deviceId} mb={2}>          
          <Box className="grid grid-cols-1 gap-2">
            {deviceChannels.map((channel) => {
              const categoryColor = getChannelCategoryColor(getChannelCategory(channel));
              const shortName = getChannelShortName(channel);
              
              return (
                <Paper
                  key={channel.name}
                  elevation={0}
                  className="p-2 border"
                  sx={{
                    borderLeft: `4px solid ${categoryColor}`,
                    backgroundColor: 'rgba(0, 0, 0, 0.02)'
                  }}
                >
                  <Box display="flex" justifyContent="space-between" alignItems="center">
                    <Typography variant="body2" className="font-medium">
                      {shortName}
                    </Typography>
                    <Typography 
                      variant="body2" 
                      className="font-mono font-bold"
                      sx={{ 
                        minWidth: '80px', 
                        textAlign: 'right' 
                      }}
                    >
                      {getLatestValue(channel)}
                    </Typography>
                  </Box>
                </Paper>
              );
            })}
          </Box>
        </Box>
      ))}
    </Box>
  );
};

export default NumericDisplay;