// src/components/data_view/DeviceSelector.tsx

import React from 'react';
import { 
  Select, 
  MenuItem, 
  FormControl, 
  Box, 
  Typography, 
  SelectChangeEvent
} from '@mui/material';
import { useDataContext } from '../../hooks/useDataContext';
import { getDeviceAvailabilityColor, findDeviceById } from '../../config/deviceConfig';
import '../../styles/components.css';

interface DeviceSelectorProps {
  selectedDeviceId: string;
  onDeviceChange: (deviceId: string) => void;
}

const DeviceSelector: React.FC<DeviceSelectorProps> = ({
  selectedDeviceId,
  onDeviceChange
}) => {
  const { devices } = useDataContext();

  // Deduplicate devices list based on device.id
  const uniqueDevices = Array.from(
    new Map(devices.map(device => [device.id, device])).values()
  );

  // Handler for device selection change
  const handleDeviceChange = (event: SelectChangeEvent<string>) => {
    onDeviceChange(event.target.value);
  };

  // Determine device status using the config; mark as expected if found in DEFAULT_DEVICES.
  const getDeviceStatus = (deviceId: string) => {
    const device = uniqueDevices.find(d => d.id === deviceId);
    if (!device) return { isAvailable: false, isExpected: false };

    // Use the config helper to see if the device is expected.
    const deviceConfig = findDeviceById(deviceId);
    const isExpected = deviceConfig !== null;
    
    return {
      isAvailable: device.available,
      isExpected
    };
  };

  // Render a status chip based on availability and expected status.
  const renderDeviceStatusChip = (deviceId: string) => {
    const { isAvailable, isExpected } = getDeviceStatus(deviceId);
    const statusColor = getDeviceAvailabilityColor(isAvailable, isExpected);

    let label = "Unknown";
    let chipClass = "status-chip status-chip-unknown";

    if (isAvailable && isExpected) {
      label = "Available";
      chipClass = "status-chip status-chip-available";
    } else if (!isAvailable && isExpected) {
      label = "Unavailable";
      chipClass = "status-chip status-chip-unavailable";
    } else if (isAvailable && !isExpected) {
      label = "New";
      chipClass = "status-chip status-chip-available";
    }

    return (
      <span 
        className={chipClass + (isAvailable ? " status-active" : "")}
        style={{ backgroundColor: statusColor }}
      >
        {label}
      </span>
    );
  };

  return (
    <Box className="device-selector">
      <Typography variant="subtitle2" gutterBottom fontWeight={600} color="#334155">
        Device
      </Typography>

      <FormControl fullWidth variant="outlined" size="small">
        <Select
          value={selectedDeviceId || ''}
          onChange={handleDeviceChange}
          displayEmpty
          className="bg-white"
          sx={{
            '& .MuiOutlinedInput-notchedOutline': {
              borderColor: '#e2e8f0',
            },
            '&:hover .MuiOutlinedInput-notchedOutline': {
              borderColor: '#cbd5e0',
            },
            '&.Mui-focused .MuiOutlinedInput-notchedOutline': {
              borderColor: '#3b82f6',
            }
          }}
          renderValue={(selected) => {
            if (!selected) {
              return <Typography color="text.secondary">Select Device</Typography>;
            }

            const device = uniqueDevices.find(d => d.id === selected);
            return (
              <Box display="flex" alignItems="center" justifyContent="space-between">
                <Typography fontWeight={500}>{device?.name || selected}</Typography>
                {renderDeviceStatusChip(selected as string)}
              </Box>
            );
          }}
        >
          {uniqueDevices.length === 0 ? (
            <MenuItem disabled>
              <Typography color="text.secondary">No devices found</Typography>
            </MenuItem>
          ) : (
            uniqueDevices.map((device) => (
              <MenuItem key={device.id} value={device.id}>
                <Box display="flex" alignItems="center" justifyContent="space-between" width="100%">
                  <Typography>{device.name}</Typography>
                  {renderDeviceStatusChip(device.id)}
                </Box>
              </MenuItem>
            ))
          )}
        </Select>
      </FormControl>
    </Box>
  );
};

export default DeviceSelector;
