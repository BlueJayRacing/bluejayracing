// src/components/data_view/ChannelSelector.tsx
import React from 'react';
import { FormGroup, FormControlLabel, Checkbox, Button, Typography } from '@mui/material';

interface ChannelSelectorProps {
  availableChannels: string[];
  selectedChannels: string[];
  onSelectionChange: (selected: string[]) => void;
}

const ChannelSelector: React.FC<ChannelSelectorProps> = ({
  availableChannels,
  selectedChannels,
  onSelectionChange
}) => {
  const handleToggleChannel = (channel: string) => {
    const isSelected = selectedChannels.includes(channel);
    if (isSelected) {
      onSelectionChange(selectedChannels.filter(c => c !== channel));
    } else {
      onSelectionChange([...selectedChannels, channel]);
    }
  };

  const handleSelectAll = () => {
    onSelectionChange([...availableChannels]);
  };

  const handleClearAll = () => {
    onSelectionChange([]);
  };

  // Group channels by type for better organization
  const channelGroups = availableChannels.reduce((groups, channel) => {
    let groupName = "Other";
    
    if (channel.includes("linpot_")) {
      groupName = "Potentiometers";
    } else if (channel.includes("wheel_speed_")) {
      groupName = "Wheel Speeds";
    } else if (channel.includes("brake_pressure_")) {
      groupName = "Brake Pressure";
    } else if (channel.includes("steering_")) {
      groupName = "Steering";
    } else if (channel.includes("axle_")) {
      groupName = "Axle";
    }
    
    if (!groups[groupName]) {
      groups[groupName] = [];
    }
    
    groups[groupName].push(channel);
    return groups;
  }, {} as Record<string, string[]>);

  return (
    <div className="p-3 border rounded-lg bg-gray-50">
      <div className="flex justify-between items-center mb-2">
        <h3 className="text-sm font-semibold">Channels</h3>
        <div className="space-x-2">
          <Button 
            size="small" 
            variant="outlined" 
            color="primary"
            onClick={handleSelectAll}
          >
            Select All
          </Button>
          <Button 
            size="small" 
            variant="outlined" 
            color="secondary"
            onClick={handleClearAll}
          >
            Clear All
          </Button>
        </div>
      </div>
      
      <div className="max-h-40 overflow-y-auto pr-1">
        {Object.entries(channelGroups).map(([groupName, channels]) => (
          <div key={groupName} className="mb-2">
            <Typography variant="caption" className="font-semibold block mb-1 text-gray-600">
              {groupName}
            </Typography>
            <FormGroup className="ml-2">
              {channels.map(channel => (
                <FormControlLabel
                  key={channel}
                  control={
                    <Checkbox
                      checked={selectedChannels.includes(channel)}
                      onChange={() => handleToggleChannel(channel)}
                      size="small"
                    />
                  }
                  label={<span className="text-sm">{channel}</span>}
                />
              ))}
            </FormGroup>
          </div>
        ))}
      </div>
    </div>
  );
};

export default ChannelSelector;