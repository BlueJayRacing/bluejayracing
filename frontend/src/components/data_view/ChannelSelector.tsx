// src/components/data_view/ChannelSelector.tsx

import React, { useState, useEffect } from 'react';
import {
  FormGroup,
  FormControlLabel,
  Checkbox,
  Button,
  Typography,
  Box,
  Chip,
  Divider,
  IconButton,
  Collapse
} from '@mui/material';
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown';
import KeyboardArrowUpIcon from '@mui/icons-material/KeyboardArrowUp';
import { useDataContext } from '../../hooks/useDataContext';
import { 
  getChannelAvailabilityColor, 
  getChannelCategoryColor,
  DEFAULT_DEVICES 
} from '../../config/deviceConfig';
import '../../styles/components.css';

interface ChannelSelectorProps {
  selectedDeviceId: string;
  selectedChannels: string[];
  onSelectionChange: (selected: string[]) => void;
}

const ChannelSelector: React.FC<ChannelSelectorProps> = ({
  selectedDeviceId,
  selectedChannels,
  onSelectionChange
}) => {
  const { devices } = useDataContext();
  const [availableChannels, setAvailableChannels] = useState<Array<{
    name: string;
    fullName: string;
    category: string;
    isAvailable: boolean;
    isExpected: boolean;
  }>>([]);
  
  // Track expanded categories
  const [expandedCategories, setExpandedCategories] = useState<Record<string, boolean>>({});
  
  // Find the selected device
  const selectedDevice = devices.find(d => d.id === selectedDeviceId);
  
  // Find if the device is expected
  const isDeviceExpected = Boolean(selectedDevice?.config);
  
  // Process available channels when device changes
  useEffect(() => {
    if (!selectedDeviceId) {
      setAvailableChannels([]);
      return;
    }
    
    // Get device configuration
    const deviceConfig = DEFAULT_DEVICES.find(d => d.id === selectedDeviceId);
    const channelsFromDevice = selectedDevice?.channels || [];
    
    // Get expected channels from device config
    const expectedChannels = deviceConfig?.expectedChannels || [];
    
    // Map of channel names from device (available channels)
    const availableChannelMap = new Map<string, boolean>();
    
    channelsFromDevice.forEach(fullName => {
      // Extract just the channel name from the full name (deviceId/channelName)
      const channelName = fullName.split('/')[1];
      availableChannelMap.set(channelName, true);
    });
    
    // Create the channel list
    const channelList: Array<{
      name: string;
      fullName: string;
      category: string;
      isAvailable: boolean;
      isExpected: boolean;
    }> = [];
    
    // Add all expected channels first
    expectedChannels.forEach(channel => {
      const isAvailable = availableChannelMap.has(channel.name);
      channelList.push({
        name: channel.name,
        fullName: `${selectedDeviceId}/${channel.name}`,
        category: channel.category || 'Other',
        isAvailable: isAvailable && selectedDevice?.available || false,
        isExpected: true
      });
      
      // Remove from map so we don't process it twice
      availableChannelMap.delete(channel.name);
    });
    
    // Add any remaining channels from the device that weren't in the expected list
    channelsFromDevice.forEach(fullName => {
      const channelName = fullName.split('/')[1];
      
      // Skip if we already processed this channel
      if (!availableChannelMap.has(channelName)) return;
      
      channelList.push({
        name: channelName,
        fullName,
        category: 'Unknown',
        isAvailable: selectedDevice?.available || false,
        isExpected: false
      });
    });
    
    // Sort by category and name
    channelList.sort((a, b) => {
      if (a.category !== b.category) return a.category.localeCompare(b.category);
      return a.name.localeCompare(b.name);
    });
    
    // Set the channel list
    setAvailableChannels(channelList);
    
    // Initialize expanded categories
    const categories = [...new Set(channelList.map(c => c.category))];
    const newExpandedCategories: Record<string, boolean> = {};
    categories.forEach(cat => {
      // Initialize all categories as expanded
      newExpandedCategories[cat] = true;
    });
    setExpandedCategories(newExpandedCategories);
  }, [selectedDeviceId, devices, selectedDevice]);
  
  // Toggle a single channel
  const handleToggleChannel = (fullName: string) => {
    if (selectedChannels.includes(fullName)) {
      onSelectionChange(selectedChannels.filter(channel => channel !== fullName));
    } else {
      onSelectionChange([...selectedChannels, fullName]);
    }
  };
  
  // Toggle category expansion
  const toggleCategoryExpansion = (category: string) => {
    setExpandedCategories(prev => ({
      ...prev,
      [category]: !prev[category]
    }));
  };
  
  // Toggle all channels in a category
  const handleToggleCategory = (category: string) => {
    const categoryChannels = availableChannels
      .filter(channel => channel.category === category)
      .map(channel => channel.fullName);
    
    // Check if all category channels are selected
    const allSelected = categoryChannels.every(channel => selectedChannels.includes(channel));
    
    if (allSelected) {
      // Remove all category channels
      onSelectionChange(selectedChannels.filter(channel => !categoryChannels.includes(channel)));
    } else {
      // Add all category channels that aren't already selected
      const newChannels = categoryChannels.filter(channel => !selectedChannels.includes(channel));
      onSelectionChange([...selectedChannels, ...newChannels]);
    }
  };
  
  // Select all channels
  const handleSelectAll = () => {
    onSelectionChange(availableChannels.map(channel => channel.fullName));
  };
  
  // Clear all channels
  const handleClearAll = () => {
    onSelectionChange([]);
  };
  
  // Group channels by category
  const channelGroups = availableChannels.reduce((groups, channel) => {
    if (!groups[channel.category]) {
      groups[channel.category] = [];
    }
    
    groups[channel.category].push(channel);
    return groups;
  }, {} as Record<string, typeof availableChannels>);
  
  // Render channel status indicator
  const renderChannelStatus = (channel: typeof availableChannels[0]) => {
    const statusColor = getChannelAvailabilityColor(
      selectedDevice?.available || false,
      isDeviceExpected,
      channel.isExpected
    );
    
    return (
      <Box 
        component="span" 
        sx={{ 
          width: 10, 
          height: 10, 
          borderRadius: '50%', 
          backgroundColor: statusColor,
          display: 'inline-block',
          ml: 1,
          opacity: channel.isAvailable ? 1 : 0.5
        }} 
      />
    );
  };
  
  // Render category header with channel count
  const renderCategoryHeader = (category: string, channels: typeof availableChannels) => {
    const categoryColor = getChannelCategoryColor(category);
    const selectedCount = channels.filter(c => selectedChannels.includes(c.fullName)).length;
    const isExpanded = expandedCategories[category] || false;
    
    return (
      <Box 
        sx={{ 
          display: 'flex', 
          alignItems: 'center', 
          justifyContent: 'space-between',
          py: 0.5,
          px: 1,
          my: 0.5,
          borderRadius: '4px',
          backgroundColor: `${categoryColor}10`,
          cursor: 'pointer',
          '&:hover': { 
            backgroundColor: `${categoryColor}20`
          }
        }}
      >
        <Box sx={{ display: 'flex', alignItems: 'center' }} onClick={() => toggleCategoryExpansion(category)}>
          <IconButton size="small" sx={{ mr: 0.5, p: 0 }}>
            {isExpanded ? <KeyboardArrowUpIcon fontSize="small" /> : <KeyboardArrowDownIcon fontSize="small" />}
          </IconButton>
          <Box 
            sx={{ 
              width: 10, 
              height: 10, 
              borderRadius: '50%', 
              bgcolor: categoryColor, 
              mr: 1 
            }}
          />
          <Typography variant="body2" sx={{ fontWeight: 600 }}>
            {category}
          </Typography>
        </Box>
        
        <Box sx={{ display: 'flex', alignItems: 'center' }}>
          <Typography variant="caption" color="text.secondary" sx={{ mr: 1 }}>
            {selectedCount}/{channels.length}
          </Typography>
          <Checkbox
            size="small"
            indeterminate={selectedCount > 0 && selectedCount < channels.length}
            checked={selectedCount === channels.length && channels.length > 0}
            onChange={() => handleToggleCategory(category)}
            sx={{ 
              p: 0.5,
              '&.Mui-checked': { color: categoryColor },
              '&.MuiCheckbox-indeterminate': { color: categoryColor }
            }}
          />
        </Box>
      </Box>
    );
  };
  
  // If no device is selected
  if (!selectedDeviceId) {
    return (
      <Box className="channel-selector">
        <Typography color="text.secondary" align="center">
          Please select a device first
        </Typography>
      </Box>
    );
  }

  return (
    <Box className="channel-selector">
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 1 }}>
        <Typography variant="subtitle2" sx={{ fontWeight: 600 }} color="#334155">
          Channels
          <Chip
            label={`${selectedChannels.length}/${availableChannels.length}`}
            size="small"
            sx={{ ml: 1, height: 20, fontSize: '0.7rem' }}
          />
        </Typography>
        <Box sx={{ display: 'flex', gap: 1 }}>
          <Button 
            size="small" 
            variant="outlined" 
            color="primary"
            onClick={handleSelectAll}
            disabled={availableChannels.length === 0}
            sx={{ 
              py: 0, 
              minWidth: 'unset',
              fontSize: '0.7rem',
              borderRadius: '4px'
            }}
          >
            All
          </Button>
          <Button 
            size="small" 
            variant="outlined" 
            color="secondary"
            onClick={handleClearAll}
            disabled={selectedChannels.length === 0}
            sx={{ 
              py: 0, 
              minWidth: 'unset',
              fontSize: '0.7rem',
              borderRadius: '4px'
            }}
          >
            Clear
          </Button>
        </Box>
      </Box>
      
      <Box className="overflow-y-auto pr-1" sx={{ maxHeight: 300 }}>
        {Object.entries(channelGroups).length === 0 ? (
          <Typography color="text.secondary" align="center">
            No channels available for this device
          </Typography>
        ) : (
          Object.entries(channelGroups).map(([category, channels], index) => (
            <Box key={category} sx={{ mb: 1 }}>
              {index > 0 && <Divider sx={{ my: 0.5 }} />}
              
              {renderCategoryHeader(category, channels)}
              
              <Collapse in={expandedCategories[category] || false}>
                <FormGroup sx={{ pl: 4, pr: 1 }}>
                  {channels.map(channel => (
                    <FormControlLabel
                      key={channel.fullName}
                      control={
                        <Checkbox
                          checked={selectedChannels.includes(channel.fullName)}
                          onChange={() => handleToggleChannel(channel.fullName)}
                          size="small"
                          disabled={!channel.isAvailable && selectedDevice?.available}
                          sx={{ 
                            p: 0.5,
                            '&.Mui-checked': { 
                              color: getChannelCategoryColor(channel.category) 
                            }
                          }}
                        />
                      }
                      label={
                        <Box sx={{ display: 'flex', alignItems: 'center' }}>
                          <Typography 
                            variant="body2" 
                            sx={{ 
                              fontSize: '0.8rem',
                              opacity: (!channel.isAvailable && selectedDevice?.available) ? 0.5 : 1
                            }}
                          >
                            {channel.name}
                          </Typography>
                          {renderChannelStatus(channel)}
                        </Box>
                      }
                      sx={{ my: 0, ml: 0 }}
                    />
                  ))}
                </FormGroup>
              </Collapse>
            </Box>
          ))
        )}
      </Box>
    </Box>
  );
};

export default ChannelSelector;