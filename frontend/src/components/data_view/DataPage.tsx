// src/components/data_view/DataPage.tsx

import React, { useState, useRef, useEffect } from 'react';
import { useDataContext } from '../../hooks/useDataContext';
import MultiViewGraph from './MultiViewGraph';
import DeviceSelector from './DeviceSelector';
import ChannelSelector from './ChannelSelector';
import NumericDisplay from './NumericDisplay';
import DataBufferManager from './DataBufferManager';
import RecordingControls from './RecordingControls';
import { 
  Button, 
  CircularProgress, 
  Box, 
  Divider, 
  IconButton, 
  Paper, 
  Typography, 
  Switch, 
  FormControlLabel,
  Chip
} from '@mui/material';
import AddIcon from '@mui/icons-material/Add';
import CloseIcon from '@mui/icons-material/Close';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import ExpandLessIcon from '@mui/icons-material/ExpandLess';

// Interface for a graph instance
interface GraphInstance {
  id: number;
  deviceId: string;
  selectedChannels: string[];
  expanded: boolean;
}

const DataPage: React.FC = () => {
  
  const { channels, isLoading, devices } = useDataContext();
  
  // Graph instances state
  const [graphInstances, setGraphInstances] = useState<GraphInstance[]>([]);
  
  // UI state
  const [useSimpleCharts, setUseSimpleCharts] = useState(false);
  
  // Graph display duration - default to 60 seconds (increased from 30s)
  const [graphDuration, setGraphDuration] = useState(60000);
  
  // Initialize with one graph when component mounts
  useEffect(() => {
    if (graphInstances.length === 0 && devices.length > 0) {
      // Find first available device
      const defaultDevice = devices.find(d => d.available) || devices[0];
      
      console.log(`Initializing first graph with device: ${defaultDevice?.id || 'none'}`);
      
      setGraphInstances([{
        id: 1,
        deviceId: defaultDevice.id,
        selectedChannels: [],
        expanded: true
      }]);
    }
  }, [devices, graphInstances.length]);
  
  // Add a new graph
  const addGraph = () => {
    // Find first available device
    const defaultDevice = devices.find(d => d.available) || devices[0];
    
    const newId = Math.max(0, ...graphInstances.map(g => g.id)) + 1;
    
    console.log(`Adding new graph #${newId} with device: ${defaultDevice?.id || 'none'}`);
    
    setGraphInstances([
      ...graphInstances, 
      {
        id: newId, 
        deviceId: defaultDevice?.id || '',
        selectedChannels: [],
        expanded: true
      }
    ]);
  };
  
  // Update graph device
  const updateGraphDevice = (id: number, deviceId: string) => {
    console.log(`Updating graph #${id} to use device: ${deviceId}`);
    
    setGraphInstances(
      graphInstances.map(graph => 
        graph.id === id ? { ...graph, deviceId, selectedChannels: [] } : graph
      )
    );
  };
  
  // Update graph channels
  const updateGraphChannels = (id: number, selectedChannels: string[]) => {
    console.log(`Updating graph #${id} channels: ${selectedChannels.join(', ')}`);
    
    setGraphInstances(
      graphInstances.map(graph => 
        graph.id === id ? { ...graph, selectedChannels } : graph
      )
    );
  };
  
  // Toggle graph expanded state
  const toggleGraphExpanded = (id: number) => {
    setGraphInstances(
      graphInstances.map(graph => 
        graph.id === id ? { ...graph, expanded: !graph.expanded } : graph
      )
    );
  };
  
  // Remove a graph
  const removeGraph = (id: number) => {
    console.log(`Removing graph #${id}`);
    
    setGraphInstances(graphInstances.filter(graph => graph.id !== id));
  };
  
  // Update all graph durations
  const updateAllGraphDurations = (duration: number) => {
    console.log(`Updating all graphs to use duration: ${duration}ms`);
    setGraphDuration(duration);
  };
  
  // Listen for duration changes from DataBufferManager
  useEffect(() => {
    // This would typically be set up as an event listener or using a context
    const handleWindowDurationChange = (event: CustomEvent) => {
      const newDuration = event.detail.duration;
      updateAllGraphDurations(newDuration);
    };
    
    window.addEventListener('windowDurationChange', handleWindowDurationChange as EventListener);
    
    return () => {
      window.removeEventListener('windowDurationChange', handleWindowDurationChange as EventListener);
    };
  }, []);
  
  // Get device name for display
  const getDeviceName = (deviceId: string) => {
    const device = devices.find(d => d.id === deviceId);
    return device ? device.name : deviceId;
  };
  
  // Get device availability
  const getDeviceAvailability = (deviceId: string) => {
    const device = devices.find(d => d.id === deviceId);
    return device?.available || false;
  };

  if (isLoading) {
    return (
      <Box 
        display="flex" 
        justifyContent="center" 
        alignItems="center" 
        minHeight="100vh"
      >
        <CircularProgress />
        <Typography variant="h6" component="div" sx={{ ml: 2 }}>
          Loading data...
        </Typography>
      </Box>
    );
  }

  return (
    <div className="flex flex-col h-screen overflow-hidden">
      {/* Header with buffer manager and recording controls */}
      <div className="bg-gray-100 p-4 border-b">
        <DataBufferManager />
        <Box display="flex" justifyContent="space-between" mt={2}>
          <Box display="flex" alignItems="center">
            <Button 
              variant="contained" 
              color="primary"
              startIcon={<AddIcon />}
              onClick={addGraph}
              className="mr-4"
            >
              Add Graph
            </Button>
            
            {/* Debug info */}
            <Typography variant="body2" color="text.secondary" className="mr-4">
              Graph Duration: {(graphDuration / 1000).toFixed(0)}s
            </Typography>
            
            <FormControlLabel
              control={
                <Switch
                  checked={useSimpleCharts}
                  onChange={(e) => setUseSimpleCharts(e.target.checked)}
                  color="primary"
                />
              }
              label="Use Simple Charts"
            />
          </Box>
          
          {/* Recording Controls */}
          <RecordingControls />
        </Box>
      </div>
      
      {/* Main content area */}
      <div className="flex flex-1 overflow-hidden">
        {/* Left area with graphs */}
        <div className="flex-1 p-4 overflow-y-auto">
          {/* Graphs section */}
          <div className="grid gap-6 grid-cols-1">
            {graphInstances.length === 0 ? (
              <Paper elevation={2} className="p-8 text-center">
                <Typography variant="h6" color="text.secondary" gutterBottom>
                  No graphs added yet
                </Typography>
                <Button
                  variant="contained"
                  color="primary"
                  startIcon={<AddIcon />}
                  onClick={addGraph}
                >
                  Add Graph
                </Button>
              </Paper>
            ) : (
              graphInstances.map(graph => (
                <Paper key={graph.id} elevation={2} className="overflow-hidden">
                  {/* Graph header */}
                  <div className="flex justify-between items-center p-4 bg-gray-50 border-b">
                    <div className="flex items-center">
                      <IconButton 
                        size="small" 
                        onClick={() => toggleGraphExpanded(graph.id)}
                      >
                        {graph.expanded ? <ExpandLessIcon /> : <ExpandMoreIcon />}
                      </IconButton>
                      
                      <Typography variant="subtitle1" className="font-semibold ml-2">
                        Graph {graph.id}
                      </Typography>
                      
                      {graph.deviceId && (
                        <Chip
                          label={getDeviceName(graph.deviceId)}
                          size="small"
                          className="ml-3"
                          color={getDeviceAvailability(graph.deviceId) ? "success" : "error"}
                        />
                      )}
                      
                      <Typography variant="body2" color="text.secondary" className="ml-3">
                        {graph.selectedChannels.length} channel{graph.selectedChannels.length !== 1 ? 's' : ''} selected
                      </Typography>
                    </div>
                    
                    <IconButton 
                      onClick={() => removeGraph(graph.id)}
                      color="error"
                      size="small"
                      disabled={graphInstances.length <= 1}
                    >
                      <CloseIcon />
                    </IconButton>
                  </div>
                  
                  {/* Graph content (only show if expanded) */}
                  {graph.expanded && (
                    <div className="flex flex-col md:flex-row">
                      {/* Left panel with selectors */}
                      <div className="w-full md:w-64 p-4 border-r">
                        <DeviceSelector
                          selectedDeviceId={graph.deviceId}
                          onDeviceChange={(deviceId) => updateGraphDevice(graph.id, deviceId)}
                        />
                        
                        <Divider className="my-4" />
                        
                        <ChannelSelector
                          selectedDeviceId={graph.deviceId}
                          selectedChannels={graph.selectedChannels}
                          onSelectionChange={(channels) => updateGraphChannels(graph.id, channels)}
                        />
                      </div>
                      
                      {/* Right panel with graph */}
                      <div className="flex-1 p-4">
                        {graph.selectedChannels.length > 0 ? (
                          useSimpleCharts ? (
                            <div className="bg-gray-50 p-4 rounded-md border">
                              <Typography color="text.secondary" align="center">
                                Simple Charts not implemented yet
                              </Typography>
                            </div>
                          ) : (
                            <MultiViewGraph 
                              channelNames={graph.selectedChannels} 
                              duration={graphDuration}
                              height={250}
                            />
                          )
                        ) : (
                          <Box 
                            display="flex" 
                            justifyContent="center" 
                            alignItems="center" 
                            height={250}
                            bgcolor="grey.100"
                            borderRadius={1}
                          >
                            <Typography color="text.secondary">
                              Select channels to display
                            </Typography>
                          </Box>
                        )}
                      </div>
                    </div>
                  )}
                </Paper>
              ))
            )}
          </div>
        </div>
        
        {/* Right sidebar with numeric values for selected channels */}
        <div className="w-72 bg-white border-l p-4 overflow-y-auto">
          <Typography variant="h6" gutterBottom>
            Selected Channel Values
          </Typography>
          
          {graphInstances.length === 0 ? (
            <Typography color="text.secondary" className="italic text-center mt-4">
              No graphs created
            </Typography>
          ) : (
            <>
              {graphInstances.map(graph => {
                if (graph.selectedChannels.length === 0) return null;
                
                // Filter channels for this graph
                const graphChannels = channels.filter(channel => 
                  graph.selectedChannels.includes(channel.name)
                );
                
                return (
                  <div key={graph.id} className="mb-4">
                    <Typography variant="subtitle2" className="mt-2 mb-1 font-semibold">
                      Graph {graph.id} - {getDeviceName(graph.deviceId)}
                    </Typography>
                    
                    <NumericDisplay channels={graphChannels} />
                    
                    <Divider className="my-3" />
                  </div>
                );
              })}
              
              {graphInstances.every(graph => graph.selectedChannels.length === 0) && (
                <Typography color="text.secondary" className="italic text-center mt-4">
                  No channels selected in any graph
                </Typography>
              )}
            </>
          )}
        </div>
      </div>
    </div>
  );
};

export default DataPage;