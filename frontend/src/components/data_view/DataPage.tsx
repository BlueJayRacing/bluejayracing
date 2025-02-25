// src/components/data_view/DataPage.tsx
import React, { useState, useRef, useEffect } from 'react';
import { useDataContext } from '../../hooks/useDataContext';
import MultiViewGraph from './MultiViewGraph';
import SimpleChartComponent from './SimpleChartComponent';
import ChannelSelector from './ChannelSelector';
import NumericDisplay from './NumericDisplay';
import DataBufferManager from './DataBufferManager';
import CarTerrainScene from '../car_terrain_scene/CarTerrainScene';
import { DEFAULT_TERRAIN_CONFIG } from '../car_terrain_scene/ct_configs/terrainConfig';
import { Button, CircularProgress, Box, Divider, IconButton, Paper, Typography, Switch, FormControlLabel } from '@mui/material';
import AddIcon from '@mui/icons-material/Add';
import CloseIcon from '@mui/icons-material/Close';

interface GraphInstance {
  id: number;
  selectedChannels: string[];
}

const DataPage: React.FC = () => {
  const { channels, isLoading } = useDataContext();
  const [graphInstances, setGraphInstances] = useState<GraphInstance[]>([
    { id: 1, selectedChannels: [] }
  ]);
  const cameraConfigRef = useRef<any>({ current: {} });
  const [useSimpleCharts, setUseSimpleCharts] = useState(false); // Default to regular charts


  const addGraph = () => {
    const newId = Math.max(0, ...graphInstances.map(g => g.id)) + 1;
    setGraphInstances([...graphInstances, { id: newId, selectedChannels: [] }]);
  };

  const updateGraphChannels = (id: number, selectedChannels: string[]) => {
    setGraphInstances(
      graphInstances.map(graph => 
        graph.id === id ? { ...graph, selectedChannels } : graph
      )
    );
  };

  const removeGraph = (id: number) => {
    setGraphInstances(graphInstances.filter(graph => graph.id !== id));
  };

  // Debug: Force mock data mode for testing
  useEffect(() => {
    console.log("Graph instances:", graphInstances);
    
    // After the component mounts, add some default channel selections for testing
    if (channels.length > 0 && graphInstances.length === 1 && graphInstances[0].selectedChannels.length === 0) {
      console.log("Setting default channel selections for testing");
      
      // Get available channel names
      const channelNames = channels.map(c => c.name);
      console.log("Available channels:", channelNames);
      
      // Select a few channels by default for the first graph
      const defaultSelections = [
        "linpot_front_left",
        "linpot_front_right",
        "brake_pressure_front"
      ].filter(name => channelNames.includes(name));
      
      if (defaultSelections.length > 0) {
        updateGraphChannels(1, defaultSelections);
      }
    }
  }, [channels, graphInstances]);

  const getChannelValue = (channelName: string): number => {
    const channel = channels.find(c => c.name === channelName);
    if (!channel || !channel.samples.length) return 0;
    return channel.samples[channel.samples.length - 1].value;
  };

  const getCarState = () => {
    // Transform channel data into car state format
    // This will map sensor data to car model parameters
    return {
      suspensionLengthsInInches: {
        frontLeft: getChannelValue('linpot_front_left'),
        frontRight: getChannelValue('linpot_front_right'),
        rearLeft: getChannelValue('linpot_rear_left'),
        rearRight: getChannelValue('linpot_rear_right')
      },
      wheelSpeedsInRPM: {
        frontLeft: getChannelValue('wheel_speed_fl'),
        frontRight: getChannelValue('wheel_speed_fr'),
        lockedRear: (getChannelValue('wheel_speed_rl') + getChannelValue('wheel_speed_rr')) / 2
      },
      steeringAngleInDegrees: getChannelValue('steering_angle'),
      framePose: {
        position: [0, 0, 0],
        rotation: [0, 0, 0, 1]
      }
    };
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

  // Get all selected channels across all graphs
  const allSelectedChannels = [...new Set(
    graphInstances.flatMap(graph => graph.selectedChannels)
  )];

  // Filter channels to show only selected ones
  const selectedChannelsData = channels.filter(
    channel => allSelectedChannels.includes(channel.name)
  );

  return (
    <div className="flex flex-col h-screen overflow-hidden">
      {/* Header with buffer manager and future recording controls */}
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
          
          <Button 
            variant="outlined"
            color="error"
            disabled={true} // Will be enabled with recording functionality
          >
            Start Recording
          </Button>
        </Box>
      </div>
      
      {/* Main content area */}
      <div className="flex flex-1 overflow-hidden">
        {/* Left area with graphs */}
        <div className="flex-1 p-4 overflow-y-auto">
          {/* Graphs section */}
          <div className="grid gap-6 grid-cols-1">
            {graphInstances.map(graph => (
              <Paper key={graph.id} elevation={2} className="p-4">
                <div className="flex justify-between items-center mb-4">
                  <Typography variant="subtitle1" className="font-semibold">
                    Graph {graph.id}
                  </Typography>
                  <IconButton 
                    onClick={() => removeGraph(graph.id)}
                    color="error"
                    size="small"
                    disabled={graphInstances.length <= 1}
                  >
                    <CloseIcon />
                  </IconButton>
                </div>
                
                <ChannelSelector
                  availableChannels={channels.map(c => c.name)}
                  selectedChannels={graph.selectedChannels}
                  onSelectionChange={(selected) => updateGraphChannels(graph.id, selected)}
                />
                
                <Divider className="my-4" />
                
                {graph.selectedChannels.length > 0 ? (
                  useSimpleCharts ? (
                    <SimpleChartComponent 
                      key={`graph-${graph.id}-simple-${Date.now()}`}
                      channelNames={graph.selectedChannels} 
                      height={250}
                    />
                  ) : (
                    <MultiViewGraph 
                      key={`graph-${graph.id}-multi-${Date.now()}`}
                      channelNames={graph.selectedChannels} 
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
              </Paper>
            ))}
          </div>
          
          {/* Car visualization - commented out for now */}
          {/* 
          <Paper elevation={2} className="mt-6 p-2">
            <Typography variant="h6" gutterBottom>Car Visualization</Typography>
            <div className="h-96">
              <CarTerrainScene
                carState={getCarState()}
                terrainConfig={DEFAULT_TERRAIN_CONFIG}
                cameraConfigRef={cameraConfigRef}
                modelPaths={{
                  car: '/models/vehicle.glb',
                  terrain: '/models/terrain.glb'
                }}
              />
            </div>
          </Paper>
          */}
        </div>
        
        {/* Right sidebar with numeric values for selected channels only */}
        <div className="w-72 bg-white border-l p-4 overflow-y-auto">
          <Typography variant="h6" gutterBottom>
            Selected Channel Values
          </Typography>
          {allSelectedChannels.length > 0 ? (
            <NumericDisplay channels={selectedChannelsData} />
          ) : (
            <Typography color="text.secondary" className="italic text-center mt-4">
              No channels selected
            </Typography>
          )}
        </div>
      </div>
    </div>
  );
}

export default DataPage;