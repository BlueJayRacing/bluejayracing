// src/components/data_view/MultiViewGraph.tsx
import React, { useEffect, useRef, useState } from 'react';
import { Line } from 'react-chartjs-2';
import { Chart, registerables } from 'chart.js';
import { useDataContext } from '../../hooks/useDataContext';
import { Channel, TimeValue } from '../shared/types';

// Register necessary Chart.js components
Chart.register(...registerables);

// Import streaming plugin and register it with Chart
import 'chartjs-adapter-date-fns';
import StreamingPlugin from 'chartjs-plugin-streaming';
Chart.register(StreamingPlugin);

interface MultiViewGraphProps {
  channelNames: string[];
  duration?: number; // milliseconds to display
  height?: string | number;
  customData?: Channel[]; // For playback mode
}

const MultiViewGraph: React.FC<MultiViewGraphProps> = ({ 
  channelNames, 
  duration = 20000, // 20 seconds default
  height = 300,
  customData
}) => {
  const chartRef = useRef<any>(null);
  const { channels, getAllNewData, getAllDataForChannel } = useDataContext();
  const [datasetState, setDatasetState] = useState<{[key: string]: any[]}>({});
  const lastUpdateRef = useRef<number>(Date.now());
  const frameCountRef = useRef<number>(0);
  const chartInstanceRef = useRef<any>(null);
  const prevChannelNamesRef = useRef<string[]>([]);
  const isInitializedRef = useRef<{[key: string]: boolean}>({});
  
  // Filter selected channels
  const selectedChannels = (customData || channels).filter(
    channel => channelNames.includes(channel.name)
  );
  
  // Create datasets - important to keep dataset indexes consistent with channelNames
  const datasets = channelNames.map((channelName, index) => {
    const hue = (index * 30) % 360;
    
    return {
      label: channelName,
      backgroundColor: `hsla(${hue}, 70%, 50%, 0.2)`,
      borderColor: `hsla(${hue}, 70%, 50%, 1)`,
      borderWidth: 1.5,
      fill: false,
      tension: 0.2,
      pointRadius: 0,
      data: datasetState[channelName] || [] // Use state-backed data array
    };
  });
  
  // Load complete data for a channel
  const loadFullChannelData = (channelName: string) => {
    // Get full data for the channel
    if (!isInitializedRef.current[channelName]) {
      const fullData = getAllDataForChannel(channelName);
      
      if (fullData && fullData.length > 0) {
        console.log(`Loading full data for ${channelName}: ${fullData.length} samples`);
        
        // Convert to chart points
        const chartPoints = fullData.map(sample => ({
          x: sample.timestamp,
          y: sample.value
        }));
        
        // Sort by timestamp
        chartPoints.sort((a, b) => a.x - b.x);
        
        // Update dataset state
        setDatasetState(prev => ({
          ...prev,
          [channelName]: chartPoints
        }));
        
        // Update chart if available
        if (chartInstanceRef.current) {
          const datasetIndex = channelNames.indexOf(channelName);
          if (datasetIndex >= 0 && chartInstanceRef.current.data.datasets[datasetIndex]) {
            chartInstanceRef.current.data.datasets[datasetIndex].data = chartPoints;
            chartInstanceRef.current.update('none');
          }
        }
        
        // Mark as initialized
        isInitializedRef.current[channelName] = true;
      } else {
        console.log(`No full data available for ${channelName}`);
      }
    }
  };
  
  // Handle channel selection changes
  useEffect(() => {
    // Get the previous channel names
    const prevChannels = prevChannelNamesRef.current;
    
    // Find added and removed channels
    const addedChannels = channelNames.filter(name => !prevChannels.includes(name));
    const removedChannels = prevChannels.filter(name => !channelNames.includes(name));
    
    // Only process changes if anything actually changed
    if (addedChannels.length > 0 || removedChannels.length > 0) {
      console.log(
        `Channel selection changed: added ${addedChannels.join(', ')} | removed ${removedChannels.join(', ')}`
      );

      // Update chart instance if we have one
      if (chartInstanceRef.current) {
        // Update the datasets array based on the new channelNames
        const currentDatasets = [...chartInstanceRef.current.data.datasets];
        const newDatasets = channelNames.map((channelName, index) => {
          // If this is a new channel, create a new dataset
          if (addedChannels.includes(channelName)) {
            const hue = (index * 30) % 360;
            return {
              label: channelName,
              backgroundColor: `hsla(${hue}, 70%, 50%, 0.2)`,
              borderColor: `hsla(${hue}, 70%, 50%, 1)`,
              borderWidth: 1.5,
              fill: false,
              tension: 0.2,
              pointRadius: 0,
              data: [] // Empty initially, will be populated
            };
          }
          
          // If it's an existing channel, find its current dataset and reuse it
          const existingDatasetIndex = prevChannels.indexOf(channelName);
          if (existingDatasetIndex >= 0 && existingDatasetIndex < currentDatasets.length) {
            return currentDatasets[existingDatasetIndex];
          }
          
          // Fallback (shouldn't happen)
          const hue = (index * 30) % 360;
          return {
            label: channelName,
            backgroundColor: `hsla(${hue}, 70%, 50%, 0.2)`,
            borderColor: `hsla(${hue}, 70%, 50%, 1)`,
            borderWidth: 1.5,
            fill: false,
            tension: 0.2,
            pointRadius: 0,
            data: []
          };
        });
        
        // Update the chart with the new datasets
        chartInstanceRef.current.data.datasets = newDatasets;
        chartInstanceRef.current.update('none');
      }
      
      // Handle new channels
      addedChannels.forEach(channelName => {
        // Mark as not initialized
        isInitializedRef.current[channelName] = false;
        
        // Load full data for this channel
        loadFullChannelData(channelName);
      });
      
      // Remove data for removed channels
      if (removedChannels.length > 0) {
        setDatasetState(prev => {
          const newState = {...prev};
          // Delete removed channels
          removedChannels.forEach(channelName => {
            delete newState[channelName];
            delete isInitializedRef.current[channelName];
          });
          return newState;
        });
      }
    }
    
    // Update previous channel names for next comparison
    prevChannelNamesRef.current = [...channelNames];
  }, [channelNames.join(',')]);
  
  // On refresh callback for the realtime chart
  const handleRefresh = (chart: any) => {
    // Store the chart instance for later use
    if (!chartInstanceRef.current && chart) {
      chartInstanceRef.current = chart;
      
      // Check for any uninitialized channels and load their data
      channelNames.forEach(channelName => {
        if (!isInitializedRef.current[channelName]) {
          loadFullChannelData(channelName);
        }
      });
    }
    
    if (customData) return; // Don't update if using custom data
    
    // Increment frame count for debug purposes
    frameCountRef.current += 1;
    
    // Throttle updates to avoid excessive processing
    const now = Date.now();
    if (now - lastUpdateRef.current < 100) return; // Max 10Hz refresh rate
    
    // Log occasional debug info
    const shouldLogDebug = frameCountRef.current % 30 === 0;
    if (shouldLogDebug) {
      console.log(`Chart refresh frame #${frameCountRef.current}, time since last update: ${now - lastUpdateRef.current}ms`);
    }
    
    lastUpdateRef.current = now;
    
    // Get all new data for selected channels
    const newData = getAllNewData(channelNames);
    let hasUpdates = false;
    let totalNewPoints = 0;
    
    const datasetUpdates: {[key: string]: any[]} = {};
    
    // Process new data for each channel
    Object.values(newData).forEach(result => {
      if (result.hasNewData && result.newSamples.length > 0) {
        const channelName = result.channelName;
        
        // Find the corresponding dataset in the chart
        const datasetIndex = channelNames.indexOf(channelName);
        if (datasetIndex >= 0 && chart.data.datasets[datasetIndex]) {
          const dataset = chart.data.datasets[datasetIndex];
          
          // Convert samples to data points 
          const newPoints = result.newSamples.map(sample => ({
            x: sample.timestamp,
            y: sample.value
          }));
          
          totalNewPoints += newPoints.length;
          
          // Create our update object with the current data plus new points
          const currentData = [...(dataset.data || [])];
          const updatedData = [...currentData, ...newPoints];
          
          // Sort by timestamp to ensure correct order
          updatedData.sort((a: any, b: any) => a.x - b.x);
          
          // Remove data points outside the visible window
          // This helps maintain chart performance
          const oldestVisibleTime = now - duration - 1000; // 1 second buffer
          const validData = updatedData.filter((point: any) => point.x >= oldestVisibleTime);
          
          // Update the dataset directly
          dataset.data = validData;
          
          // Track that we need to update our state
          datasetUpdates[channelName] = [...validData];
          hasUpdates = true;
        }
      }
    });
    
    // Update dataset state if we have new data
    if (hasUpdates) {
      if (shouldLogDebug) {
        console.log(`Adding ${totalNewPoints} new points across ${Object.keys(datasetUpdates).length} channels`);
      }
      
      setDatasetState(prev => {
        const newState = {...prev};
        // Only update the changed channels
        Object.keys(datasetUpdates).forEach(channelName => {
          newState[channelName] = datasetUpdates[channelName];
        });
        return newState;
      });
      
      chart.update('none'); // Update without animation for performance
    } else if (shouldLogDebug) {
      // Log if we're not getting updates
      console.log("No new data points received for chart in this refresh");
    }
  };
  
  // Chart options with streaming configuration
  const options = {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    parsing: false, // Disable parsing for better performance
    scales: {
      x: {
        type: 'realtime',
        realtime: {
          duration: duration,
          refresh: 100, // 10Hz refresh
          delay: 2000,
          ttl: duration * 1.2, // Time to live slightly longer than duration
          frameRate: 30, // Reduce for better performance
          onRefresh: handleRefresh,
          pause: !!customData
        },
        ticks: {
          source: 'auto',
          autoSkip: true,
          maxRotation: 0
        }
      },
      y: {
        beginAtZero: false,
        ticks: {
          precision: 1
        }
      }
    },
    plugins: {
      legend: {
        position: 'top',
        labels: {
          font: {
            size: 10
          },
          boxWidth: 12
        }
      },
      tooltip: {
        mode: 'nearest',
        intersect: false
      }
    },
    interaction: {
      mode: 'nearest',
      axis: 'x',
      intersect: false
    }
  };
  
  // Initialize datasets for custom data or when component first mounts
  useEffect(() => {
    // Skip if no channels selected or if it's not the first mount
    // (channel changes are handled by the other useEffect)
    if (channelNames.length === 0 || prevChannelNamesRef.current.length > 0) return;
    
    console.log("Initial component mount, initializing all channels");
    
    if (customData) {
      // For customData, initialize with all the data
      const initialData: {[key: string]: any[]} = {};
      
      customData.forEach(channel => {
        if (channelNames.includes(channel.name)) {
          initialData[channel.name] = channel.samples.map(sample => ({
            x: sample.timestamp,
            y: sample.value
          }));
          isInitializedRef.current[channel.name] = true;
        }
      });
      
      console.log("Initializing with custom data for channels:", Object.keys(initialData));
      setDatasetState(initialData);
    } else {
      // Initialize with full data for each channel
      const initialData: {[key: string]: any[]} = {};
      
      // Process each channel separately
      channelNames.forEach(channelName => {
        // Get all data for just this channel
        const fullData = getAllDataForChannel(channelName);
        
        if (fullData && fullData.length > 0) {
          // Convert to chart data points
          initialData[channelName] = fullData.map(sample => ({
            x: sample.timestamp,
            y: sample.value
          }));
          
          // Sort by timestamp
          initialData[channelName].sort((a, b) => a.x - b.x);
          
          console.log(`Initial full data for ${channelName}: ${fullData.length} samples`);
          isInitializedRef.current[channelName] = true;
        } else {
          // Initialize with empty array if no data
          initialData[channelName] = [];
          console.log(`No initial data for ${channelName}`);
          isInitializedRef.current[channelName] = false;
        }
      });
      
      if (Object.keys(initialData).length > 0) {
        console.log("Setting initial data for all channels");
        setDatasetState(initialData);
      }
    }
    
    // Update previous channel names
    prevChannelNamesRef.current = [...channelNames];
  }, []);
  
  // If no channels selected, display a message
  if (selectedChannels.length === 0) {
    return (
      <div style={{ 
        height: typeof height === 'number' ? `${height}px` : height,
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        backgroundColor: '#f5f5f5',
        borderRadius: '4px'
      }}>
        <span>No channels selected</span>
      </div>
    );
  }
  
  return (
    <div style={{ height: typeof height === 'number' ? `${height}px` : height }}>
      <Line 
        ref={chartRef}
        data={{ datasets }}
        options={options as any}
      />
    </div>
  );
};

export default React.memo(MultiViewGraph);