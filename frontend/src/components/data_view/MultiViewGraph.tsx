// src/components/data_view/MultiViewGraph.tsx
import React, { useEffect, useRef, useState } from 'react';
import { Line } from 'react-chartjs-2';
import { Chart, registerables } from 'chart.js';
import { useDataContext } from '../../hooks/useDataContext';
import { useTimestamp } from '../../contexts/TimestampContext';
import { Channel, TimeValue } from '../shared/types';
import { getChannelCategoryColor } from '../../config/deviceConfig';

// Register necessary Chart.js components
Chart.register(...registerables);

// Import streaming plugin and register it with Chart
import 'chartjs-adapter-date-fns';
import StreamingPlugin from 'chartjs-plugin-streaming';
Chart.register(StreamingPlugin);

// Helper function to determine channel category
const getChannelCategory = (channelName: string): string => {
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

interface MultiViewGraphProps {
  channelNames: string[];
  duration?: number; // milliseconds to display
  height?: string | number;
  customData?: Channel[]; // For playback mode
  showLegend?: boolean;
  realtime?: boolean;
}

const MultiViewGraph: React.FC<MultiViewGraphProps> = ({ 
  channelNames, 
  duration = 60000, // 60 seconds default
  height = 300,
  customData,
  showLegend = true,
  realtime = true
}) => {
  console.log(`MultiViewGraph rendering with duration=${duration}ms, channels=${channelNames.length}, realtime=${realtime}`);
  
  const chartRef = useRef<any>(null);
  const { channels, getAllNewData, getAllDataForChannel } = useDataContext();
  const { convertTimestamp, updateLastTimestamp } = useTimestamp();
  const [datasetState, setDatasetState] = useState<{[key: string]: any[]}>({});
  const lastUpdateRef = useRef<number>(Date.now());
  const frameCountRef = useRef<number>(0);
  const chartInstanceRef = useRef<any>(null);
  const prevChannelNamesRef = useRef<string[]>([]);
  const isInitializedRef = useRef<{[key: string]: boolean}>({});
  
  // For timestamp mapping
  const timeOffsetRef = useRef<{[key: string]: number}>({});
  const lastDisplayTimeRef = useRef<{[key: string]: number}>({});
  
  // For tracking processed samples
  const processedSamplesRef = useRef<{[key: string]: Set<number>}>({});
  
  // Debug data counts
  const dataPointsCountRef = useRef<{[key: string]: number}>({});
  
  // Get device and channel information
  const getDeviceAndChannelName = (fullName: string): { deviceId: string, channelName: string } => {
    const parts = fullName.split('/');
    if (parts.length < 2) {
      return { deviceId: 'unknown', channelName: fullName };
    }
    return { deviceId: parts[0], channelName: parts[1] };
  };
  
  // Filter selected channels
  const selectedChannels = (customData || channels).filter(
    channel => channelNames.includes(channel.name)
  );
  
  // Create datasets - keep dataset indexes consistent with channelNames
  const datasets = channelNames.map((channelName, index) => {
    // Extract channel info
    const { channelName: shortName } = getDeviceAndChannelName(channelName);
    
    // Get category from channel
    const category = getChannelCategory(shortName);
    
    // Get color based on category
    const categoryColor = getChannelCategoryColor(category);
    
    return {
      label: shortName,
      backgroundColor: `${categoryColor}33`, // Add transparency
      borderColor: categoryColor,
      borderWidth: 1.5,
      fill: false,
      tension: 0.2,
      pointRadius: 0,
      data: datasetState[channelName] || [] // Use state-backed data array
    };
  });
  
  // Map a device timestamp to display time
  const mapToDisplayTime = (timestamp: number, channelName: string): number => {
    // Normalize timestamp to milliseconds
    let normalizedTimestamp = timestamp;
    
    // Convert microseconds to milliseconds if needed
    if (normalizedTimestamp > 1000000000000) {
      normalizedTimestamp = Math.floor(normalizedTimestamp / 1000);
    }
    // Convert seconds to milliseconds if needed
    else if (normalizedTimestamp < 10000000) {
      normalizedTimestamp = normalizedTimestamp * 1000;
    }
    
    // Initialize time offset for this channel if needed
    if (!timeOffsetRef.current[channelName]) {
      // Calculate offset: current time minus the device timestamp
      // This aligns the device's time with current wall clock time
      const now = Date.now();
      timeOffsetRef.current[channelName] = now - normalizedTimestamp;
      
      console.log(`Initialized time offset for ${channelName}: ${timeOffsetRef.current[channelName]}ms`);
      console.log(`First timestamp ${normalizedTimestamp} mapped to ${new Date(normalizedTimestamp + timeOffsetRef.current[channelName]).toISOString()}`);
    }
    
    // Apply offset to get wall clock time
    const displayTime = normalizedTimestamp + timeOffsetRef.current[channelName];
    
    // Keep track of the latest display time for this channel
    if (!lastDisplayTimeRef.current[channelName] || displayTime > lastDisplayTimeRef.current[channelName]) {
      lastDisplayTimeRef.current[channelName] = displayTime;
    }
    
    return displayTime;
  };
  
  // Process new samples for a channel
  const processNewSamples = (channelName: string, samples: TimeValue[]): any[] => {
    if (!samples || samples.length === 0) return [];
    
    // Initialize processed set if needed
    if (!processedSamplesRef.current[channelName]) {
      processedSamplesRef.current[channelName] = new Set();
    }
    
    // Filter out samples we've already processed
    const newSamples = samples.filter(sample => 
      !processedSamplesRef.current[channelName].has(sample.timestamp)
    );
    
    if (newSamples.length === 0) return [];
    
    console.log(`Processing ${newSamples.length} new samples for ${channelName}`);
    
    // Mark these samples as processed
    newSamples.forEach(sample => {
      processedSamplesRef.current[channelName].add(sample.timestamp);
    });
    
    // Map to display points
    const displayPoints = newSamples.map(sample => {
      const displayTime = mapToDisplayTime(sample.timestamp, channelName);
      
      return {
        x: displayTime,
        y: sample.value,
        _original: {
          x: sample.timestamp,
          y: sample.value
        }
      };
    });
    
    console.log(`Added ${displayPoints.length} new display points for ${channelName}`);
    
    return displayPoints;
  };
  
  // Load initial data for a channel
  const loadChannelData = (channelName: string) => {
    try {
      if (!isInitializedRef.current[channelName]) {
        const fullData = getAllDataForChannel(channelName);
        
        if (fullData && fullData.length > 0) {
          console.log(`Loading data for ${channelName}: ${fullData.length} samples`);
          console.log(`Time range: ${fullData[0].timestamp} to ${fullData[fullData.length-1].timestamp}`);
          
          // Process the full data
          const displayPoints = processNewSamples(channelName, fullData);
          
          if (displayPoints.length > 0) {
            // Calculate the time span
            const timeSpan = (displayPoints[displayPoints.length-1].x - displayPoints[0].x) / 1000;
            console.log(`Display time span: ${timeSpan.toFixed(1)}s with ${displayPoints.length} points`);
            
            // Update dataset state
            setDatasetState(prev => ({
              ...prev,
              [channelName]: displayPoints
            }));
            
            // Update chart if available
            if (chartInstanceRef.current) {
              const datasetIndex = channelNames.indexOf(channelName);
              if (datasetIndex >= 0 && chartInstanceRef.current.data.datasets[datasetIndex]) {
                chartInstanceRef.current.data.datasets[datasetIndex].data = displayPoints;
                chartInstanceRef.current.update('none');
              }
            }
            
            // Track data points count
            dataPointsCountRef.current[channelName] = displayPoints.length;
          }
          
          // Mark as initialized
          isInitializedRef.current[channelName] = true;
        } else {
          console.log(`No data available for ${channelName}`);
        }
      }
    } catch (error) {
      console.error(`Error loading data for ${channelName}:`, error);
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
            // Extract channel info
            const { channelName: shortName } = getDeviceAndChannelName(channelName);
            
            // Get category from channel
            const category = getChannelCategory(shortName);
            
            // Get color based on category
            const categoryColor = getChannelCategoryColor(category);
            
            return {
              label: shortName,
              backgroundColor: `${categoryColor}33`, // Add transparency
              borderColor: categoryColor,
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
          
          // Fallback
          const { channelName: shortName } = getDeviceAndChannelName(channelName);
          const category = getChannelCategory(shortName);
          const categoryColor = getChannelCategoryColor(category);
          
          return {
            label: shortName,
            backgroundColor: `${categoryColor}33`, // Add transparency
            borderColor: categoryColor,
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
        
        // Load data for this channel
        loadChannelData(channelName);
      });
      
      // Remove data for removed channels
      if (removedChannels.length > 0) {
        setDatasetState(prev => {
          const newState = {...prev};
          // Delete removed channels
          removedChannels.forEach(channelName => {
            delete newState[channelName];
            delete isInitializedRef.current[channelName];
            delete timeOffsetRef.current[channelName];
            delete lastDisplayTimeRef.current[channelName];
            delete processedSamplesRef.current[channelName];
            delete dataPointsCountRef.current[channelName];
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
          loadChannelData(channelName);
        }
      });
      
      // Log chart configuration
      console.log('Chart initialized with options:', {
        duration: chart.options.scales.x.realtime.duration, 
        ttl: chart.options.scales.x.realtime.ttl,
        delay: chart.options.scales.x.realtime.delay,
        refresh: chart.options.scales.x.realtime.refresh,
        frameRate: chart.options.scales.x.realtime.frameRate
      });
    }
    
    if (customData || !realtime) return; // Don't update if using custom data or not in realtime mode
    
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
    
    // Process new data for each channel
    let hasUpdates = false;
    let totalNewPoints = 0;
    
    Object.values(newData).forEach(result => {
      if (result.hasNewData && result.newSamples.length > 0) {
        const channelName = result.channelName;
        
        // Find the corresponding dataset in the chart
        const datasetIndex = channelNames.indexOf(channelName);
        if (datasetIndex >= 0 && chart.data.datasets[datasetIndex]) {
          const dataset = chart.data.datasets[datasetIndex];
          
          // Get current data or initialize empty array
          if (!dataset.data) {
            dataset.data = [];
          }
          
          // Process new samples into display points
          const newPoints = processNewSamples(channelName, result.newSamples);
          totalNewPoints += newPoints.length;
          
          if (newPoints.length > 0) {
            // Add new points to dataset
            const combinedData = [...dataset.data, ...newPoints];
            
            // Sort by display timestamp
            combinedData.sort((a: any, b: any) => a.x - b.x);
            
            // Update dataset
            dataset.data = combinedData;
            
            // Track data points count
            dataPointsCountRef.current[channelName] = combinedData.length;
            
            hasUpdates = true;
            
            if (shouldLogDebug) {
              console.log(`Added ${newPoints.length} new points for ${channelName} (${result.newSamples.length} samples)`);
              
              if (combinedData.length > 0) {
                const timeSpan = (combinedData[combinedData.length - 1].x - combinedData[0].x) / 1000;
                console.log(`${channelName} data now spans ${timeSpan.toFixed(1)}s with ${combinedData.length} points`);
              }
            }
          } else if (shouldLogDebug && result.newSamples.length > 0) {
            console.log(`Received ${result.newSamples.length} samples for ${channelName} but all were duplicates`);
          }
        }
      }
    });
    
    // Trim old data points periodically
    if (frameCountRef.current % 100 === 0) {
      // The current time minus duration minus some buffer
      const cutoffTime = now - (duration * 3);
      let totalTrimmed = 0;
      
      // For each dataset, remove points older than the cutoff
      chart.data.datasets.forEach((dataset: any, index: number) => {
        if (index < channelNames.length && dataset.data) {
          const oldCount = dataset.data.length;
          
          // Filter out old points
          dataset.data = dataset.data.filter((point: any) => point.x >= cutoffTime);
          
          const trimmed = oldCount - dataset.data.length;
          totalTrimmed += trimmed;
          
          if (trimmed > 0) {
            const channelName = channelNames[index];
            dataPointsCountRef.current[channelName] = dataset.data.length;
          }
        }
      });
      
      if (totalTrimmed > 0) {
        console.log(`Trimmed ${totalTrimmed} old data points from chart datasets`);
      }
    }
    
    // Update chart if we have changes
    if (hasUpdates) {
      // Update state to match chart data
      const updatedState: {[key: string]: any[]} = {};
      chart.data.datasets.forEach((dataset: any, index: number) => {
        if (index < channelNames.length) {
          updatedState[channelNames[index]] = dataset.data;
        }
      });
      
      setDatasetState(updatedState);
      
      // Check visible time range in chart (debug only)
      if (shouldLogDebug && chart && chart.scales && chart.scales.x) {
        const visibleRange = {
          min: new Date(chart.scales.x.min).toISOString(),
          max: new Date(chart.scales.x.max).toISOString(),
          duration: (chart.scales.x.max - chart.scales.x.min) / 1000
        };
        console.log(`Visible time range: ${visibleRange.duration.toFixed(1)}s (${visibleRange.min} to ${visibleRange.max})`);
        
        // Log total data points in each channel
        const pointCounts = channelNames.map(name => ({
          channel: name,
          count: dataPointsCountRef.current[name] || 0
        }));
        
        console.log("Total data points per channel:", pointCounts);
      }
      
      // Update chart (no animation for better performance)
      chart.update('none');
    } else if (shouldLogDebug && totalNewPoints > 0) {
      console.log(`Received ${totalNewPoints} points but no unique new points to add`);
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
          delay: 100, // Minimal delay
          ttl: duration * 3, // Keep 3x the visible duration in memory
          frameRate: 30, // Reduce for better performance
          onRefresh: handleRefresh,
          pause: !!customData || !realtime
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
        display: showLegend,
        labels: {
          font: {
            size: 10
          },
          boxWidth: 12,
          padding: 8
        }
      },
      tooltip: {
        mode: 'nearest',
        intersect: false,
        backgroundColor: 'rgba(0, 0, 0, 0.7)',
        titleFont: {
          size: 12
        },
        bodyFont: {
          size: 11
        },
        padding: 8,
        displayColors: true,
        callbacks: {
          // Add callback to show original values in tooltip
          label: function(context: any) {
            const dataPoint = context.raw;
            let label = context.dataset.label || '';
            
            if (label) {
              label += ': ';
            }
            
            if (dataPoint && dataPoint._original) {
              // Show both display value and original value
              return [
                `${label}${context.formattedValue} (displayed)`,
                `Original: timestamp=${dataPoint._original.x}, value=${dataPoint._original.y}`
              ];
            }
            
            return `${label}${context.formattedValue}`;
          }
        }
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
          // Process channel data into display points
          initialData[channel.name] = processNewSamples(channel.name, channel.samples);
          isInitializedRef.current[channel.name] = true;
          dataPointsCountRef.current[channel.name] = initialData[channel.name].length;
        }
      });
      
      console.log("Initializing with custom data for channels:", Object.keys(initialData));
      setDatasetState(initialData);
    } else {
      // Initialize with full data for each channel
      channelNames.forEach(channelName => {
        loadChannelData(channelName);
      });
    }
    
    // Update previous channel names
    prevChannelNamesRef.current = [...channelNames];
  }, []);
  
  // If no channels selected, display a message
  if (selectedChannels.length === 0) {
    return (
      <div className="flex items-center justify-center h-full bg-gray-50 rounded-md border border-gray-200">
        <span className="text-gray-500">No channels selected</span>
      </div>
    );
  }
  
  return (
    <div className="h-full w-full">
      <Line 
        ref={chartRef}
        data={{ datasets }}
        options={options as any}
        className="rounded-md bg-white"
      />
    </div>
  );
};

export default React.memo(MultiViewGraph);