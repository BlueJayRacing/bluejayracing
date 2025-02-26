// src/components/data_view/MultiViewGraph.tsx
import React, { useEffect, useRef } from 'react';
import { Chart, ChartConfiguration, ChartDataset, registerables } from 'chart.js';
import 'chartjs-plugin-streaming';
import { StreamingPlugin } from 'chartjs-plugin-streaming';
import { useDataContext } from '../../hooks/useDataContext';
import { Channel, TimeValue } from '../shared/types';
import { DATA_CONFIG, debugLog } from '../../config/dataConfig';

// Import necessary adapters
import 'chartjs-adapter-date-fns'; // This is important for time-based charts

// Register Chart.js components and plugins
Chart.register(...registerables);
Chart.register(StreamingPlugin);

// Only force draw points in playback mode
const forceDrawAllPoints = false;

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
  const chartRef = useRef<HTMLCanvasElement | null>(null);
  const chartInstance = useRef<Chart<"line"> | null>(null);
  const { channels, playbackMode } = useDataContext();
  const lastUpdateRef = useRef<number>(0);

  useEffect(() => {
    if (!chartRef.current) return;
    
    // Throttle updates to avoid excessive re-renders
    const now = Date.now();
    if (now - lastUpdateRef.current < 100 && chartInstance.current) {
      return;
    }
    lastUpdateRef.current = now;
    
    // Determine which data source to use
    const dataSource = customData || channels;
    
    // If no valid channels or names, don't proceed
    if (!dataSource.length || channelNames.length === 0) {
      return;
    }

    // Clean up previous chart instance to avoid the "Canvas is already in use" error
    if (chartInstance.current) {
      chartInstance.current.destroy();
      chartInstance.current = null;
    }

    // Filter selected channels
    const selectedChannels = dataSource.filter(channel => 
      channelNames.includes(channel.name)
    );

    debugLog('MULTI_VIEW_GRAPH', `Creating chart with ${selectedChannels.length} channels`);
    if (selectedChannels.length === 0) {
      debugLog('MULTI_VIEW_GRAPH', 'No matching channels found among:', 
        dataSource.map(c => c.name), 'Looking for:', channelNames);
      return;
    }

    // Find first valid timestamp for normalization in playback mode
    const firstTimestamp = playbackMode && selectedChannels.length > 0 && selectedChannels[0].samples.length > 0 ? 
      selectedChannels[0].samples[0].timestamp : 0;

    // Set up datasets with time-based data
    const datasets: ChartDataset<"line">[] = selectedChannels.map((channel, index) => {
      // Create a color based on channel index
      const hue = (index * 30) % 360; // Spread colors
      
      return {
        label: channel.name,
        backgroundColor: `hsla(${hue}, 70%, 50%, 0.2)`,
        borderColor: `hsla(${hue}, 70%, 50%, 1)`,
        borderWidth: 1.5,
        pointRadius: (playbackMode || customData) ? 1 : 0, // Show points only in playback mode
        tension: 0.1, // Slight smoothing
        data: channel.samples.map(sample => ({
          x: playbackMode || customData ? 
            // In playback mode, normalize timestamps relative to first sample
            sample.timestamp - firstTimestamp : 
            sample.timestamp,
          y: sample.value
        }))
      };
    });

    // Create new chart
    const ctx = chartRef.current.getContext('2d');
    if (!ctx) return;

    const currentTime = Date.now();
    
    // Calculate min and max for x-axis
    const minTime = playbackMode || customData ? 
      0 : // Start at 0 for playback
      currentTime - duration; // Use sliding window for live
      
    const maxTime = playbackMode || customData ?
      // For playback, use the max timestamp from data
      Math.max(
        1000, // Minimum 1 second range
        ...selectedChannels.flatMap(c => 
          c.samples.map(s => (playbackMode || customData) ? 
            s.timestamp - firstTimestamp : 
            s.timestamp)
        )
      ) + 500 : // Add a little buffer
      currentTime; // Use current time for live
    
    // Calculate min and max for y-axis with a bit of padding
    let minValue = Math.min(...selectedChannels.map(c => 
      c.min_value !== undefined ? c.min_value : 
      Math.min(...c.samples.map(s => s.value), 0)
    ));
    let maxValue = Math.max(...selectedChannels.map(c => 
      c.max_value !== undefined ? c.max_value : 
      Math.max(...c.samples.map(s => s.value), 100)
    ));
    
    // Add 10% padding to the range
    const valueRange = maxValue - minValue;
    minValue = minValue - valueRange * 0.1;
    maxValue = maxValue + valueRange * 0.1;

    const config: ChartConfiguration<'line'> = {
      type: 'line',
      data: { datasets },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        elements: {
          point: {
            radius: (playbackMode || customData) ? 1 : 0,
            hoverRadius: 5
          },
          line: {
            tension: 0.1
          }
        },
        plugins: {
          tooltip: {
            mode: 'nearest',
            intersect: false
          },
          legend: {
            position: 'top',
            labels: {
              boxWidth: 12,
              font: { size: 10 }
            }
          }
        },
        scales: {
          x: {
            type: 'linear',
            min: minTime,
            max: maxTime,
            ticks: {
              maxRotation: 0,
              autoSkip: true,
              callback: function(value) {
                if (playbackMode || customData) {
                  // Show seconds from start
                  return `${(Number(value) / 1000).toFixed(1)}s`;
                } else {
                  // Format timestamp as HH:MM:SS
                  const date = new Date(value as number);
                  return date.toTimeString().substring(0, 8);
                }
              }
            }
          },
          y: {
            beginAtZero: false,
            min: minValue,
            max: maxValue,
            ticks: {
              precision: 1
            }
          }
        },
        interaction: {
          mode: 'nearest',
          axis: 'x',
          intersect: false
        }
      }
    };

    try {
      chartInstance.current = new Chart(ctx, config as any);
      debugLog('MULTI_VIEW_GRAPH', 'Chart created successfully');
    } catch (error) {
      console.error("Error creating chart:", error);
    }

    // Update interval for live mode only
    if (!playbackMode && !customData) {
      // Set up data update interval (5Hz to reduce load)
      const updateInterval = setInterval(() => {
        if (!chartInstance.current) return;
        
        try {
          // Only update if we have new data
          let hasNewData = false;
          
          selectedChannels.forEach((channel, index) => {
            if (!chartInstance.current?.data.datasets?.[index]) return;
            
            // Check if we have any new data by comparing lengths
            const currentSamples = chartInstance.current.data.datasets[index].data.length;
            if (channel.samples.length !== currentSamples) {
              hasNewData = true;
              
              // Update the dataset with new samples
              chartInstance.current.data.datasets[index].data = channel.samples
                .map(sample => ({ x: sample.timestamp, y: sample.value }));
            }
          });
          
          // Update the x-axis range to keep scrolling
          if (chartInstance.current.options.scales?.['x']) {
            const now = Date.now();
            chartInstance.current.options.scales['x'].min = now - duration;
            chartInstance.current.options.scales['x'].max = now;
          }
          
          // Only call update if data changed
          if (hasNewData) {
            chartInstance.current.update('quiet'); // Use 'quiet' mode for better performance
          }
        } catch (error) {
          console.error("Error updating chart:", error);
        }
      }, 200); // 5Hz updates
      
      return () => clearInterval(updateInterval);
    }
    
    // Return cleanup function
    return () => {
      if (chartInstance.current) {
        chartInstance.current.destroy();
        chartInstance.current = null;
      }
    };
  }, [channels, channelNames, playbackMode, customData]);

  return (
    <div style={{ height: typeof height === 'number' ? `${height}px` : height }}>
      <canvas ref={chartRef} />
    </div>
  );
};

export default React.memo(MultiViewGraph);