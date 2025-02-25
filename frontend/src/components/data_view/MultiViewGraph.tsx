// Force draw all points option for debugging
const forceDrawAllPoints = true;// src/components/data_view/MultiViewGraph.tsx
import React, { useEffect, useRef } from 'react';
import { Chart, ChartConfiguration, ChartDataset, registerables } from 'chart.js';
import 'chartjs-plugin-streaming';
import { StreamingPlugin } from 'chartjs-plugin-streaming';
import { useDataContext } from '../../hooks/useDataContext';
import { Channel, TimeValue } from '../shared/types';

// Import necessary adapters
import 'chartjs-adapter-date-fns'; // This is important for time-based charts

// Register Chart.js components and plugins
Chart.register(...registerables);
Chart.register(StreamingPlugin);

interface MultiViewGraphProps {
  channelNames: string[];
  duration?: number; // milliseconds to display
  height?: string | number;
}

const MultiViewGraph: React.FC<MultiViewGraphProps> = ({ 
  channelNames, 
  duration = 20000, // 20 seconds default
  height = 300
}) => {
  const chartRef = useRef<HTMLCanvasElement | null>(null);
  const chartInstance = useRef<Chart<"line"> | null>(null);
  const { channels } = useDataContext();

  useEffect(() => {
    if (!chartRef.current || !channels.length || channelNames.length === 0) {
      return;
    }

    // Clean up previous chart instance to avoid the "Canvas is already in use" error
    if (chartInstance.current) {
      chartInstance.current.destroy();
      chartInstance.current = null;
    }

    // Filter selected channels
    const selectedChannels = channels.filter(channel => 
      channelNames.includes(channel.name)
    );

    // Set up datasets with simple time-based data
    const datasets: ChartDataset<"line">[] = selectedChannels.map((channel, index) => ({
      label: channel.name,
      backgroundColor: `hsla(${index * 30}, 70%, 50%, 0.2)`,
      borderColor: `hsla(${index * 30}, 70%, 50%, 1)`,
      borderWidth: 1.5,
      pointRadius: 1, // Disable points for performance
      data: channel.samples.map(sample => ({
        x: sample.timestamp,
        y: sample.value
      })),
      tension: 0.2 // Slight smoothing
    }));

    // Create new chart
    const ctx = chartRef.current.getContext('2d');
    if (!ctx) return;

    // Use a simple line chart with a basic configuration
    const config = {
      type: 'line',
      data: { datasets },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        plugins: {
          legend: {
            position: 'top',
            labels: {
              font: { size: 11 }
            }
          },
          tooltip: {
            mode: 'nearest',
            intersect: false
          }
        },
        scales: {
          x: {
            type: 'linear',
            min: Date.now() - 20000, // Show the last 20 seconds
            max: Date.now(),
            ticks: {
              maxRotation: 0,
              autoSkip: true,
              callback: function(value) {
                // Format timestamp as HH:MM:SS
                const date = new Date(value);
                return date.toTimeString().substring(0, 8);
              }
            }
          },
          y: {
            beginAtZero: false,
            suggestedMin: Math.min(...selectedChannels.map(c => c.min_value || 0)),
            suggestedMax: Math.max(...selectedChannels.map(c => c.max_value || 100))
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
    } catch (error) {
      console.error("Error creating chart:", error);
    }

    // Function to push new data to chart
    const pushData = () => {
      if (!chartInstance.current) return;
      
      selectedChannels.forEach((channel, index) => {
        if (!chartInstance.current?.data.datasets?.[index]) return;
        
        const chartData = chartInstance.current.data.datasets[index].data as { x: number, y: number }[];
        const existingTimestamps = new Set(chartData.map(point => point.x));
        
        // Find samples not yet in chart
        const newSamples = channel.samples
          .filter(sample => !existingTimestamps.has(sample.timestamp))
          .map(sample => ({ x: sample.timestamp, y: sample.value }));
        
        if (newSamples.length > 0) {
          // Append new samples
          chartData.push(...newSamples);
          
          // Sort data by timestamp
          chartData.sort((a, b) => a.x - b.x);
          
          // Keep data array size reasonable
          const maxPoints = 1000;
          if (chartData.length > maxPoints) {
            chartInstance.current.data.datasets[index].data = chartData.slice(-maxPoints);
          }
        }
      });
      
      chartInstance.current.update('quiet');
    };

    // Set up data update interval (10Hz)
    const updateInterval = setInterval(() => {
      if (!chartInstance.current) return;
      
      try {
        // Update each dataset with new data points
        selectedChannels.forEach((channel, index) => {
          if (!chartInstance.current?.data.datasets?.[index]) return;
          
          // Clear existing data and add all current samples
          chartInstance.current.data.datasets[index].data = channel.samples
            .map(sample => ({ x: sample.timestamp, y: sample.value }));
        });
        
        // Update x-axis min and max to scroll with time
        const now = Date.now();
        if (chartInstance.current.options.scales?.['x']) {
          chartInstance.current.options.scales['x'].min = now - 20000; // 20 seconds of data
          chartInstance.current.options.scales['x'].max = now;
        }
        
        chartInstance.current.update();
      } catch (error) {
        console.error("Error updating chart:", error);
      }
    }, 100);
    
    // Cleanup on unmount
    return () => {
      clearInterval(updateInterval);
      if (chartInstance.current) {
        chartInstance.current.destroy();
        chartInstance.current = null;
      }
    };
  }, [channels, channelNames]);

  return (
    <div style={{ height: typeof height === 'number' ? `${height}px` : height }}>
      <canvas ref={chartRef} />
    </div>
  );
};

export default React.memo(MultiViewGraph);