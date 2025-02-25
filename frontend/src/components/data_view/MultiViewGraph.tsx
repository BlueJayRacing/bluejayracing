// src/components/data_view/MultiViewGraph.tsx
import React, { useEffect, useRef } from 'react';
import { Chart, ChartConfiguration, ChartDataset, registerables } from 'chart.js';
import 'chartjs-plugin-streaming';
import { StreamingPlugin } from 'chartjs-plugin-streaming';
import { useDataContext } from '../../hooks/useDataContext';
import { Channel, TimeValue } from '../shared/types';

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
      pointRadius: 0, // Disable points for performance
      data: channel.samples.map(sample => ({
        x: sample.timestamp,
        y: sample.value
      })),
      tension: 0.2 // Slight smoothing
    }));

    // Create new chart
    const ctx = chartRef.current.getContext('2d');
    if (!ctx) return;

    // Use a simple line chart with time scale instead of realtime plugin for now
    const config: ChartConfiguration<"line"> = {
      type: 'line',
      data: { datasets },
      options: {
        animation: { duration: 0 }, // Disable animation for performance
        scales: {
          x: {
            type: 'time',
            time: {
              unit: 'second',
              displayFormats: {
                second: 'HH:mm:ss'
              }
            },
            ticks: {
              maxRotation: 0,
              autoSkipPadding: 20
            }
          },
          y: {
            beginAtZero: false,
            // Dynamic y-axis range based on data
            suggestedMin: Math.min(...selectedChannels.map(c => c.min_value || 0)),
            suggestedMax: Math.max(...selectedChannels.map(c => c.max_value || 100)),
            ticks: {
              maxTicksLimit: 8
            }
          }
        },
        plugins: {
          legend: {
            position: 'top',
            labels: {
              usePointStyle: true,
              boxWidth: 6,
              font: { size: 11 }
            }
          },
          tooltip: {
            mode: 'nearest',
            intersect: false,
            callbacks: {
              label: (context) => {
                const value = context.parsed.y;
                return `${context.dataset.label}: ${value.toFixed(2)}`;
              }
            }
          }
        },
        responsive: true,
        maintainAspectRatio: false,
        interaction: {
          mode: 'nearest',
          axis: 'x',
          intersect: false
        }
      }
    };

    chartInstance.current = new Chart(ctx, config);

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
      
      // Update each dataset with new data points
      selectedChannels.forEach((channel, index) => {
        if (!chartInstance.current?.data.datasets?.[index]) return;
        
        // Clear existing data and add all current samples
        chartInstance.current.data.datasets[index].data = channel.samples
          .map(sample => ({ x: sample.timestamp, y: sample.value }));
      });
      
      chartInstance.current.update('quiet');
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