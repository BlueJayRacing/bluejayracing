// src/components/data_view/SimpleChartComponent.tsx

import React, { useEffect, useRef, useState } from 'react';
import { Line } from 'react-chartjs-2';
import { Chart, registerables } from 'chart.js';
import { useDataContext } from '../../hooks/useDataContext';
import { getChannelCategoryColor } from '../../config/deviceConfig';
import { Box, Typography, Paper } from '@mui/material';

// Register Chart.js components
Chart.register(...registerables);

// Import streaming plugin
import 'chartjs-adapter-date-fns';
import StreamingPlugin from 'chartjs-plugin-streaming';
Chart.register(StreamingPlugin);

interface SimpleChartProps {
  channelName: string;
  height?: number;
  showValue?: boolean;
  timeWindow?: number;
}

const SimpleChart: React.FC<SimpleChartProps> = ({
  channelName,
  height = 80,
  showValue = false,
  timeWindow = 10000 // 10 seconds default
}) => {
  const { channels, getAllDataForChannel, getAllNewData } = useDataContext();
  const chartRef = useRef<any>(null);
  const [data, setData] = useState<Array<{x: number, y: number}>>([]);
  const [currentValue, setCurrentValue] = useState<number | null>(null);
  
  // Get device and channel information
  const getDeviceAndChannelName = (fullName: string): { deviceId: string, channelName: string } => {
    const parts = fullName.split('/');
    if (parts.length < 2) {
      return { deviceId: 'unknown', channelName: fullName };
    }
    return { deviceId: parts[0], channelName: parts[1] };
  };
  
  // Get channel display name (without device prefix)
  const { channelName: displayName } = getDeviceAndChannelName(channelName);
  
  // Get channel details
  const channel = channels.find(c => c.name === channelName);
  
  // Get channel category and color
  const getChannelCategory = (name: string): string => {
    if (name.includes("linpot_")) return "Potentiometers";
    if (name.includes("wheel_speed_")) return "Wheel Speeds";
    if (name.includes("brake_pressure_")) return "Brake Pressure";
    if (name.includes("steering_")) return "Steering";
    if (name.includes("axle_")) return "Axle";
    if (name.includes("temperature_")) return "Temperature";
    if (name.includes("pressure_")) return "Pressure";
    if (name.includes("imu_")) return "IMU";
    if (name.includes("gps_")) return "GPS";
    if (name.includes("Channel_")) return "WFT";
    
    return "Other";
  };
  
  const category = getChannelCategory(displayName);
  const color = getChannelCategoryColor(category);
  
  // Format the current value
  const formatValue = (value: number): string => {
    if (Math.abs(value) < 0.01) {
      return value.toFixed(5);
    } else if (Math.abs(value) < 10) {
      return value.toFixed(3);
    } else if (Math.abs(value) < 100) {
      return value.toFixed(2);
    } else if (Math.abs(value) < 1000) {
      return value.toFixed(1);
    } else {
      return value.toFixed(0);
    }
  };
  
  // Load initial data
  useEffect(() => {
    const loadInitialData = async () => {
      const channelData = getAllDataForChannel(channelName);
      
      if (channelData && channelData.length > 0) {
        // Convert to chart data points
        const points = channelData.map(sample => ({
          x: sample.timestamp,
          y: sample.value
        }));
        
        // Sort by timestamp
        points.sort((a, b) => a.x - b.x);
        
        // Set the current value
        if (points.length > 0) {
          setCurrentValue(points[points.length - 1].y);
        }
        
        // Update state
        setData(points);
      }
    };
    
    loadInitialData();
  }, [channelName]);
  
  // Chart refresh callback
  const handleRefresh = (chart: any) => {
    // Get new data for this channel
    const result = getAllNewData([channelName])[channelName];
    
    if (result && result.hasNewData && result.newSamples.length > 0) {
      // Convert to chart points
      const newPoints = result.newSamples.map(sample => ({
        x: sample.timestamp,
        y: sample.value
      }));
      
      // Update current value
      if (newPoints.length > 0) {
        setCurrentValue(newPoints[newPoints.length - 1].y);
      }
      
      // Add to chart data
      const now = Date.now();
      const oldestVisibleTime = now - timeWindow - 1000; // Add 1s buffer
      
      // Filter to visible range only
      const newData = [
        ...data,
        ...newPoints
      ].filter(point => point.x >= oldestVisibleTime);
      
      // Sort by timestamp
      newData.sort((a, b) => a.x - b.x);
      
      // Update state
      setData(newData);
      
      // Update chart
      chart.data.datasets[0].data = newData;
      chart.update('none');
    }
  };
  
  // Chart options
  const options = {
    responsive: true,
    maintainAspectRatio: false,
    interaction: {
      mode: 'nearest',
      intersect: false,
      axis: 'x'
    },
    plugins: {
      legend: {
        display: false
      },
      tooltip: {
        enabled: true,
        mode: 'nearest',
        intersect: false
      }
    },
    scales: {
      x: {
        type: 'realtime',
        realtime: {
          duration: timeWindow,
          refresh: 100,
          delay: 1000,
          onRefresh: handleRefresh
        },
        display: false
      },
      y: {
        beginAtZero: channel?.min_value === 0,
        display: false,
        suggestedMin: channel?.min_value,
        suggestedMax: channel?.max_value
      }
    },
    elements: {
      point: {
        radius: 0
      },
      line: {
        tension: 0.3,
        borderWidth: 1.5
      }
    },
    animation: false
  };
  
  // Chart data
  const chartData = {
    datasets: [
      {
        data,
        borderColor: color,
        backgroundColor: `${color}33`,
        fill: true,
        cubicInterpolationMode: 'monotone'
      }
    ]
  };
  
  return (
    <Paper
      elevation={0}
      className="p-2 border"
      sx={{ 
        borderLeft: `4px solid ${color}`,
        height: showValue ? height + 24 : height,
        backgroundColor: 'rgba(0, 0, 0, 0.02)'
      }}
    >
      {showValue && (
        <Box display="flex" justifyContent="space-between" alignItems="center" mb={0.5}>
          <Typography variant="body2" fontWeight="medium">
            {displayName}
          </Typography>
          
          {currentValue !== null && (
            <Typography variant="body2" fontWeight="bold" fontFamily="monospace">
              {formatValue(currentValue)}
            </Typography>
          )}
        </Box>
      )}
      
      <Box height={height}>
        <Line
          ref={chartRef}
          data={chartData}
          options={options as any}
        />
      </Box>
    </Paper>
  );
};

export default React.memo(SimpleChart);
