// src/components/playback_view/PlaybackGraph.tsx
import React, { useEffect, useRef, useState, useCallback } from 'react';
import { Box, Typography, CircularProgress } from '@mui/material';
import { Channel } from '../../components/shared/types';

// Debug helper
const debugLog = (message: string, ...args: any[]) => {
  // console.log(`[PlaybackGraph] ${message}`, ...args);
};

interface PlaybackGraphProps {
  channelNames: string[];
  visibleData: Channel[];
  currentTime: number;
  totalDuration: number;
  recording: {
    id?: string;
    startTime: number;
    endTime: number | null;
  } | null;
  height?: number;
  onZoomChange?: (newDuration: number) => void;
  graphDuration?: number; // Duration in milliseconds to display
}

const PlaybackGraph: React.FC<PlaybackGraphProps> = ({ 
  channelNames, 
  visibleData,
  currentTime,
  totalDuration,
  height = 400,
  recording
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationFrameRef = useRef<number | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const lastDrawTime = useRef<number>(0);
  
  // Process data for rendering - memoize to prevent unnecessary recalculations
  const processedData = React.useMemo(() => {
    if (!visibleData || !channelNames.length) {
      return [];
    }

    return channelNames.map((channelName, index) => {
      const channel = visibleData.find(c => c.name === channelName);
      const hue = (index * 30) % 360;
      
      if (!channel || !channel.samples || channel.samples.length === 0) {
        return {
          name: channelName,
          color: `hsl(${hue}, 70%, 50%)`,
          samples: [],
          minValue: 0,
          maxValue: 100
        };
      }

      // Process and validate samples
      const validSamples = channel.samples
        .map(sample => {
          // Ensure the timestamp is a valid number
          let timestamp = typeof sample.timestamp === 'number' ? 
            sample.timestamp : Number(sample.timestamp);
          
          // Skip invalid timestamps
          if (isNaN(timestamp) || timestamp <= 0) {
            return null;
          }
          
          // Format to ensure it's in milliseconds range
          if (timestamp > 1000000000000000) { // nanoseconds
            timestamp = Math.floor(timestamp / 1000000);
          } else if (timestamp < 10000000000) { // seconds
            timestamp = timestamp * 1000;
          }
          
          return {
            timestamp: timestamp,
            value: sample.value
          };
        })
        .filter(sample => sample !== null);
      
      // Calculate min/max values for scaling
      const values = validSamples.map(s => s.value);
      const minValue = channel.min_value !== undefined ? 
        channel.min_value : (values.length ? Math.min(...values) : 0);
      const maxValue = channel.max_value !== undefined ? 
        channel.max_value : (values.length ? Math.max(...values) : 100);
      
      return {
        name: channelName,
        color: `hsl(${hue}, 70%, 50%)`,
        samples: validSamples,
        minValue,
        maxValue
      };
    });
  }, [visibleData, channelNames]);

  // Draw function - using canvas for better performance
  const drawChart = useCallback(() => {
    if (!canvasRef.current || processedData.length === 0) {
      return;
    }
    
    // Get current time
    const now = Date.now();
    
    // Throttle drawing to max 30fps for performance
    if (now - lastDrawTime.current < 33) {
      animationFrameRef.current = requestAnimationFrame(drawChart);
      return;
    }
    
    lastDrawTime.current = now;
    
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    
    if (!ctx) return;
    
    // Set canvas dimensions for proper scaling
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);
    
    const width = rect.width;
    const height = rect.height;
    
    // Clear the canvas
    ctx.clearRect(0, 0, width, height);
    
    // Draw background
    ctx.fillStyle = '#f8f9fa';
    ctx.fillRect(0, 0, width, height);
    
    // Draw grid
    const gridColor = '#e0e0e0';
    ctx.strokeStyle = gridColor;
    ctx.lineWidth = 1;
    
    // Vertical grid - time intervals
    const timeRange = 60000; // 60 seconds visible
    const viewStartTime = Math.max(0, currentTime - timeRange/2);
    const viewEndTime = viewStartTime + timeRange;
    
    // Time grid every 5 seconds
    const timeInterval = 5000;
    for (let t = Math.floor(viewStartTime / timeInterval) * timeInterval; t <= viewEndTime; t += timeInterval) {
      const x = ((t - viewStartTime) / timeRange) * width;
      
      // Skip if outside visible area
      if (x < 0 || x > width) continue;
      
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
      
      // Add time label
      ctx.fillStyle = '#666';
      ctx.font = '10px Arial';
      const timeLabel = new Date(t + (recording?.startTime || 0)).toLocaleTimeString([], {
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
      });
      ctx.fillText(timeLabel, x + 2, height - 5);
    }
    
    // Horizontal grid - 4 evenly spaced lines
    for (let i = 1; i <= 3; i++) {
      const y = (i / 4) * height;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }
    
    // No data message if needed
    if (processedData.every(d => d.samples.length === 0)) {
      ctx.fillStyle = '#666';
      ctx.font = '14px Arial';
      ctx.textAlign = 'center';
      ctx.fillText('No data available for the selected channels', width/2, height/2);
      setIsLoading(false);
      return;
    }
    
    // Draw data for each channel
    processedData.forEach(channel => {
      if (channel.samples.length === 0) return;
      
      ctx.strokeStyle = channel.color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      
      // Plot points
      let isFirstPoint = true;
      
      // Adjust timestamps relative to recording start
      const adjustedSamples = channel.samples.map(s => ({
        timestamp: s.timestamp - (recording?.startTime || 0),
        value: s.value
      }));
      
      // Sort by timestamp
      adjustedSamples.sort((a, b) => a.timestamp - b.timestamp);
      
      // Draw points in visible range
      for (const sample of adjustedSamples) {
        // Skip if outside visible range
        if (sample.timestamp < viewStartTime || sample.timestamp > viewEndTime) continue;
        
        // Calculate screen coordinates
        const x = ((sample.timestamp - viewStartTime) / timeRange) * width;
        
        // Scale value to canvas height (inverted Y axis)
        const valueRange = channel.maxValue - channel.minValue;
        const normalizedValue = valueRange > 0 ? 
          (sample.value - channel.minValue) / valueRange : 0.5;
        const y = height - normalizedValue * height;
        
        if (isFirstPoint) {
          ctx.moveTo(x, y);
          isFirstPoint = false;
        } else {
          ctx.lineTo(x, y);
        }
      }
      
      ctx.stroke();
      
      // Add channel label
      ctx.fillStyle = channel.color;
      ctx.font = '12px Arial';
      ctx.textAlign = 'left';
      ctx.fillText(channel.name, 10, 20 + processedData.indexOf(channel) * 20);
    });
    
    // Add current time indicator
    const currentX = ((currentTime - viewStartTime) / timeRange) * width;
    if (currentX >= 0 && currentX <= width) {
      ctx.strokeStyle = '#ff0000';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(currentX, 0);
      ctx.lineTo(currentX, height);
      ctx.stroke();
    }
    
    // Request next frame
    animationFrameRef.current = requestAnimationFrame(drawChart);
    
    // Done loading
    setIsLoading(false);
  }, [processedData, currentTime, recording]);
  
  // Initialize and clean up drawing
  useEffect(() => {
    debugLog(`Setting up graph for ${channelNames.length} channels`);
    
    // Start drawing
    drawChart();
    
    // Clean up
    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
        animationFrameRef.current = null;
      }
    };
  }, [drawChart, channelNames.length]);
  
  // Force redraw when recording changes
  useEffect(() => {
    if (recording?.id) {
      debugLog(`Recording changed to: ${recording.id}`);
      setIsLoading(true);
      // Clear previous animation frame
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
        animationFrameRef.current = null;
      }
      // Schedule new draw
      animationFrameRef.current = requestAnimationFrame(drawChart);
    }
  }, [recording?.id, drawChart]);
  
  // If no channels selected, display a message
  if (channelNames.length === 0) {
    return (
      <Box 
        height={height} 
        display="flex" 
        alignItems="center" 
        justifyContent="center"
        bgcolor="rgba(0,0,0,0.03)"
        borderRadius={1}
      >
        <Typography color="textSecondary">
          No channels selected
        </Typography>
      </Box>
    );
  }
  
  return (
    <Box position="relative" height={height}>
      {isLoading && (
        <Box 
          position="absolute" 
          top="50%" 
          left="50%" 
          sx={{ transform: 'translate(-50%, -50%)', zIndex: 10 }}
        >
          <CircularProgress size={40} />
        </Box>
      )}
      
      <canvas
        ref={canvasRef}
        style={{
          width: '100%',
          height: '100%',
          display: 'block'
        }}
      />
    </Box>
  );
};

// Use React.memo to prevent unnecessary re-renders
export default React.memo(PlaybackGraph);