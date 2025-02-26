// Draw all points regardless of time range - for debugging
const forceDrawAllPoints = true;
// Increase time window to show more data (60 seconds instead of 20)
const timeWindowSeconds = 60;// src/components/data_view/SimpleChartComponent.tsx
// A simpler chart component as fallback
import React, { useEffect, useRef } from 'react';
import { useDataContext } from '../../hooks/useDataContext';
import { safeFormatDate, normalizeTimestamp } from '../../utils/timeDebug';

interface SimpleChartProps {
channelNames: string[];
height?: number;
}

const SimpleChartComponent: React.FC<SimpleChartProps> = ({ 
channelNames, 
height = 300 
}) => {
const canvasRef = useRef<HTMLCanvasElement>(null);
const { channels } = useDataContext();

useEffect(() => {
  if (!canvasRef.current || !channels.length || !channelNames.length) return;
  
  const ctx = canvasRef.current.getContext('2d');
  if (!ctx) return;
  
  // Get selected channels data
  const selectedChannels = channels.filter(channel => 
    channelNames.includes(channel.name)
  );
  
  // Setup canvas
  const canvas = canvasRef.current;
  const width = canvas.width;
  const height = canvas.height;
  
  // Clear canvas
  ctx.clearRect(0, 0, width, height);
  
  // Draw background
  ctx.fillStyle = '#f5f5f5';
  ctx.fillRect(0, 0, width, height);
  
  // Draw grid
  ctx.strokeStyle = '#e0e0e0';
  ctx.lineWidth = 1;
  
  // Vertical grid lines
  for (let x = 50; x < width; x += 50) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  
  // Horizontal grid lines
  for (let y = 50; y < height; y += 50) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
  
  // Time range - show last 20 seconds
  const now = Date.now();
  const timeStart = now - 20000;
  
  // Draw data for each channel
  selectedChannels.forEach((channel, channelIndex) => {
    if (!channel.samples.length) return;
    
    // Set line style
    ctx.strokeStyle = `hsl(${channelIndex * 30}, 70%, 50%)`;
    ctx.lineWidth = 2;
    
    // Find min/max values for scaling
    const minValue = channel.min_value !== undefined ? channel.min_value : 
      Math.min(...channel.samples.map(s => s.value));
    const maxValue = channel.max_value !== undefined ? channel.max_value : 
      Math.max(...channel.samples.map(s => s.value));
    
    // Start drawing the line
    ctx.beginPath();
    
    // Draw each data point
    let pointsDrawn = 0;
    channel.samples.forEach((sample, index) => {
      // Skip points outside time range - Ensure proper numeric conversion
      const sampleTime = normalizeTimestamp(sample.timestamp) || 0;
      const startTime = Number(timeStart);
      const currentTime = Number(now);
      
      if (sampleTime === 0) {
        console.warn(`Invalid timestamp value after normalization: ${sample.timestamp}`);
        return;
      }
      
      // Debug timestamp issues for first few samples
      if (index < 5) {
        try {
          console.log(`Sample ${index} timestamp: ${sampleTime}, formatted: ${safeFormatDate(sampleTime)}`);
        } catch (e) {
          console.log(`Sample ${index} timestamp: ${sampleTime} (not a valid date)`);
        }
      }
      
      // Check if point is in visible time range or bypass check if forceDrawAllPoints is true
      const pointInRange = forceDrawAllPoints || (sampleTime >= startTime && sampleTime <= currentTime);
      
      if (!pointInRange) {
        return;
      }
      
      // Calculate position with error checks
      let x;
      if (forceDrawAllPoints) {
        // Use array index for X position when forcing all points
        x = (index / channel.samples.length) * width;
      } else {
        // Normal time-based position
        const xRatio = (sampleTime - startTime) / (currentTime - startTime);
        if (isNaN(xRatio)) {
          console.warn(`Invalid x ratio: ${xRatio}`);
          return;
        }
        x = xRatio * width;
      }
      const valueRange = maxValue - minValue;
      if (valueRange <= 0) {
        console.warn(`Invalid value range: ${valueRange}`);
        return;
      }
      
      const yRatio = (sample.value - minValue) / valueRange;
      if (isNaN(yRatio)) {
        console.warn(`Invalid y ratio: ${yRatio}`);
        return;
      }
      
      const y = height - yRatio * height;
      
      // if (firstPoint) {
      //   ctx.moveTo(x, y);
      //   firstPoint = false;
      // } else {
      //   ctx.lineTo(x, y);
      // }
      
      pointsDrawn++;
      
      // Draw dots for every 10th point to make sure something is visible
      if (pointsDrawn % 1 === 0) {
        // Save current state
        ctx.save();
        
        // Draw a small red dot
        ctx.fillStyle = 'red';
        ctx.beginPath();
        ctx.arc(x, y, 3, 0, Math.PI * 2);
        ctx.fill();
        
        // Restore state
        ctx.restore();
      }
    });
    
    console.log(`Drew ${pointsDrawn} points for ${channel.name}`);
    
    // Stroke the line
    ctx.stroke();
    
    // Draw label
    ctx.fillStyle = ctx.strokeStyle;
    ctx.font = '12px Arial';
    ctx.fillText(channel.name, 10, 20 + channelIndex * 20);
  });
  
  // Draw time labels
  ctx.fillStyle = '#666';
  ctx.font = '10px Arial';
  
  const dateFormat = new Intl.DateTimeFormat('en-US', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    hour12: false
  });
  
  // Current time for reference
  const currentTime = Date.now();
  
  // Show labels every 5 seconds
  for (let t = 0; t <= 20000; t += 5000) {
    const x = (t / 20000) * width;
    const labelTime = new Date(currentTime - (20000 - t));
    ctx.fillText(dateFormat.format(labelTime), x, height - 5);
  }
}, [channels, channelNames]);

// Re-render the chart every 100ms
useEffect(() => {
  const intervalId = setInterval(() => {
    // Force re-render by updating a ref
    if (canvasRef.current) {
      const event = new Event('update');
      canvasRef.current.dispatchEvent(event);
    }
  }, 100);
  
  return () => clearInterval(intervalId);
}, []);

return (
  <div style={{ height: `${height}px`, width: '100%', position: 'relative' }}>
    <canvas 
      ref={canvasRef} 
      height={height} 
      width={800}
      style={{ width: '100%', height: '100%' }}
    />
    {!channelNames.length && (
      <div style={{
        position: 'absolute',
        top: 0,
        left: 0,
        width: '100%',
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        color: '#999'
      }}>
        Select channels to display
      </div>
    )}
  </div>
);
};

export default SimpleChartComponent;