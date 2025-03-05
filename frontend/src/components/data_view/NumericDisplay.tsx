// src/components/data_view/NumericDisplay.tsx
import React, { useEffect, useState } from 'react';
import { Channel } from '../shared/types';

interface NumericDisplayProps {
  channels: Channel[];
}

const NumericDisplay: React.FC<NumericDisplayProps> = ({ channels }) => {
  const [values, setValues] = useState<{ [key: string]: number }>({});
  
  // Update values at a rate of 10Hz (100ms)
  useEffect(() => {
    const intervalId = setInterval(() => {
      const newValues: { [key: string]: number } = {};
      
      channels.forEach(channel => {
        const samples = channel.samples;
        if (samples && samples.length > 0) {
          // Get the latest value
          newValues[channel.name] = samples[samples.length - 1].value;
        } else {
          newValues[channel.name] = 0;
        }
      });
      
      setValues(newValues);
    }, 100);
    
    return () => clearInterval(intervalId);
  }, [channels]);

  // Get formatted display value based on channel type
  const getDisplayValue = (channel: Channel): string => {
    const value = values[channel.name] ?? 0;
    
    // Format based on value range and precision needed
    if (value === 0) return '0';
    
    if (Math.abs(value) < 0.01) {
      return value.toExponential(2);
    }
    
    if (Math.abs(value) < 1) {
      return value.toFixed(3);
    }
    
    if (Math.abs(value) < 10) {
      return value.toFixed(2);
    }
    
    if (Math.abs(value) < 100) {
      return value.toFixed(1);
    }
    
    return value.toFixed(0);
  };

  return (
    <div className="space-y-2">
      {channels.map(channel => {
        const value = values[channel.name] ?? 0;
        const displayValue = getDisplayValue(channel);
        
        // Determine color based on value relative to range
        let valueColor = 'text-gray-900';
        if (channel.min_value !== undefined && channel.max_value !== undefined) {
          const range = channel.max_value - channel.min_value;
          const normalizedValue = (value - channel.min_value) / range;
          
          if (normalizedValue > 0.8) valueColor = 'text-red-600';
          else if (normalizedValue > 0.6) valueColor = 'text-orange-500';
          else if (normalizedValue < 0.2) valueColor = 'text-blue-600';
        }
        
        return (
          <div 
            key={channel.name}
            className="flex justify-between items-center p-2 border rounded bg-white hover:bg-gray-50 shadow-sm"
          >
            <div className="text-sm font-medium text-gray-700">{channel.name}</div>
            <div className="text-right">
              <div className={`text-lg font-semibold ${valueColor}`}>
                {displayValue}
              </div>
              <div className="text-xs text-gray-500">
                Range: [{channel.min_value}, {channel.max_value}]
              </div>
            </div>
          </div>
        );
      })}
      
      {channels.length === 0 && (
        <div className="text-gray-500 italic text-center py-4">
          No channels selected
        </div>
      )}
    </div>
  );
};

export default React.memo(NumericDisplay);