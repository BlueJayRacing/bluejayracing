// SuspensionControls.tsx
import React from 'react';
import { Slider, Box, Switch, FormControlLabel } from '@mui/material';

interface SuspensionControlsProps {
  shockExtension: number;
  setShockExtension: (value: number) => void;
  showSuspension: boolean;
  setShowSuspension: (value: boolean) => void;
}

const SuspensionControls: React.FC<SuspensionControlsProps> = ({
  shockExtension,
  setShockExtension,
  showSuspension,
  setShowSuspension,
}) => {
  const handleShockExtensionChange = (event: Event, newValue: number | number[]) => {
    setShockExtension(newValue as number);
  };

  return (
    <Box sx={{ width: 300, padding: 2 }}>
      <h2>Suspension Controls</h2>
      <Slider
        value={shockExtension}
        onChange={handleShockExtensionChange}
        min={-1000}
        max={1000}
        step={1}
        aria-labelledby="shock-extension-slider"
      />
      <FormControlLabel
        control={
          <Switch
            checked={showSuspension}
            onChange={(e) => setShowSuspension(e.target.checked)}
          />
        }
        label="Show Suspension"
      />
    </Box>
  );
};

export default SuspensionControls;