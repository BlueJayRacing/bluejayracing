// SuspensionControls.tsx
import React from 'react';
import { Slider, Box, Switch, FormControlLabel } from '@mui/material';

interface SuspensionControlsProps {
  // shockExtension: number;
  // setShockExtension: (value: number) => void;
  showSuspension: boolean;
  setShowSuspension: (value: boolean) => void;
  // suspensionAngle: number;
  // setSuspensionAngle: (value: number) => void;

  // leftShockExtension: number;
  // setLeftShockExtension: (value: number) => void;
  // rightShockExtension: number;
  // setRightShockExtension: (value: number) => void;

  leftFrontShockExtension: number;
  setLeftFrontShockExtension: (value: number) => void;
  leftBackShockExtension: number;
  setLeftBackShockExtension: (value: number) => void;
  rightFrontShockExtension: number;
  setRightFrontShockExtension: (value: number) => void;
  rightBackShockExtension: number;
  setRightBackShockExtension: (value: number) => void;
}

const SuspensionControls: React.FC<SuspensionControlsProps> = ({
  
  // shockExtension,
  // setShockExtension,
  showSuspension,
  setShowSuspension,
  // suspensionAngle,
  // setSuspensionAngle,
  // leftShockExtension,
  // setLeftShockExtension,
  // rightShockExtension,
  // setRightShockExtension,

  leftFrontShockExtension,
  setLeftFrontShockExtension,
  leftBackShockExtension,
  setLeftBackShockExtension,
  rightFrontShockExtension,
  setRightFrontShockExtension,
  rightBackShockExtension,
  setRightBackShockExtension,

}) => {
  // const handleShockExtensionChange = (event: Event, newValue: number | number[]) => {
  //   setShockExtension(newValue as number); }
  // const handleSuspensionAngleChange = (event: Event, newValue: number | number[]) => {
  //   setSuspensionAngle(newValue as number);}
  // const handleLeftShockExtensionChange = (event: Event, newValue: number | number[]) => {
  //   setLeftShockExtension(newValue as number); }
  // const handleRightShockExtensionChange = (event: Event, newValue: number | number[]) => {
  //     setRightShockExtension(newValue as number); }
  const handleLeftFrontShockExtensionChange = (event: Event, newValue: number | number[]) => {
    setLeftFrontShockExtension(newValue as number); }
  const handleLeftBackShockExtensionChange = (event: Event, newValue: number | number[]) => {
    setLeftBackShockExtension(newValue as number); }
  const handleRightFrontShockExtensionChange = (event: Event, newValue: number | number[]) => {
    setRightFrontShockExtension(newValue as number); }
  const handleRightBackShockExtensionChange = (event: Event, newValue: number | number[]) => {
    setRightBackShockExtension(newValue as number); 
  };

  return (
    <Box sx={{ width: 300, padding: 2 }}>
      <h2>Suspension Controls</h2>
      {/* <Slider
        value={shockExtension}
        onChange={handleShockExtensionChange}
        min={-1000}
        max={1000}
        step={1}
        aria-labelledby="shock-extension-slider"
      />
      <Slider
        value={suspensionAngle}
        onChange={handleSuspensionAngleChange}
        min={-1000}
        max={1000}
        step={1}
        aria-labelledby="suspension-angle-slider"
      />

      <Slider
        value={leftShockExtension}
        onChange={handleLeftShockExtensionChange}
        min={-1000}
        max={1000}
        step={1}
        aria-labelledby="left-shock-extension-slider"
      />

      <Slider
        value={rightShockExtension}
        onChange={handleRightShockExtensionChange}
        min={-1000}
        max={1000}
        step={1}
        aria-labelledby="right-shock-extension-slider"
      /> */}

    <Slider
        value={leftFrontShockExtension}
        onChange={handleLeftFrontShockExtensionChange}
        min={-1000}
        max={1000}
        step={1}
        aria-labelledby="left-front-shock-extension-slider"
      />

    <Slider
        value={leftBackShockExtension}
        onChange={handleLeftBackShockExtensionChange}
        min={-1000}
        max={1000}
        step={1}
        aria-labelledby="left-back-shock-extension-slider"
      />

    <Slider
        value={rightFrontShockExtension}
        onChange={handleRightFrontShockExtensionChange}
        min={-1000}
        max={1000}
        step={1}
        aria-labelledby="right-front-shock-extension-slider"
      />

    <Slider
        value={rightBackShockExtension}
        onChange={handleRightBackShockExtensionChange}
        min={-1000}
        max={1000}
        step={1}
        aria-labelledby="right-back-shock-extension-slider"
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