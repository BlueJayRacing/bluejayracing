import React from 'react';
import { useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import { Slider, Box, Switch, FormControlLabel , Button} from '@mui/material';

export interface CameraPositionProps {
  horizontalPosition: number;
  setHorizontalPosition: (value: number) => void;
  verticalPosition: number;
  setVerticalPosition: (value: number) => void;
}

export interface CameraControlProps extends CameraPositionProps{
      isGoingToFront: boolean;
      setIsGoingToFront: (value: boolean) => void;
      isGoingToBack: boolean;
      setIsGoingToBack: (value: boolean) => void;
      isGoingToLeft: boolean;
      setIsGoingToLeft: (value: boolean) => void;
      isGoingToRight: boolean;
      setIsGoingToRight: (value: boolean) => void;
      goToSetPosition: () => void;
      // handlePanToFront: () => void;
      // handlePanToBack: () => void;
      // handlePanToLeft: () => void;
      // handlePanToRight: () => void;
      handleMoveToRequiredPosition: (value: number[]) => void;
}

const CameraControls: React.FC<CameraControlProps> = ({
  horizontalPosition,
  setHorizontalPosition,
  verticalPosition,
  setVerticalPosition,
  setIsGoingToFront,
  setIsGoingToBack,
  setIsGoingToLeft,
  setIsGoingToRight,
  goToSetPosition,
  isGoingToFront,
  handleMoveToRequiredPosition

}) => {
  const handleHorizontalPosition = (event: Event, newValue: number | number[]) => {
    setHorizontalPosition(newValue as number);}
  const handleVerticalPosition = (event: Event, newValue: number |  number[]) => {
    setVerticalPosition(newValue as number);}
    
  // const handlePanToFront = () => {
  //   setIsGoingToFront(true);
  //   console.log(isGoingToFront);
  //   goToSetPosition();
  //   console.log('Front clicked');
    
  // }
  // const handlePanToBack = () => {
  //   setIsGoingToBack(true);
  //   goToSetPosition();
  // }
  // const handlePanToLeft = () => {
  //   setIsGoingToLeft(true);
  //   goToSetPosition();
  // }
  // const handlePanToRight = () => {
  //   setIsGoingToRight(true);
  //   goToSetPosition();
  // }
  return ( 
    <>
    <Box sx ={{width: 300, padding: 2}}>
        <h2>Camera Controls</h2>
          <Slider
            value={horizontalPosition}
            onChange={handleHorizontalPosition}
            min={- Math.PI}
            max={Math.PI}
            step={0.01}
            aria-labelledby="horizontal-rotation-slider"
            valueLabelDisplay="off"
            />
          <Slider
            value={verticalPosition}
            onChange={handleVerticalPosition}
            min={0}
            max={Math.PI}
            step={0.01}
            aria-labelledby="vertical-rotation-slider"
            valueLabelDisplay="off"
            />
          <Box sx ={{display:'flex', width: 300, padding: 1, alignContent:"space-around", justifyContent:'row'}}>
          <Button 
            
            variant = "contained"
            onClick={handleMoveToRequiredPosition([-1.55, 1.5])}
            >Front
          </Button>
          <Button 
            variant = "contained"
            onClick={handleMoveToRequiredPosition([1.55, 1.5])}
          >Back
          </Button>
          <Button 
            variant = "contained"
            onClick={handleMoveToRequiredPosition([0, 1.5])}
          >Left
          </Button>
          <Button 
            variant = "contained"
            onClick={handleMoveToRequiredPosition([3.14, 1.5])}
          >Right
          </Button>
          </Box>
    </Box>
    </>
    

  );
};

export default CameraControls;
