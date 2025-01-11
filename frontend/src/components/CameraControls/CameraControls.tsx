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
      goToFront: (value: boolean) => void;
      isGoingToBack: boolean;
      goToBack: (value: boolean) => void;
      isGoingToLeft: boolean;
      goToLeft: (value: boolean) => void;
      isGoingToRight: boolean;
      goToRight: (value: boolean) => void;

}

const CameraControls: React.FC<CameraControlProps> = ({
  horizontalPosition,
  setHorizontalPosition,
  verticalPosition,
  setVerticalPosition,
  goToFront,
  goToBack,
  goToLeft,
  goToRight,

}) => {
  //const { camera, gl } = useThree();
  const handleHorizontalPosition = (event: Event, newValue: number | number[]) => {
    setHorizontalPosition(newValue as number);}
  const handleVerticalPosition = (event: Event, newValue: number |  number[]) => {
    setVerticalPosition(newValue as number);}
  const handlePanToFront = () => {goToFront(false);}
  const handlePanToBack = () => {goToBack(false);}
  const handlePanToLeft = () => {goToLeft(false);}
  const handlePanToRight = () => {goToRight(false);}
  return ( 
    <>
    {/* <OrbitControls args={[camera, gl.domElement]} />; */}
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
            onClick={handlePanToFront}
            >Front
          </Button>
          <Button 
            variant = "contained"
            onClick={handlePanToBack}
          >Back
          </Button>
          <Button 
            variant = "contained"
            onClick={handlePanToLeft}
          >Left
          </Button>
          <Button 
            variant = "contained"
            onClick={handlePanToRight}
          >Right
          </Button>
          </Box>
    </Box>
    </>
    

  );
};

export default CameraControls;
