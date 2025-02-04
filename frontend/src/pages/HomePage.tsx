// HomePage.tsx
import React, { useState, useRef, useEffect, FC} from 'react';
import { Canvas, render } from '@react-three/fiber';
import CarModel from '../components/CarModel/CarModel';
import SuspensionModel from '../components/SuspensionModel/SuspensionModel';
import SuspensionControls from '../components/SuspensionControls/SuspensionControls';
import CameraControls from '../components/CameraControls/CameraControls';
import { OrbitControlProps, Orbit} from '../components/CameraControls/Orbit';
import {PanCamera, PanningProps, RequiredPosition} from '../components/CameraControls/PanCamera';
import LightingEffects from '../components/LightingEffects/LightingEffects';
import useSuspensionData from '../hooks/useSuspensionData';
import { useThree , useFrame} from '@react-three/fiber';
import { RenderTarget } from 'three';


const HomePage: React.FC = () => {
  const [showSuspension, setShowSuspension] = useState(true);
  const [leftFrontShockExtension, setLeftFrontShockExtension] = useState(0);
  const [leftBackShockExtension, setLeftBackShockExtension] = useState(0);
  const [rightFrontShockExtension, setRightFrontShockExtension] = useState(0);
  const [rightBackShockExtension, setRightBackShockExtension] = useState(0);
  const [horizontalPosition, setHorizontalPosition] = useState(0);
  const [verticalPosition, setVerticalPosition] = useState(0);
  // isGoingToFront is true when the camera is currently panning to
  // the front of the car, occurs when the "Front" button is pressed.
  // setIsGoingToFront sets isGoingToFront true when the "Front" button
  // is pressed, and is false when the camera has finished moving or does.
  // Likewise for the other isGoingTo___ and setIsGoingTo___
  const [isGoingToFront, setIsGoingToFront] = useState(false);
  const [isGoingToBack, setIsGoingToBack] = useState(false);
  const [isGoingToLeft, setIsGoingToLeft] = useState(false);
  const [isGoingToRight, setIsGoingToRight] = useState(false);
  const [requiredHorizontalPosition, setRequiredHorizontalPosition] = useState(0);
  const [requiredVerticalPosition, setRequiredVerticalPosition] = useState(0);
  //const [requiredPosition, setRequiredPosition] = useState<RequiredPosition[]>([]);
  const [isGoingToRequiredPosition, setIsGoingToRequiredPosition] = useState(false);
  const [increment, setIncrement] = useState(0);
  const [threshold, setThreshold] = useState(0);

  
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

  const handleMoveToRequiredPosition = (newRequiredPosition:number[]) => {
    //useEffect( () => {
      setRequiredHorizontalPosition(newRequiredPosition[0])
      setRequiredVerticalPosition(newRequiredPosition[1]);
      console.log(newRequiredPosition[0]);
      //setRequiredPosition(requiredPosition);
      setIsGoingToRequiredPosition(true);
      goToSetPosition();
    //}, newRequiredPosition)
    
  }

  const goToSetPosition = () => {
    // how much to increment the camera by when panning
    setIncrement(0.01);
    // the threshold for how close the camera has to be to
    // a certain position (front, back, left, right), before it
    //is considered at that position
    setThreshold(0.01);
    if(isGoingToRequiredPosition) {
      //setRequiredHorizontalPosition(requiredPosition[0]);
      //etRequiredVerticalPosition(requiredPosition[1]);
      if (Math.abs(horizontalPosition - requiredHorizontalPosition) < threshold && Math.abs(verticalPosition - requiredVerticalPosition) < threshold) {
        setIsGoingToRequiredPosition(false);
      }
    }
  }

  // const goToSetPosition = () => {
  //   // how much to increment the camera by when panning
  //   setIncrement(0.01);
  //   // the threshold for how close the camera has to be to
  //   // a certain position (front, back, left, right), before it
  //   // is considered at that position
  //   const threshold = 0.01;
  //   if(isGoingToFront) {
  //     console.log('front was pressed');
  //     setRequiredHorizontalPosition(-1.55);
  //     setRequiredVerticalPosition(1.5);
  //     console.log(requiredHorizontalPosition);
  //     //console.log(Math.abs(horizontalPosition - requiredHorizontalPosition));
  //     if (Math.abs(horizontalPosition - requiredHorizontalPosition) < threshold && Math.abs(verticalPosition - requiredVerticalPosition) < threshold) {
        
  //       setIsGoingToFront(false);
  //       console.log('front has been set to false');
  //     }
  //   } else if(isGoingToBack) {
  //     console.log('back was pressed');
  //     setRequiredHorizontalPosition(1.55);
  //     setRequiredVerticalPosition(1.5);
  //     if (Math.abs(horizontalPosition - requiredHorizontalPosition) < threshold && Math.abs(verticalPosition - requiredVerticalPosition) < threshold) {
  //       setIsGoingToBack(false);
  //     }
  //   } else if(isGoingToLeft) {
  //     console.log('left was pressed');
  //     setRequiredHorizontalPosition(0);
  //     setRequiredVerticalPosition(1.5);
  //     if (Math.abs(horizontalPosition - requiredHorizontalPosition) < threshold && Math.abs(verticalPosition - requiredVerticalPosition) < threshold) {
  //       setIsGoingToLeft(false);
  //     }
  //   } else if (isGoingToRight) {
  //     console.log('right was pressed');
  //     setRequiredHorizontalPosition(3.14);
  //     setRequiredVerticalPosition(1.5);
  //     if (Math.abs(horizontalPosition - requiredHorizontalPosition) < threshold && Math.abs(verticalPosition - requiredVerticalPosition) < threshold) {
  //       setIsGoingToRight(false);
  //     }
  //   }
  // }

  useSuspensionData(); // Start polling for suspension data
  
  return (
    <div className="flex h-screen" style={{ width: "100vw", height: "100vh" }}>
      <div className="w-1/4 p-4 overflow-y-auto">
        <SuspensionControls
          showSuspension={showSuspension}
          setShowSuspension={setShowSuspension}
          leftFrontShockExtension={leftFrontShockExtension}
          setLeftFrontShockExtension={setLeftFrontShockExtension}
          leftBackShockExtension={leftBackShockExtension}
          setLeftBackShockExtension={setLeftBackShockExtension}
          rightFrontShockExtension={rightFrontShockExtension}
          setRightFrontShockExtension={setRightFrontShockExtension}
          rightBackShockExtension={rightBackShockExtension}
          setRightBackShockExtension={setRightBackShockExtension}
        />
        <CameraControls horizontalPosition={horizontalPosition} 
          setHorizontalPosition={setHorizontalPosition}
          verticalPosition={verticalPosition}
          setVerticalPosition={setVerticalPosition}
          isGoingToFront={isGoingToFront}
          setIsGoingToFront={setIsGoingToFront}
          isGoingToBack={isGoingToBack}
          setIsGoingToBack={setIsGoingToBack}
          isGoingToLeft={isGoingToLeft}
          setIsGoingToLeft={setIsGoingToLeft}
          isGoingToRight={isGoingToRight}
          setIsGoingToRight={setIsGoingToRight}
          goToSetPosition={goToSetPosition}
          // handlePanToBack={handlePanToBack}
          // handlePanToFront={handlePanToFront}
          // handlePanToLeft={handlePanToLeft}
          // handlePanToRight={handlePanToRight}
          handleMoveToRequiredPosition={handleMoveToRequiredPosition}
          />
       
      </div>
      <div className="w-3/4 h-full relative" style={{ width: "100%", height: "100%" }}>
        <Canvas
          className="absolute top-0 left-0 w-full h-full"
          camera={{ position: [0, 50, 100], fov: 60 }}
        >
          <Orbit horizontalPosition={horizontalPosition} verticalPosition={verticalPosition} 
          setHorizontalPosition={setHorizontalPosition} setVerticalPosition={setVerticalPosition}/> 
          <PanCamera horizontalPosition={horizontalPosition} 
          verticalPosition={verticalPosition} 
          setHorizontalPosition={setHorizontalPosition} 
          setVerticalPosition={setVerticalPosition}
          isGoingToFront={isGoingToFront}
          setIsGoingToFront={setIsGoingToFront}
          isGoingToBack={isGoingToBack}
          setIsGoingToBack={setIsGoingToBack}
          isGoingToLeft={isGoingToLeft}
          setIsGoingToLeft={setIsGoingToLeft}
          isGoingToRight={isGoingToRight}
          setIsGoingToRight={setIsGoingToRight}
          isGoingToRequiredPosition={isGoingToRequiredPosition}
          goToSetPosition={goToSetPosition}
          requiredHorizontalPosition={requiredHorizontalPosition}
          setRequiredHorizontalPosition={setRequiredHorizontalPosition}
          requiredVerticalPosition={requiredVerticalPosition}
          setRequiredVerticalPosition={setRequiredVerticalPosition}
          threshold={threshold}
          setThreshold={setThreshold}
          increment={increment}
          setIncrement={setIncrement}
          // handlePanToBack={handlePanToBack}
          // handlePanToFront={handlePanToFront}
          // handlePanToLeft={handlePanToLeft}
          // handlePanToRight={handlePanToRight}
          handleMoveToRequiredPosition={handleMoveToRequiredPosition}
          />
          <LightingEffects />
          <CarModel showSuspension={!showSuspension} />
          {showSuspension && <SuspensionModel leftFrontShockExtension={leftFrontShockExtension}
          leftBackShockExtension={leftBackShockExtension} rightFrontShockExtension={rightFrontShockExtension}
          rightBackShockExtension={rightBackShockExtension}/>}
        </Canvas>
      </div>
    </div>
  );
};

export default HomePage;