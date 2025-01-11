// HomePage.tsx
import React, { useState, useRef, useEffect, FC} from 'react';
import { Canvas, render } from '@react-three/fiber';
import CarModel from '../components/CarModel/CarModel';
import SuspensionModel from '../components/SuspensionModel/SuspensionModel';
import SuspensionControls from '../components/SuspensionControls/SuspensionControls';
import CameraControls from '../components/CameraControls/CameraControls';
import { OrbitControlProps, Orbit} from '../components/CameraControls/Orbit';
import {PanCamera} from '../components/CameraControls/PanCamera';
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

  // if the Front button is pressed
  const goToFront = () => {
    setIsGoingToFront(true);
  }
  // if the Back button is pressed
  const goToBack = () => {
    setIsGoingToBack(true);
  }
  // if the Left button is pressed
  const goToLeft = () => {
    setIsGoingToLeft(true);
  }
  // if the Right button is pressed
  const goToRight = () => {
    setIsGoingToRight(true);
  }

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
          goToFront={goToFront}
          isGoingToBack={isGoingToBack}
          goToBack={goToBack}
          isGoingToLeft={isGoingToLeft}
          goToLeft={goToLeft}
          isGoingToRight={isGoingToRight}
          goToRight={goToRight}
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
          goToFront={goToFront}
          goToBack={goToBack}
          goToLeft={goToLeft}
          goToRight={goToRight}
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