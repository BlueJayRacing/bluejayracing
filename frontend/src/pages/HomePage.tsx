// HomePage.tsx
import React, { useState, useRef, useEffect, FC} from 'react';
import { Canvas, render } from '@react-three/fiber';
import CarModel from '../components/CarModel/CarModel';
import SuspensionModel from '../components/SuspensionModel/SuspensionModel';
import SuspensionControls from '../components/SuspensionControls/SuspensionControls';
import CameraControls from '../components/CameraControls/CameraControls';
import {CameraPositionProps, CameraControlProps} from '../components/CameraControls/CameraControls';
import LightingEffects from '../components/LightingEffects/LightingEffects';
import useSuspensionData from '../hooks/useSuspensionData';
import { OrbitControls } from '@react-three/drei';
import { useThree , useFrame} from '@react-three/fiber';
import { RenderTarget } from 'three';

interface OrbitControlProps extends CameraControlProps {
  setIsGoingToFront: (value: boolean) => void;
  setIsGoingToBack: (value: boolean) => void;
  setIsGoingToLeft: (value: boolean) => void;
  setIsGoingToRight: (value: boolean) => void;
}

const Orbit: FC<CameraPositionProps> = ({horizontalPosition, verticalPosition}) => {
  const { camera, gl } = useThree();
  
  return (
    <OrbitControls
      autoRotate={false}
      maxPolarAngle={verticalPosition}
      minPolarAngle={verticalPosition}
      minAzimuthAngle={horizontalPosition}
      maxAzimuthAngle={horizontalPosition}
      args={[camera, gl.domElement]}
      target={[35, -10, 0]}
      zoom0={0.5}
    />
  );
};

const PanCamera: FC<OrbitControlProps> = ({
  horizontalPosition, verticalPosition, 
  setHorizontalPosition, setVerticalPosition, 
  isGoingToFront, setIsGoingToFront,
  isGoingToBack, setIsGoingToBack,
  isGoingToLeft, setIsGoingToLeft,
  isGoingToRight, setIsGoingToRight

}) => {
  // how much to increment the camera by when panning
  const increment = 0.01;
  // the threshold for how close the camera has to be to
  // a certain position (front, back, left, right), before it
  // is considered at that position
  const threshold = 0.01;
  // the front, back, left, and right view have the same verticalPosition
  const goToPresetVerticalPosition = () => {
    if (verticalPosition > 1.5) {
      setVerticalPosition(verticalPosition - increment);
    } else if (verticalPosition < 1.5) {
      setVerticalPosition(verticalPosition + increment);
    }
  }
  useFrame(() => {
    if(isGoingToFront) {
      if (horizontalPosition > -1.55) {
        setHorizontalPosition((horizontalPosition - increment));
      } else if (horizontalPosition < -1.55) {
        setHorizontalPosition(horizontalPosition + increment);
      } 
      goToPresetVerticalPosition();
      if (Math.abs(horizontalPosition + 1.55) < threshold && Math.abs(verticalPosition - 1.5) < threshold) {
        setIsGoingToFront(false);
      }
    }
    if(isGoingToBack) {
      if (horizontalPosition > 1.55) {
        setHorizontalPosition((horizontalPosition - increment));
      } else if (horizontalPosition < 1.55) {
        setHorizontalPosition(horizontalPosition + increment);
      } 
      goToPresetVerticalPosition();
      if (Math.abs(horizontalPosition - 1.55 ) < threshold && Math.abs(verticalPosition - 1.50) < threshold) {
        setIsGoingToBack(false);
      }
    }
    if(isGoingToLeft) {
      if (horizontalPosition > 0) {
        setHorizontalPosition((horizontalPosition - increment));
      } else if (horizontalPosition < 0) {
        setHorizontalPosition(horizontalPosition + increment);
      } 
      goToPresetVerticalPosition();
      if (Math.abs(horizontalPosition - 0 ) < threshold && Math.abs(verticalPosition - 1.50) < threshold) {
        setIsGoingToLeft(false);
      }
    }
    if(isGoingToRight) {
      if (horizontalPosition > 3.14) {
        setHorizontalPosition((horizontalPosition - increment));
      } else if (horizontalPosition < 3.14) {
        setHorizontalPosition(horizontalPosition + increment);
      } 
      goToPresetVerticalPosition();
      if (Math.abs(horizontalPosition - 3.14 ) < threshold && Math.abs(verticalPosition - 1.50) < threshold) {
        setIsGoingToRight(false);
      }
    }

  })

  return(
    <></>
  )
}


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