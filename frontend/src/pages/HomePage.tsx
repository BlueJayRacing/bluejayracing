// HomePage.tsx
import React, { useState, useRef, useEffect} from 'react';
import { Canvas, render } from '@react-three/fiber';
import CarModel from '../components/CarModel/CarModel';
import SuspensionModel from '../components/SuspensionModel/SuspensionModel';
import SuspensionControls from '../components/SuspensionControls/SuspensionControls';
import CameraControls from '../components/CameraControls/CameraControls';
import LightingEffects from '../components/LightingEffects/LightingEffects';
import useSuspensionData from '../hooks/useSuspensionData';
import { OrbitControls } from '@react-three/drei';
import { useThree , useFrame} from '@react-three/fiber';
import { RenderTarget } from 'three';


const Orbit = ({horizontalPosition, verticalPosition}) => { //add props // pass in car center
  const { camera, gl } = useThree();
  //const xPosition2 = xPosition
  //let origin = {xPosition, yPosition,xPosition2};
  //let origin = [0, 0, -20];
  //let normalizedHorizontalPosition = (horizontalPosition + 2 * Math.PI) / (4 * Math.PI);
  
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

const PanCamera = ({
  horizontalPosition, verticalPosition, 
  setHorizontalPosition, setVerticalPosition, 
  isGoingToFront, setIsGoingToFront,
  isGoingToBack, setIsGoingToBack,
  isGoingToLeft, setIsGoingToLeft,
  isGoingToRight, setIsGoingToRight

}) => {
  useFrame(() => {
    if(isGoingToFront) {
      if (horizontalPosition > -1.55) {
        setHorizontalPosition((horizontalPosition - 0.01));
      } else if (horizontalPosition < -1.55) {
        setHorizontalPosition(horizontalPosition + 0.01);
      } 
      if (verticalPosition > 1.5) {
        setVerticalPosition(verticalPosition - 0.01);
      } else if (verticalPosition < 1.5) {
        setVerticalPosition(verticalPosition + 0.01);
      }
      if (Math.abs(horizontalPosition + 1.55) < 0.01 && Math.abs(verticalPosition - 1.5) < 0.01) {
        setIsGoingToFront(false);
      }
    }
    if(isGoingToBack) {
      if (horizontalPosition > 1.55) {
        setHorizontalPosition((horizontalPosition - 0.01));
      } else if (horizontalPosition < 1.55) {
        setHorizontalPosition(horizontalPosition + 0.01);
      } 
      if (verticalPosition > 1.5) {
        setVerticalPosition(verticalPosition - 0.01);
      } else if (verticalPosition < 1.5) {
        setVerticalPosition(verticalPosition + 0.01);
      }
      if (Math.abs(horizontalPosition - 1.55 ) < 0.01 && Math.abs(verticalPosition - 1.50) < 0.01) {
        setIsGoingToBack(false);
      }
    }
    if(isGoingToLeft) {
      if (horizontalPosition > 0) {
        setHorizontalPosition((horizontalPosition - 0.01));
      } else if (horizontalPosition < 0) {
        setHorizontalPosition(horizontalPosition + 0.01);
      } 
      if (verticalPosition > 1.5) {
        setVerticalPosition(verticalPosition - 0.01);
      } else if (verticalPosition < 1.5) {
        setVerticalPosition(verticalPosition + 0.01);
      }
      if (Math.abs(horizontalPosition - 0 ) < 0.01 && Math.abs(verticalPosition - 1.50) < 0.01) {
        setIsGoingToLeft(false);
      }
    }
    if(isGoingToRight) {
      if (horizontalPosition > 3.14) {
        setHorizontalPosition((horizontalPosition - 0.01));
      } else if (horizontalPosition < 3.14) {
        setHorizontalPosition(horizontalPosition + 0.01);
      } 
      if (verticalPosition > 1.5) {
        setVerticalPosition(verticalPosition - 0.01);
      } else if (verticalPosition < 1.5) {
        setVerticalPosition(verticalPosition + 0.01);
      }
      if (Math.abs(horizontalPosition - 3.14 ) < 0.01 && Math.abs(verticalPosition - 1.50) < 0.01) {
        setIsGoingToRight(false);
      }
    }

  })

  return(
    <></>
  )
}


const HomePage: React.FC = () => {
  // const [shockExtension, setShockExtension] = useState(0);
  const [showSuspension, setShowSuspension] = useState(true);
  // const [suspensionAngle, setSuspensionAngle] = useState(0);
  // const [leftShockExtension, setLeftShockExtension] = useState(0);
  // const [rightShockExtension, setRightShockExtension] = useState(0);

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
  //const { camera, gl } = useThree();

  const goToFront = () => {
    setIsGoingToFront(true);
  }

  const goToBack = () => {
    setIsGoingToBack(true);
  }
  const goToLeft = () => {
    setIsGoingToLeft(true);
  }

  const goToRight = () => {
    setIsGoingToRight(true);
  }

  useSuspensionData(); // Start polling for suspension data
  

  return (
    <div className="flex h-screen" style={{ width: "100vw", height: "100vh" }}>
      <div className="w-1/4 p-4 overflow-y-auto">
        <SuspensionControls
          // shockExtension={shockExtension}
          // setShockExtension={setShockExtension}
          showSuspension={showSuspension}
          setShowSuspension={setShowSuspension}
          // suspensionAngle={suspensionAngle}
          // setSuspensionAngle={setSuspensionAngle}
          // leftShockExtension={leftShockExtension}
          // setLeftShockExtension={setLeftShockExtension}
          // rightShockExtension={rightShockExtension}
          // setRightShockExtension={setRightShockExtension}

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

          <Orbit horizontalPosition={horizontalPosition} verticalPosition={verticalPosition} /> 
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