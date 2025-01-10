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

const PanCameraFront = ({
  horizontalPosition, verticalPosition, 
  setHorizontalPosition, setVerticalPosition, 
  toFront, setToFront

}) => {
  useFrame(() => {
    if(toFront) {
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
        setToFront(false);
      }
    }

  })

  return(
    <></>
  )
}


const PanCameraBack = ({
  horizontalPosition, verticalPosition, 
  setHorizontalPosition, setVerticalPosition, 
  toBack, setToBack

}) => {
  useFrame(() => {
    if(toBack) {
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
        setToBack(false);
      }
    }
  })

  return(
    <></>
  )
}

const PanCameraLeft = ({
  horizontalPosition, verticalPosition, 
  setHorizontalPosition, setVerticalPosition, 
  toLeft, setToLeft

}) => {
  useFrame(() => {
    if(toLeft) {
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
        setToLeft(false);
      }
    }
  })

  return(
    <></>
  )
}

const PanCameraRight = ({
  horizontalPosition, verticalPosition, 
  setHorizontalPosition, setVerticalPosition, 
  toRight, setToRight

}) => {
  useFrame(() => {
    if(toRight) {
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
        setToRight(false);
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
  const [toFront, setToFront] = useState(false);
  const [toBack, setToBack] = useState(false);
  const [toLeft, setToLeft] = useState(false);
  const [toRight, setToRight] = useState(false);
  //const { camera, gl } = useThree();


  const goToBack = () => {
    setToBack(true);
  }

  const goToFront = () => {
    setToFront(true);
  }

  const goToLeft = () => {
    setToLeft(true);
  }

  const goToRight = () => {
    setToRight(true);
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
          toFront={toFront}
          goToFront={goToFront}
          toBack={toBack}
          goToBack={goToBack}
          toLeft={toLeft}
          goToLeft={goToLeft}
          toRight={toRight}
          goToRight={goToRight}
          />
       
      </div>
      <div className="w-3/4 h-full relative" style={{ width: "100%", height: "100%" }}>
        <Canvas
          className="absolute top-0 left-0 w-full h-full"
          camera={{ position: [0, 50, 100], fov: 60 }}
        >

          <Orbit horizontalPosition={horizontalPosition} verticalPosition={verticalPosition} /> 
          <PanCameraFront horizontalPosition={horizontalPosition} 
          verticalPosition={verticalPosition} 
          setHorizontalPosition={setHorizontalPosition} 
          setVerticalPosition={setVerticalPosition}
          toFront={toFront}
          setToFront={setToFront}
          />
         <PanCameraBack horizontalPosition={horizontalPosition} 
          verticalPosition={verticalPosition} 
          setHorizontalPosition={setHorizontalPosition} 
          setVerticalPosition={setVerticalPosition}
          toBack={toBack}
          setToBack={setToBack}
          />
          <PanCameraLeft horizontalPosition={horizontalPosition} 
          verticalPosition={verticalPosition} 
          setHorizontalPosition={setHorizontalPosition} 
          setVerticalPosition={setVerticalPosition}
          toLeft={toLeft}
          setToLeft={setToLeft}
          />
          <PanCameraRight horizontalPosition={horizontalPosition} 
          verticalPosition={verticalPosition} 
          setHorizontalPosition={setHorizontalPosition} 
          setVerticalPosition={setVerticalPosition}
          toRight={toRight}
          setToRight={setToRight}
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