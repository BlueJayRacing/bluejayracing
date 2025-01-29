import { createRoot } from "react-dom/client";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Stars } from "@react-three/drei";
import { Physics } from "@react-three/cannon";
import "../../style.css";
import { CameraOverlay } from "./Views/CameraPositionOverlay";
import { Stats } from '@react-three/drei'

import Car from "./Car/Car.jsx";
import Terrain from "./Terrain/Terrain.jsx";
import Perspective from "./Views/Perspective";
import GroundFeature from "./Terrain/GroundFeature"; // Adjust path as needed

import { useState, useEffect } from "react";

function World() {
  const [thirdPerson, setThirdPerson] = useState(false);
  const [cameraPosition, setCameraPosition] = useState([-2, -.7, .2]);

  // Rotation states for the ground feature
  const [rotX, setRotX] = useState(0); 
  const [rotY, setRotY] = useState(0);
  const [rotZ, setRotZ] = useState(0);

  useEffect(() => {
    function keydownHandler(e) {
      if (e.key === "k") {
        if (thirdPerson) {
          // Small change to camera position so toggling is noticeable
          setCameraPosition([-2, -.7, .2 + Math.random() * 0.01]);
        }
        setThirdPerson(!thirdPerson);
      }
    }
    window.addEventListener("keydown", keydownHandler);
    return () => window.removeEventListener("keydown", keydownHandler);
  }, [thirdPerson]);





	const handlePositionChange = (newPosition) => {
    setCameraPosition(newPosition);
    // Update the actual camera position if needed
    if (cameraRef.current) {
      cameraRef.current.position.set(...newPosition);
    }
  };







  return (
    <>
      {/* Sliders overlaying the Canvas */}
      <div
        style={{
          position: "absolute",
          top: 10,
          right: 10,
          backgroundColor: "rgba(255, 255, 255, 0.8)",
          padding: "8px",
          borderRadius: "4px",
          zIndex: 999
        }}
      >
        <div>
          <label>X Rotation</label>
          <input
            type="range"
            min={-Math.PI/4}
            max={Math.PI/4}
            step={0.00001}
            value={rotX}
            onChange={(e) => setRotX(parseFloat(e.target.value))}
          />
        </div>

        <div>
          <label>Y Rotation</label>
          <input
            type="range"
            min={-Math.PI/4}
            max={Math.PI/4}
            step={0.00001}
            value={rotY}
            onChange={(e) => setRotY(parseFloat(e.target.value))}
          />
        </div>

        <div>
          <label>Z Rotation</label>
          <input
            type="range"
            min={-Math.PI/4}
            max={Math.PI/4}
            step={0.00001}
            value={rotZ}
            onChange={(e) => setRotZ(parseFloat(e.target.value))}
          />
        </div>
				{/* <CameraOverlay 
        position={cameraPosition} 
        onPositionChange={handlePositionChange}
      /> */}
      </div>
			

      <Canvas camera={{ fov: 75, position: [5, 5, -5] }} frameloop="demand" dpr={2}>
				<Stats showPanel={0} className="stats" />  
        <Stars />
        <ambientLight intensity={1} />
        <spotLight position={[100, 15, 10]} angle={0.3} />

        <Perspective cameraPosition={cameraPosition} thirdPerson={thirdPerson} />

        <Physics broadphase="SAP" gravity={[0, -2.6, 0]}>
          <Car thirdPerson={thirdPerson} adjust={[rotX, rotY, rotZ]} />
          <Terrain />

          {/* GroundFeature with sliders controlling rotation */}
          {/* <GroundFeature 
  					position={[0, -2, ]}
  					rotation={[0.543407346410207, 0.115407346410207, 0.093407346410207]} // 30 degrees around X axis
					/> */}
        </Physics>
      </Canvas>
    </>
  );
}

export default World;
