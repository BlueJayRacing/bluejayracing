import { useBox, useRaycastVehicle } from "@react-three/cannon";
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";
import { DRACOLoader } from "three/examples/jsm/loaders/DRACOLoader";
import { useFrame, useLoader} from '@react-three/fiber';
import "../../../style.css";
import { useRef, useEffect } from 'react';
import { useWheels } from "./useWheels.jsx";
import { WheelDebug } from "./WheelDebug";
import { useControls } from "./useControls";
import RearCorner from "./RearCorner";
import FrontCorner from "./FrontCorner";
import { Vector3, Quaternion } from "three";


function Car(props) {
  
  const { thirdPerson } = props;

  let mesh = useLoader(
    STLLoader,
    // GLTFLoader,
    '../models/lowpolyframe.STL'
    // './3DBenchy.stl'
    // '../models/20XT_render.glb'
  );

  const dracoLoader = new DRACOLoader();
  dracoLoader.setDecoderPath('https://www.gstatic.com/draco/versioned/decoders/1.5.5/');
  dracoLoader.setDecoderConfig({ type: 'js' });
  const gltf = useLoader(GLTFLoader, './public/models/20XT_render.glb', loader => {
    loader.setDRACOLoader(dracoLoader);
  });

	const position = [0, .2, 0];
  
	let scale = [.00025,.00025,.00025];
  
	let rotation = [0, -Math.PI, 0];
  // let scale = [.005,.005,.005];
  // let rotation = [-Math.PI/2,0 , Math.PI/2];


  const width = 0.25;
  const height = 0.1;
  const front = 0.175;
  const wheelRadius = 0.07;

	const chassisBodyArgs = [width, height, 2*front];


	const [chassisBody, chassisApi] = useBox(() => ({
    args: chassisBodyArgs,
    mass: 22500,
    position,
  }), useRef(null));

  const [wheels, wheelInfos] = useWheels(width, height, front, wheelRadius);

  const [vehicle, vehicleApi] = useRaycastVehicle(() => ({
    chassisBody,
    wheelInfos,
    wheels,
    length,
  }), useRef(null));

  useControls(vehicleApi, chassisApi);

  useFrame((state) => {
    if(!thirdPerson) return;

    let position = new Vector3(0,0,0);
    position.setFromMatrixPosition(chassisBody.current.matrixWorld);

    let quaternion = new Quaternion(0, 0, 0, 0);
    quaternion.setFromRotationMatrix(chassisBody.current.matrixWorld);

    let wDir = new Vector3(0, 0, -1);
    wDir.applyQuaternion(quaternion);
    wDir.normalize();

    let cameraPosition = position.clone().add(
      wDir.clone().multiplyScalar(-1).add(
        new Vector3(0, 0.3, 0)
      )
    );

    state.camera.position.copy(cameraPosition);
    state.camera.lookAt(position);
  });

	return (
		<group ref={vehicle} name="vehicle" >
      <group ref={chassisBody} name="chassisBody">
        <group scale={scale} rotation={rotation} position={[0, -.025, 0]} >
          {/* <mesh geometry={mesh}><meshLambertMaterial color="red" /></mesh> */}
          <primitive 
            object={gltf.scene} 
            scale={[35, 35, 35]} // Scale the model to 1.5 times its original size
            rotation={[0, Math.PI/2, 0]} // Rotate the model 45 degrees on the Y-axis
            position={[0, 0, 950]} // Position the model at the origin
          />
        </group>
        <FrontCorner scale={scale} flip={true}/>
        <FrontCorner scale={scale} flip={false}/>
        <RearCorner scale={scale} flip={true}/>
        <RearCorner scale={scale} flip={false}/>
      </group>


			{/* <mesh ref={chassisBody} name="chassisBody">
        <meshBasicMaterial transparent={true} opacity={0.3} />
        <boxGeometry args={chassisBodyArgs} />
        
      </mesh> */}
      
      <WheelDebug wheelRef={wheels[0]} radius={wheelRadius} />
      <WheelDebug wheelRef={wheels[1]} radius={wheelRadius} />
      <WheelDebug wheelRef={wheels[2]} radius={wheelRadius} />
      <WheelDebug wheelRef={wheels[3]} radius={wheelRadius} />

      
      
		</group>
		
	);
}

export default Car;