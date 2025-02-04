// Car.jsx
import { useBox, useRaycastVehicle } from "@react-three/cannon";
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader';
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";
import { useFrame, useLoader } from '@react-three/fiber';
import { Suspense, useRef, useEffect, useState } from 'react';
import { useWheels } from "./useWheels.jsx";
import { WheelDebug } from "./WheelDebug";
import { useControls } from "./useControls";
import { Vector3, Quaternion, Mesh } from "three";
import { SuspensionGroup } from './SuspensionGroup';

const dracoLoader = new DRACOLoader();
dracoLoader.setDecoderPath('https://www.gstatic.com/draco/versioned/decoders/1.5.6/');
dracoLoader.preload();

const gltfLoader = new GLTFLoader();
gltfLoader.setDRACOLoader(dracoLoader);

function Car({ thirdPerson, adjust }) {
  const gltf = useLoader(
    GLTFLoader,
    '../models/vehicle.glb',
    (loader) => {
      loader.setDRACOLoader(dracoLoader);
    }
  );

  // State for suspension meshes
  const [leftFrontSuspension, setLeftFrontSuspension] = useState([]);
  const [rightFrontSuspension, setRightFrontSuspension] = useState([]);
  const [leftRearSuspension, setLeftRearSuspension] = useState([]);
  const [rightRearSuspension, setRightRearSuspension] = useState([]);
  
  // State for frame meshes
  const [frameMeshes, setFrameMeshes] = useState([]);


  const [front_right_pos, setFrontRightPos] = useState(new Vector3(0, 0, 0));
  const [front_left_pos, setFrontLeftPos] = useState(new Vector3(0, 0, 0));
  const [rear_right_pos, setRearRightPos] = useState(new Vector3(0, 0, 0));
  const [rear_left_pos, setRearLeftPos] = useState(new Vector3(0, 0, 0));
  
  // Parse suspension and frame meshes from model
  useEffect(() => {
    const leftFront = [], rightFront = [], leftRear = [], rightRear = [];
    const frame = [];
    
    gltf.scene.traverse((object) => {
      if (object instanceof Mesh) {
        if (object.name.toLowerCase().includes('suspension')) {
          if (object.name.toLowerCase().includes('006') ) {
            setFrontRightPos(object.geometry.boundingSphere.center)
            console.log(object.geometry.boundingSphere.center)
          } else if (object.name.toLowerCase().includes('178')) {
            setFrontLeftPos(object.geometry.boundingSphere.center)
            console.log(object.geometry.boundingSphere.center)
          } else if (object.name.toLowerCase().includes('008')) {
            setRearLeftPos(object.geometry.boundingSphere.center)
            console.log(object.geometry.boundingSphere.center)
          } else if (object.name.toLowerCase().includes('167')) {
            setRearRightPos(object.geometry.boundingSphere.center)
            console.log(object.geometry.boundingSphere.center)
          }
        }


        if (object.name.toLowerCase().includes('suspension')) {
          const { x, z } = object.geometry.boundingSphere.center;
          
          if (z > 0) { // Left side
            if (x < 35) leftFront.push(object);
            else leftRear.push(object);
          } else { // Right side
            if (x < 35) rightFront.push(object);
            else rightRear.push(object);
          }
        } else {
          // If it's not a suspension mesh, it's part of the frame
          frame.push(object);
        }
      }
    });

    setLeftFrontSuspension(leftFront);
    setRightFrontSuspension(rightFront);
    setLeftRearSuspension(leftRear);
    setRightRearSuspension(rightRear);
    setFrameMeshes(frame);
  }, [gltf]);

  const position = [0, .15, 0];
  const width = 0.25;
  const height = 0.1;
  const front = 0.175;
  const wheelRadius = 0.07;

  const chassisBodyArgs = [width, height, 2 * front];

  const [chassisBody, chassisApi] = useBox(() => ({
    args: chassisBodyArgs,
    mass: 750,
    position,
  }), useRef(null));

  const [wheels, wheelInfos, [w1r, w2r, w3r, w4r]] = useWheels(width, height, front, wheelRadius);
  console.log(w1r)
  const w1_pos = useRef([0, 0, 0])
  const w2_pos = useRef([0, 0, 0])
  const w3_pos = useRef([0, 0, 0])
  const w4_pos = useRef([0, 0, 0])
  const w1_qot = useRef([0, 0, 0, 0])
  const w2_qot = useRef([0, 0, 0, 0])
  const w3_qot = useRef([0, 0, 0, 0])
  const w4_qot = useRef([0, 0, 0, 0])
  useEffect(() => {
    const unsubscribe = w1r.position.subscribe((p) => (w1_pos.current = p))
    return unsubscribe
  }, [])
  useEffect(() => {
    const unsubscribe = w2r.position.subscribe((p) => (w2_pos.current = p))
    return unsubscribe
  }, [])
  useEffect(() => {
    const unsubscribe = w3r.position.subscribe((p) => (w3_pos.current = p))
    return unsubscribe
  }, [])
  useEffect(() => {
    const unsubscribe = w4r.position.subscribe((p) => (w4_pos.current = p))
    return unsubscribe
  }, [])
  useEffect(() => {
    const unsubscribe = w1r.quaternion.subscribe((q) => (w1_qot.current = q))
    return unsubscribe
  }, [])
  useEffect(() => {
    const unsubscribe = w2r.quaternion.subscribe((q) => (w2_qot.current = q))
    return unsubscribe
  }, [])
  useEffect(() => {
    const unsubscribe = w3r.quaternion.subscribe((q) => (w3_qot.current = q))
    return unsubscribe
  }, [])

  console.log(chassisApi)

  const chassis_qot = useRef([0, 0, 0, 0])
  useEffect(() => {
    const unsubscribe = chassisApi.quaternion.subscribe((q) => (chassis_qot.current = q))
    return unsubscribe
  }, [])

  const [vehicle, vehicleApi] = useRaycastVehicle(() => ({
    chassisBody,
    wheelInfos,
    wheels,
  }), useRef(null));

  useControls(vehicleApi, chassisApi);

  // useFrame((state) => {
  //   if (!vehicle.current) return;

  //   // Camera follow logic
  //   if (thirdPerson) {
  //     let pos = new Vector3(0, 0, 0);
  //     pos.setFromMatrixPosition(chassisBody.current.matrixWorld);

  //     let quaternion = new Quaternion(0, 0, 0, 0);
  //     quaternion.setFromRotationMatrix(chassisBody.current.matrixWorld);

  //     let wDir = new Vector3(0, 0, -1);
  //     wDir.applyQuaternion(quaternion);
  //     wDir.normalize();

  //     let cameraPosition = pos.clone().add(
  //       wDir.clone().multiplyScalar(-1).add(
  //         new Vector3(0, 0.3, 0)
  //       )
  //     );

  //     state.camera.position.copy(cameraPosition);
  //     state.camera.lookAt(pos);
  //   }
  // });
  // Camera follow setup
  const followDistance = 2.0;  // Distance behind the car
  const followHeight = 0.75;    // Height above the car
  const followSpeed = 0.1;     // How quickly camera follows (0-1)

  useFrame((state) => {
    if (!vehicle.current || !chassisBody.current) return;

    // Camera follow logic
    if (thirdPerson) {
      // Get current chassis state
      const chassisPos = new Vector3();
      const chassisRot = new Quaternion();
      
      chassisBody.current.getWorldPosition(chassisPos);
      chassisBody.current.getWorldQuaternion(chassisRot);

      // Calculate ideal camera position
      const idealOffset = new Vector3(0, followHeight, followDistance);
      idealOffset.applyQuaternion(chassisRot);
      const idealPos = chassisPos.clone().add(idealOffset);

      // Update camera position with smoothing
      state.camera.position.lerp(idealPos, followSpeed);
      
      // Always look at vehicle
      const lookAtPos = chassisPos.clone().add(new Vector3(0, 0.2, 0));  // Slight offset to look at car's "center"
      state.camera.lookAt(lookAtPos);
      
      // Keep camera level
      state.camera.up.set(0, 2, 0);
    }
  });

  

  useEffect(() => {
    return () => {
      dracoLoader.dispose();
    };
  }, []);

  const scale = [.01, .01, .01];
  const rotation = [0, -Math.PI/2, 0];



  return (
    <Suspense fallback={null}>
      <group ref={vehicle} name="vehicle">
        <group ref={chassisBody} name="chassisBody">
          <group scale={scale} rotation={rotation} position={[0, 0, -.25]}>
            {/* Render frame meshes */}
            {frameMeshes.map((mesh, index) => (
              <primitive key={`frame-${index}`} object={mesh.clone()}/>
            ))}
          </group>
          
          
        </group>

        {/* Suspension Groups */}
        <SuspensionGroup
          meshes={leftFrontSuspension}
          posRef={w1_pos}
          qotRef={chassis_qot}
          diffPos={[-0.15851670762394457, -0.040562102177363245, 0.4257602923760555]}
          wheelIndex={0}
        />
        <SuspensionGroup
          meshes={rightFrontSuspension}
          posRef={w2_pos}
          qotRef={chassis_qot}
          diffPos={[0.1662480202628072, -0.040509979737193134, 0.4261460202628072]}
          wheelIndex={1}
        />
        <SuspensionGroup
          meshes={rightRearSuspension}
          posRef={w3_pos}
          qotRef={chassis_qot}
          diffPos={[-0.1590329797371928, -0.040509979737193134, 0.07445702026280687]}
          wheelIndex={2}
        />
        <SuspensionGroup
          meshes={leftRearSuspension}
          posRef={w4_pos}
          qotRef={chassis_qot}
          diffPos={[0.16646525699485823, -0.0404873430051421, 0.0749262569948579]}
          wheelIndex={3}
        /> 

        <WheelDebug wheelRef={wheels[0]} radius={wheelRadius} refs={leftRearSuspension} />
        <WheelDebug wheelRef={wheels[1]} radius={wheelRadius} />
        <WheelDebug wheelRef={wheels[2]} radius={wheelRadius} />
        <WheelDebug wheelRef={wheels[3]} radius={wheelRadius} />
      </group>
    </Suspense>
  );
}

export default Car;