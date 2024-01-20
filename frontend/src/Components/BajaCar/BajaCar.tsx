import { useBox, useRaycastVehicle } from "@react-three/cannon";
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import { useFrame, useLoader} from '@react-three/fiber';
import "../../../style.css";
import { useRef, useEffect } from 'react';
import { Vector3, Quaternion } from "three";


function BajaCar(props) {

    const { thirdPerson } = props;


    const width = 0.6;
    const height = 1;
    const length = 1.6;
    const wheelRadius = 0.07;

    const position = [0, .5, 0];


    let rotation = [0, -Math.PI, 0];

    const chassisBodyArgs = [width, height, length];


    const [chassisBody, chassisApi] = useBox(() => ({
        args: chassisBodyArgs,
        mass: 750,
        position,
    }), useRef(null));



    // const [wheels, wheelInfos] = useWheels(width, height, front, wheelRadius);
    //
    // const [vehicle, vehicleApi] = useRaycastVehicle(() => ({
    //     chassisBody,
    //     wheelInfos,
    //     wheels,
    //     length,
    // }), useRef(null));
    //
    // useControls(vehicleApi, chassisApi);
    //
    // useFrame((state) => {
    //     if(!thirdPerson) return;
    //
    //     let position = new Vector3(0,0,0);
    //     position.setFromMatrixPosition(chassisBody.current.matrixWorld);
    //
    //     let quaternion = new Quaternion(0, 0, 0, 0);
    //     quaternion.setFromRotationMatrix(chassisBody.current.matrixWorld);
    //
    //     let wDir = new Vector3(0, 0, -1);
    //     wDir.applyQuaternion(quaternion);
    //     wDir.normalize();
    //
    //     let cameraPosition = position.clone().add(
    //         wDir.clone().multiplyScalar(-1).add(
    //             new Vector3(0, 0.3, 0)
    //         )
    //     );
    //
    //     state.camera.position.copy(cameraPosition);
    //     state.camera.lookAt(position);
    // });
    //
    return (
            <group ref={chassisBody} name="chassisBody">

            </group>

    );
}

export default BajaCar;
