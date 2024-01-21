import {
    ConstraintTypes,
    PublicApi, SpringOptns,
    Triplet,
    useBox, useCylinder,
    useHingeConstraint,
    useRaycastVehicle,
    useSpring
} from "@react-three/cannon";
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader';
import {useFrame, useLoader, useThree} from '@react-three/fiber';
import "../../../style.css";
import {useRef, useEffect, RefObject} from 'react';
import {Vector3, Quaternion, Object3D} from "three";
import {SusLinkInfo, UprightInfo, WheelInfo} from './bajaCarTypes'
import {makeSusLink, makeRearSuspension, makeUpright, makeWheel, makeFrontSuspension} from "./frontSuspension";





function BajaCar(props) {

    const {thirdPerson} = props;


    const width = 0.3;
    const height = 0.5;
    const length = 1.6;
    const wheelRadius = 0.07;

    const position: Triplet = [0, 1.2, 0];

    let rotation: Triplet = [0, -Math.PI, 0];

    const chassisBodyArgs: Triplet = [width, height, length];

    const [chassisBody, chassisApi] = useBox(() => ({
        args: chassisBodyArgs,
        mass: 150,
        position,
    }), useRef(null));



    makeFrontSuspension(chassisBody)



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
        <group name="vehicle">

            <group ref={chassisBody} name="chassisBody">
            </group>

        </group>

    );
}

export default BajaCar;
