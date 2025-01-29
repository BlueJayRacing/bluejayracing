import { Vector3, Quaternion, Euler, MathUtils } from "three";
import { useFrame } from '@react-three/fiber';
import { useRef, useEffect } from 'react';

// Function to rotate a position vector by a quaternion
function rotatePositionByQuaternion(position, quaternion) {
  // Create a Vector3 from the position
  const positionVector = new Vector3(position[0], position[1], position[2]);
  
  // Create a pure quaternion from the position (w = 0)
  const positionQuaternion = new Quaternion(positionVector.x, positionVector.y, positionVector.z, 0);
  
  // Calculate q * p * q^(-1)
  const conjugate = quaternion.clone().conjugate();
  const rotatedQuaternion = quaternion.clone()
    .multiply(positionQuaternion)
    .multiply(conjugate);
  
  // Extract the rotated position
  return new Vector3(rotatedQuaternion.x, rotatedQuaternion.y, rotatedQuaternion.z);
}

// Function to eliminate roll (rotation around X axis)
function eliminateRoll(quaternion) {
  const euler = new Euler();
  euler.setFromQuaternion(quaternion, 'XYZ');
  
  // Create new euler with roll set to 0
  const newEuler = new Euler(0, euler.y, euler.z, 'XYZ');
  const newQuaternion = new Quaternion();
  newQuaternion.setFromEuler(newEuler);
  
  return newQuaternion;
}

// Function to eliminate pitch (rotation around Y axis)
function eliminatePitch(quaternion) {
  const euler = new Euler();
  euler.setFromQuaternion(quaternion, 'XYZ');
  
  // Create new euler with pitch set to 0
  const newEuler = new Euler(euler.x, 0, euler.z, 'XYZ');
  const newQuaternion = new Quaternion();
  newQuaternion.setFromEuler(newEuler);
  
  return newQuaternion;
}

// Function to eliminate yaw (rotation around Z axis)
function eliminateYaw(quaternion) {
  const euler = new Euler();
  euler.setFromQuaternion(quaternion, 'XYZ');
  
  // Create new euler with yaw set to 0
  const newEuler = new Euler(euler.x, euler.y, 0, 'XYZ');
  const newQuaternion = new Quaternion();
  newQuaternion.setFromEuler(newEuler);
  
  return newQuaternion;
}

function addEulerToQuaternion(quaternion, roll = 0, pitch = 0, yaw = 0) {
  // Create a quaternion from the Euler angles
  const eulerRotation = new Euler(roll, pitch, yaw, 'XYZ');
  const rotationQuaternion = new Quaternion();
  rotationQuaternion.setFromEuler(eulerRotation);
  
  // Multiply the original quaternion by the rotation quaternion
  const resultQuaternion = quaternion.clone();
  resultQuaternion.multiply(rotationQuaternion);
  
  return resultQuaternion;
}

export function SuspensionGroup({ 
  meshes, 
  posRef,
  qotRef,
  diffPos,
  scale = [.007, .007, .007],
  rotation = [0, -Math.PI/2, 0],
  wheelIndex
}) {
  const groupRef = useRef();
  const meshRefs = useRef([]);

  useEffect(() => {
    meshRefs.current = meshes.map(mesh => mesh.clone());
  }, [meshes]);

  useFrame(() => {
    if (!groupRef.current || !posRef.current) return;

    const [x, y, z] = posRef.current;
    const [q0, q1, q2, q3] = qotRef.current;
    const quaternion = new Quaternion(q0, q1, q2, q3);

    let rectified_quaternion = new Quaternion();
    rectified_quaternion = addEulerToQuaternion(quaternion, 0, -Math.PI/2, 0)
    
    meshRefs.current.forEach(mesh => {
      mesh.scale.set(...scale);
      mesh.rotation.set(...rotation);
      

      let rotatedDiffPos = new Vector3();
      // Calculate rotated position difference
      rotatedDiffPos = rotatePositionByQuaternion(diffPos, quaternion);
      
      
      // Apply position with rotated offset
      mesh.position.set(
        x - rotatedDiffPos.x,
        y - rotatedDiffPos.y,
        z - rotatedDiffPos.z
      );
      


      // Apply wheel quaternion
      mesh.quaternion.copy(rectified_quaternion);

    });
  });

  return (
    <group ref={groupRef}>
      {meshRefs.current.map((mesh, index) => (
        <primitive key={`${mesh.name}-${index}`} object={mesh} />
      ))}
    </group>
  );
}