// SuspensionModel.tsx
import React, { useRef, useEffect, useState } from 'react';
import { useGLTF } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { GLTF } from 'three/examples/jsm/loaders/GLTFLoader';

interface SuspensionModelProps {
  // shockExtension: number;
  // suspensionAngle: number;

  // leftShockExtension: number;
  // rightShockExtension: number;

  leftFrontShockExtension: number;
  leftBackShockExtension: number;
  rightFrontShockExtension: number;
  rightBackShockExtension: number;
}

type GLTFResult = GLTF & {
  nodes: { [key: string]: THREE.Mesh };
  materials: { [key: string]: THREE.Material };
}

const SuspensionModel: React.FC<SuspensionModelProps> = ({ leftFrontShockExtension, leftBackShockExtension, rightFrontShockExtension, rightBackShockExtension }) => {
  const group = useRef<THREE.Group>();
  const { scene, materials } = useGLTF('/models/vehicle.glb') as unknown as GLTFResult;
  //const [suspensionMeshes, setSuspensionMeshes] = useState<THREE.Mesh[]>([]);
  // const [leftSuspensionMeshes, setLeftSuspensionMeshes] = useState<THREE.Mesh[]>([]);
  // const [rightSuspensionMeshes, setRightSuspensionMeshes] = useState<THREE.Mesh[]>([]);

  const [leftFrontSuspensionMeshes, setLeftFrontSuspensionMeshes] = useState<THREE.Mesh[]>([]);
  const [leftBackSuspensionMeshes, setLeftBackSuspensionMeshes] = useState<THREE.Mesh[]>([]);
  const [rightFrontSuspensionMeshes, setRightFrontSuspensionMeshes] = useState<THREE.Mesh[]>([]);
  const [rightBackSuspensionMeshes, setRightBackSuspensionMeshes] = useState<THREE.Mesh[]>([]);

  useEffect(() => {
    const newSuspensionMeshes: THREE.Mesh[] = [];
    const leftSuspensionMesh: THREE.Mesh[] = [];
    const rightSuspensionMesh: THREE.Mesh[] = [];
    const leftFrontSuspensionMesh: THREE.Mesh[] = [];
    const leftBackSuspensionMesh: THREE.Mesh[] = [];

    const rightFrontSuspensionMesh: THREE.Mesh[] = [];
    const rightBackSuspensionMesh: THREE.Mesh[] = [];
    scene.traverse((object) => {
      if (object instanceof THREE.Mesh && object.name.toLowerCase().includes('suspension')) {
        newSuspensionMeshes.push(object);
        if (object.geometry.boundingSphere.center.z > 0) {
          // leftSuspensionMesh.push(object);
          //console.log(object.name);
          if(object.geometry.boundingSphere.center.x < 35) {
            leftFrontSuspensionMesh.push(object);
          } else {
            leftBackSuspensionMesh.push(object);
          }
          
        } else if (object.geometry.boundingSphere.center.z <= 0) {
          // rightSuspensionMesh.push(object);
          if(object.geometry.boundingSphere.center.x < 35) {
            rightFrontSuspensionMesh.push(object);
          } else {
            rightBackSuspensionMesh.push(object);
          }
        }


        // Debug logging
        //console.log(`Suspension Mesh: ${object.name}`);
        //console.log(`Material: ${object.material.name}`);
        //console.log(object.geometry.boundingSphere.center);
        //console.log(`y position: ${object.position.y}`);
        //if (object.material instanceof THREE.MeshStandardMaterial) {
          //console.log(`Base Color Map: ${object.material.map ? 'Yes' : 'No'}`);
        //}
      }
    });
    //setSuspensionMeshes(newSuspensionMeshes);
    // setLeftSuspensionMeshes(leftSuspensionMesh);
    // setRightSuspensionMeshes(rightSuspensionMesh);
    setLeftFrontSuspensionMeshes(leftFrontSuspensionMesh);
    setLeftBackSuspensionMeshes(leftBackSuspensionMesh);
    setRightFrontSuspensionMeshes(rightFrontSuspensionMesh);
    setRightBackSuspensionMeshes(rightBackSuspensionMesh);
  }, [scene]);

  // Load textures
  const texturePromises = Object.values(materials).map(material => {
    if (material instanceof THREE.MeshStandardMaterial && material.map) {
      return new Promise((resolve, reject) => {
        new THREE.TextureLoader().load(
          material.map?.image.src,
          texture => {
            material.map = texture;
            material.needsUpdate = true;
            resolve(texture);
          },
          undefined,
          reject
        );
      });
    }
    return Promise.resolve();
  });

  useEffect(() => {
    Promise.all(texturePromises)
      //.then(() => console.log('All suspension textures loaded successfully'))
      //.catch(error => console.error('Error loading suspension textures:', error));
  }, []);

  useFrame(() => {
    // if (group.current) {
    //   leftSuspensionMeshes.forEach((mesh) => {
    //     const clonedMesh = group.current?.getObjectByName(mesh.name) as THREE.Mesh;
        
    //     if (clonedMesh) {
    //       clonedMesh.position.y = mesh.position.y + leftShockExtension / 100; // Adjust the division factor as needed
    //       //clonedMesh.rotation.x = mesh.rotation.x + suspensionAngle / 100;
    //       //console.log(clonedMesh);

    //     }

    //   });
    // }

    // if (group.current) {
    //   rightSuspensionMeshes.forEach((mesh) => {
    //     const clonedMesh = group.current?.getObjectByName(mesh.name) as THREE.Mesh;
        
    //     if (clonedMesh) {
    //       //clonedMesh.position.y = mesh.position.y + shockExtension / 100; // Adjust the division factor as needed
    //       clonedMesh.rotation.x = mesh.rotation.x + suspensionAngle / 100;
    //       //console.log(clonedMesh);

    //     }

    //   });
    // }
    // if (group.current) {
    //   rightSuspensionMeshes.forEach((mesh) => {
    //     const clonedMesh = group.current?.getObjectByName(mesh.name) as THREE.Mesh;
        
    //     if (clonedMesh) {
    //       clonedMesh.position.y = mesh.position.y + rightShockExtension / 100; // Adjust the division factor as needed
    //       //clonedMesh.rotation.x = mesh.rotation.x + suspensionAngle / 100;
    //       //console.log(clonedMesh);

    //     }

    //   });
    // }

    if (group.current) {
      leftFrontSuspensionMeshes.forEach((mesh) => {
        const clonedMesh = group.current?.getObjectByName(mesh.name) as THREE.Mesh;
        
        if (clonedMesh) {
          clonedMesh.position.y = mesh.position.y + leftFrontShockExtension / 100; // Adjust the division factor as needed
          //clonedMesh.rotation.x = mesh.rotation.x + suspensionAngle / 100;
          //console.log(clonedMesh);

        }

      });
    }

    if (group.current) {
      leftBackSuspensionMeshes.forEach((mesh) => {
        const clonedMesh = group.current?.getObjectByName(mesh.name) as THREE.Mesh;
        
        if (clonedMesh) {
          clonedMesh.position.y = mesh.position.y + leftBackShockExtension / 100; // Adjust the division factor as needed
          //clonedMesh.rotation.x = mesh.rotation.x + suspensionAngle / 100;
          //console.log(clonedMesh);

        }

      });
    }

    if (group.current) {
      rightFrontSuspensionMeshes.forEach((mesh) => {
        const clonedMesh = group.current?.getObjectByName(mesh.name) as THREE.Mesh;
        
        if (clonedMesh) {
          clonedMesh.position.y = mesh.position.y + rightFrontShockExtension / 100; // Adjust the division factor as needed
          //clonedMesh.rotation.x = mesh.rotation.x + suspensionAngle / 100;
          //console.log(clonedMesh);

        }

      });
    }

    if (group.current) {
      rightBackSuspensionMeshes.forEach((mesh) => {
        const clonedMesh = group.current?.getObjectByName(mesh.name) as THREE.Mesh;
        
        if (clonedMesh) {
          clonedMesh.position.y = mesh.position.y + rightBackShockExtension / 100; // Adjust the division factor as needed
          //clonedMesh.rotation.x = mesh.rotation.x + suspensionAngle / 100;
          //console.log(clonedMesh);

        }

      });
    }

  });

  return (
    <group ref={group}>
      {/* {leftSuspensionMeshes.map((mesh) => (
        <primitive key={mesh.name} object={mesh.clone()} />
      ))}
      {rightSuspensionMeshes.map((mesh) => (
        <primitive key={mesh.name} object={mesh.clone()} />
      ))} */}
      {leftFrontSuspensionMeshes.map((mesh) => (
        <primitive key={mesh.name} object={mesh.clone()} />
      ))}
      {leftBackSuspensionMeshes.map((mesh) => (
        <primitive key={mesh.name} object={mesh.clone()} />
      ))}
      {rightFrontSuspensionMeshes.map((mesh) => (
        <primitive key={mesh.name} object={mesh.clone()} />
      ))}
      {rightBackSuspensionMeshes.map((mesh) => (
        <primitive key={mesh.name} object={mesh.clone()} />
      ))}
      
    </group>
  );
};

export default SuspensionModel;