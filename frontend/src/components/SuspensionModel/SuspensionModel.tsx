// SuspensionModel.tsx
import React, { useRef, useEffect, useState } from 'react';
import { useGLTF } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { GLTF } from 'three/examples/jsm/loaders/GLTFLoader';

interface SuspensionModelProps {
  shockExtension: number;
}

type GLTFResult = GLTF & {
  nodes: { [key: string]: THREE.Mesh };
  materials: { [key: string]: THREE.Material };
}

const SuspensionModel: React.FC<SuspensionModelProps> = ({ shockExtension }) => {
  const group = useRef<THREE.Group>();
  const { scene, materials } = useGLTF('/models/vehicle.glb') as unknown as GLTFResult;
  const [suspensionMeshes, setSuspensionMeshes] = useState<THREE.Mesh[]>([]);

  useEffect(() => {
    const newSuspensionMeshes: THREE.Mesh[] = [];
    scene.traverse((object) => {
      if (object instanceof THREE.Mesh && object.name.toLowerCase().includes('suspension')) {
        newSuspensionMeshes.push(object);

        // Debug logging
        console.log(`Suspension Mesh: ${object.name}`);
        console.log(`Material: ${object.material.name}`);
        if (object.material instanceof THREE.MeshStandardMaterial) {
          console.log(`Base Color Map: ${object.material.map ? 'Yes' : 'No'}`);
        }
      }
    });
    setSuspensionMeshes(newSuspensionMeshes);
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
      .then(() => console.log('All suspension textures loaded successfully'))
      .catch(error => console.error('Error loading suspension textures:', error));
  }, []);

  useFrame(() => {
    if (group.current) {
      suspensionMeshes.forEach((mesh) => {
        const clonedMesh = group.current?.getObjectByName(mesh.name) as THREE.Mesh;
        if (clonedMesh) {
          clonedMesh.position.y = mesh.position.y + shockExtension / 100; // Adjust the division factor as needed
        }
      });
    }
  });

  return (
    <group ref={group}>
      {suspensionMeshes.map((mesh) => (
        <primitive key={mesh.name} object={mesh.clone()} />
      ))}
    </group>
  );
};

export default SuspensionModel;