// CarModel.tsx
import React, { useRef, useEffect, useState } from 'react';
import { useGLTF, useTexture } from '@react-three/drei';
import * as THREE from 'three';
import { GLTF } from 'three/examples/jsm/loaders/GLTFLoader';

interface CarModelProps {
  showSuspension: boolean;
}

type GLTFResult = GLTF & {
  nodes: { [key: string]: THREE.Mesh };
  materials: { [key: string]: THREE.Material };
}

const CarModel: React.FC<CarModelProps> = ({ showSuspension }) => {
  const group = useRef<THREE.Group>();
  const { scene, materials } = useGLTF('/models/vehicle.glb') as unknown as GLTFResult;
  const [meshes, setMeshes] = useState<{ [key: string]: THREE.Mesh }>({});

  useEffect(() => {
    const newMeshes: { [key: string]: THREE.Mesh } = {};
    scene.traverse((object) => {
      if (object instanceof THREE.Mesh) {
        const isSuspension = object.name.toLowerCase().includes('suspension');
        const isFrame = object.name.toLowerCase().includes('frame');
        if (isFrame || (isSuspension && showSuspension)) {
          newMeshes[object.name] = object;

          // Debug logging
          console.log(`Mesh: ${object.name}`);
          console.log(`Material: ${object.material.name}`);
          if (object.material instanceof THREE.MeshStandardMaterial) {
            console.log(`Base Color Map: ${object.material.map ? 'Yes' : 'No'}`);
          }
        }
      }
    });
    setMeshes(newMeshes);
  }, [scene, showSuspension]);

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
      .then(() => console.log('All textures loaded successfully'))
      .catch(error => console.error('Error loading textures:', error));
  }, []);

  return (
    <group ref={group}>
      {Object.values(meshes).map((mesh) => (
        <primitive key={mesh.name} object={mesh.clone()} />
      ))}
    </group>
  );
};

export default CarModel;