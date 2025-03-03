// File: src/components/car_terrain_scene/ct_utils/gltfParser.ts

import { Object3D, Mesh, Vector3, Group } from 'three';
import { CarConfig } from '../ct_configs/carConfig';

export interface ParsedMesh {
  mesh: Mesh;
  originalPosition: [number, number, number];
  originalRotation: [number, number, number];
  boundingSphere: {
    center: Vector3;
    radius: number;
  };
}

export interface ParsedCarComponents {
  suspension: {
    frontLeft: ParsedMesh[];
    frontRight: ParsedMesh[];
    rearLeft: ParsedMesh[];
    rearRight: ParsedMesh[];
  };
  frame: ParsedMesh[];
  drivetrain: {
    frontLeft: ParsedMesh[];
    frontRight: ParsedMesh[];
    rearLeft: ParsedMesh[];
    rearRight: ParsedMesh[];
  };
}

function createEmptyParsedComponents(): ParsedCarComponents {
  return {
    suspension: {
      frontLeft: [],
      frontRight: [],
      rearLeft: [],
      rearRight: []
    },
    frame: [],
    drivetrain: {
      frontLeft: [],
      frontRight: [],
      rearLeft: [],
      rearRight: []
    }
  };
}

function extractMeshData(mesh: Mesh): ParsedMesh | null {
  if (!mesh) return null;
  
  try {
    mesh.geometry.computeBoundingSphere();
    const clonedMesh = mesh.clone();
    return {
      mesh: clonedMesh,
      originalPosition: [
        mesh.position.x,
        mesh.position.y,
        mesh.position.z
      ],
      originalRotation: [
        mesh.rotation.x,
        mesh.rotation.y,
        mesh.rotation.z
      ],
      boundingSphere: {
        center: mesh.geometry.boundingSphere?.center.clone() || new Vector3(),
        radius: mesh.geometry.boundingSphere?.radius || 0
      }
    };
  } catch (error) {
    console.error('Error extracting mesh data:', error);
    return null;
  }
}

function classifyCornerByPosition(mesh: Mesh): string {
  mesh.geometry.computeBoundingSphere();
  const { x, z } = mesh.geometry.boundingSphere.center;
  const frontThreshold = 35;
  
  if (z > 0) { // Left side
    if (x < frontThreshold) return 'frontLeft';
    return 'rearLeft';
  } else { // Right side
    if (x < frontThreshold) return 'frontRight';
    return 'rearRight';
  }
}

export function parseCarGLTF(scene: THREE.Group | null): ParsedCarComponents {
  const result = createEmptyParsedComponents();

  if (!scene) {
    console.warn('No scene provided to parseCarGLTF');
    return result;
  }

  try {
    const clonedScene = scene.clone(true);

    clonedScene.traverse((object: Object3D) => {
      if (!(object instanceof Mesh)) return;

      const meshData = extractMeshData(object);
      if (!meshData) return;

      const name = object.name.toLowerCase();

      // First check for exact wheel names
      if (name.includes('suspension')) {
        if (name.includes('008')) {
          result.drivetrain.frontLeft.push(meshData);
          return;
        }
        if (name.includes('006')) {
          result.drivetrain.frontRight.push(meshData);
          return;
        }
        if (name.includes('178')) {
          result.drivetrain.rearLeft.push(meshData);
          return;
        }
        if (name.includes('167')) {
          result.drivetrain.rearRight.push(meshData);
          return;
        }
        if (name.includes('00678 ')) {
          // Single rear wheel - add to both sides
          result.drivetrain.rearLeft.push(meshData);
          const rightMeshData = extractMeshData(object);
          if (rightMeshData) {
            result.drivetrain.rearRight.push(rightMeshData);
          }
          return;
        }
      }
      

      // Then check for suspension components
      if (name.includes('suspension')) {
        const corner = classifyCornerByPosition(object);
        result.suspension[corner].push(meshData);
        return;
      }

      // Finally, classify as frame
      if (name.includes('frame') || 
          (!name.includes('suspension') && 
           !name.includes('front_left') && 
           !name.includes('front_right') && 
           !name.includes('rear_left') && 
           !name.includes('rear_right') && 
           !name.includes('rear'))) {
        result.frame.push(meshData);
        return;
      }
    });
  } catch (error) {
    console.error('Error parsing car scene:', error);
  }

  return result;
}

export function parseTerrainGLTF(scene: Group | null): ParsedMesh[] {
  const meshes: ParsedMesh[] = [];

  if (!scene) {
    console.warn('No scene provided to parseTerrainGLTF');
    return meshes;
  }

  try {
    const clonedScene = scene.clone();

    clonedScene.traverse((object: Object3D) => {
      if (object instanceof Mesh) {
        const meshData = extractMeshData(object);
        if (meshData) {
          meshes.push(meshData);
        }
      }
    });
  } catch (error) {
    console.error('Error parsing terrain GLTF:', error);
  }

  return meshes;
}