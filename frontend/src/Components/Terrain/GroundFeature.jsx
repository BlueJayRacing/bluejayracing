import { useTrimesh } from "@react-three/cannon";
import { useLoader } from "@react-three/fiber";
import { useRef } from "react";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";

function GroundFeature({ 
  position = [0, -2, 0], // Default position lower in the scene
  rotation = [0, 0, 0]   // Default flat rotation
}) {
  const result = useLoader(GLTFLoader, "../../../models/penn2_rec.glb");
  const geometry = result.scene.children[0].geometry;
  console.log(result.scene.children)
  console.log(geometry)

  const scale = 0.02;
  const positionAttr = geometry.attributes.position;

  for (let i = 0; i < positionAttr.count; i++) {
    positionAttr.setXYZ(
      i,
      positionAttr.getX(i) * scale,
      positionAttr.getY(i) * scale,
      positionAttr.getZ(i) * scale
    );
  }

  const vertices = geometry.attributes.position.array;
  const indices = geometry.index.array;

  const [ref] = useTrimesh(
    () => ({
      args: [vertices, indices],
      mass: 0,
      type: "Static",
      position: position
    }),
    useRef(null)
  );

  return (
    <group ref={ref} rotation={rotation}>
      <mesh>
        <primitive object={result.scene} geometry={geometry} />
        <meshBasicMaterial color="blue" opacity={0.3} />
      </mesh>
    </group>
  );
}

export default GroundFeature;