import React from 'react';
import { useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';

const CameraControls: React.FC = () => {
  const { camera, gl } = useThree();

  return <OrbitControls args={[camera, gl.domElement]} />;
};

export default CameraControls;
