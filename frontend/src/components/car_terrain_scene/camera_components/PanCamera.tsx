// File: src/components/car_terrain_scene/camera_components/PanCamera.tsx

import React, { useEffect } from 'react';
import { useFrame } from '@react-three/fiber';

interface PanCameraProps {
  horizontalPosition: number;
  verticalPosition: number;
  setHorizontalPosition: (value: number) => void;
  setVerticalPosition: (value: number) => void;
  requiredHorizontalPosition: number;
  requiredVerticalPosition: number;
  isGoingToRequiredPosition: boolean;
  increment: number;
  threshold: number;
  onPositionReached?: () => void;
}

const PanCamera: React.FC<PanCameraProps> = ({
  horizontalPosition,
  verticalPosition,
  setHorizontalPosition,
  setVerticalPosition,
  requiredHorizontalPosition,
  requiredVerticalPosition,
  isGoingToRequiredPosition,
  increment,
  threshold,
  onPositionReached
}) => {
  // Handle smooth camera transitions
  useFrame(() => {
    if (!isGoingToRequiredPosition) return;

    const horizontalDiff = requiredHorizontalPosition - horizontalPosition;
    const verticalDiff = requiredVerticalPosition - verticalPosition;

    // Check if we've reached the target position within threshold
    if (Math.abs(horizontalDiff) < threshold && Math.abs(verticalDiff) < threshold) {
      onPositionReached?.();
      return;
    }

    // Update horizontal position with smooth interpolation
    if (Math.abs(horizontalDiff) >= threshold) {
      setHorizontalPosition(
        horizontalPosition + Math.sign(horizontalDiff) * Math.min(Math.abs(horizontalDiff), increment)
      );
    }

    // Update vertical position with smooth interpolation
    if (Math.abs(verticalDiff) >= threshold) {
      setVerticalPosition(
        verticalPosition + Math.sign(verticalDiff) * Math.min(Math.abs(verticalDiff), increment)
      );
    }
  });

  // Apply initial position on mount
  useEffect(() => {
    if (!isGoingToRequiredPosition) {
      setHorizontalPosition(horizontalPosition);
      setVerticalPosition(verticalPosition);
    }
  }, []);

  // This is a utility component that doesn't render anything visible
  return null;
};

export default React.memo(PanCamera);

// Helper type for preset camera positions
export interface CameraPreset {
  horizontal: number;
  vertical: number;
  name: string;
}

// Common camera presets
export const CAMERA_PRESETS: Record<string, CameraPreset> = {
  FRONT: { horizontal: -1.55, vertical: 1.5, name: 'Front' },
  BACK: { horizontal: 1.55, vertical: 1.5, name: 'Back' },
  LEFT: { horizontal: 0, vertical: 1.5, name: 'Left' },
  RIGHT: { horizontal: Math.PI, vertical: 1.5, name: 'Right' },
  TOP: { horizontal: 0, vertical: 0.1, name: 'Top' },
  ISOMETRIC: { horizontal: -Math.PI / 4, vertical: Math.PI / 4, name: 'Isometric' },
};