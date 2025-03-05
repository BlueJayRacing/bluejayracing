import { useMemo } from 'react';
import { SuspensionState } from '../contexts/SuspensionContext';
import { getControlArmAngle } from '../utils/mappingUtils';

const use3DTransforms = (suspensionData: SuspensionState) => {
  return useMemo(() => {
    const { shockExtension } = suspensionData;

    // Calculate control arm angle using the mapping utility
    const controlArmAngle = getControlArmAngle(shockExtension);

    // Calculate positions and rotations for different components
    const wheelPosition = [0, shockExtension, 0];
    const controlArmRotation = [0, 0, controlArmAngle];

    return {
      wheelPosition,
      controlArmRotation,
      // Add other transformations as needed
    };
  }, [suspensionData]);
};

export default use3DTransforms;
